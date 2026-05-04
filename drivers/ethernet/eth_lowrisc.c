/*
 * Copyright (c) 2024 ETH Zurich and University of Bologna
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ethernet driver for the LowRISC 100MHz Ethernet IP core.
 * Ported from the Linux driver by Jonathan Kimmitt.
 *
 * This IP has:
 *  - Memory-mapped TX buffer at offset 0x1000
 *  - Memory-mapped RX buffers at offset 0x4000 (8 buffers, 2KB each)
 *  - 64-bit register access
 *  - MDIO bitbang interface for PHY management
 *  - Single RX interrupt
 */

#define DT_DRV_COMPAT lowrisc_eth

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/byteorder.h>

#include <ethernet/eth_stats.h>

LOG_MODULE_REGISTER(eth_lowrisc, CONFIG_ETHERNET_LOG_LEVEL);

/*
 * ============================================================
 * Register map (from lowrisc_100MHz.h)
 * All registers are 64-bit aligned.
 * ============================================================
 */
#define TXBUFF_OFFSET       0x1000
#define MACLO_OFFSET        0x0800
#define MACHI_OFFSET        0x0808
#define TPLR_OFFSET         0x0810
#define TFCS_OFFSET         0x0818
#define MDIOCTRL_OFFSET     0x0820
#define RFCS_OFFSET         0x0828
#define RSR_OFFSET          0x0830
#define RBAD_OFFSET         0x0838
#define RPLR_OFFSET         0x0840
#define RXBUFF_OFFSET       0x4000

/* MACHI register bits */
#define MACHI_MACADDR_MASK    0x0000FFFF
#define MACHI_LOOPBACK_MASK   0x00020000
#define MACHI_ALLPKTS_MASK    0x00400000
#define MACHI_IRQ_EN          0x00800000

/* TPLR register bits */
#define TPLR_BUSY_MASK        0x80000000
#define TPLR_PACKET_LEN_MASK  0x00000FFF

/* RSR register bits */
#define RSR_RECV_FIRST_MASK   0x0000000F
#define RSR_RECV_DONE_MASK    0x00001000

/* RX buffer geometry: 8 buffers, each 2KB (256 x 8 bytes) */
#define RX_BUF_WORDS          256

/* Timeouts */
#define TX_BUSY_TIMEOUT_US    100000
#define TX_BUSY_POLL_US       10

/*
 * ============================================================
 * Driver data structures
 * ============================================================
 */

struct eth_lowrisc_config {
	mem_addr_t base;
	void (*irq_config_func)(const struct device *dev);
	uint8_t mac_addr[6];
};

struct eth_lowrisc_dev_data {
	struct net_if *iface;
	uint8_t mac_addr[6];
	struct k_sem tx_sem;
	struct k_sem rx_sem;

	K_KERNEL_STACK_MEMBER(rx_thread_stack,
			      CONFIG_ETH_LOWRISC_RX_THREAD_STACK_SIZE);
	struct k_thread rx_thread;
};

/*
 * ============================================================
 * Low-level register access (64-bit MMIO)
 * ============================================================
 */

static inline void eth_reg_write(mem_addr_t base, uint32_t offset, uint64_t val)
{
	sys_write64(val, base + offset);
}

static inline uint64_t eth_reg_read(mem_addr_t base, uint32_t offset)
{
	return sys_read64(base + offset);
}

static void eth_copy_to_txbuf(mem_addr_t base, const uint8_t *data, size_t len)
{
	size_t rnd = ((len - 1) | 7) + 1;
	mem_addr_t txbuf = base + TXBUFF_OFFSET;

	if (((uintptr_t)data & 7) == 0) {
		const uint64_t *src = (const uint64_t *)data;

		for (size_t i = 0; i < rnd / 8; i++) {
			sys_write64(src[i], txbuf + i * 8);
		}
	} else {
		for (size_t i = 0; i < rnd / 8; i++) {
			uint64_t tmp;

			memcpy(&tmp, data + (i * 8), sizeof(uint64_t));
			sys_write64(tmp, txbuf + i * 8);
		}
	}
}

static void eth_copy_from_rxbuf(mem_addr_t base, uint8_t *data, size_t len,
				int buf_idx)
{
	size_t rnd = ((len - 1) | 7) + 1;
	mem_addr_t rxbuf = base + RXBUFF_OFFSET + (buf_idx & 7) * (RX_BUF_WORDS * 8);

	if (((uintptr_t)data & 7) == 0) {
		uint64_t *dst = (uint64_t *)data;

		for (size_t i = 0; i < rnd / 8; i++) {
			dst[i] = sys_read64(rxbuf + i * 8);
		}
	} else {
		for (size_t i = 0; i < rnd / 8; i++) {
			uint64_t tmp = sys_read64(rxbuf + i * 8);

			memcpy(data + (i * 8), &tmp, sizeof(uint64_t));
		}
	}
}

/*
 * ============================================================
 * IRQ enable / disable
 * ============================================================
 */

static void eth_lowrisc_irq_enable(mem_addr_t base)
{
	uint64_t val = eth_reg_read(base, MACHI_OFFSET);

	val |= MACHI_IRQ_EN;
	eth_reg_write(base, MACHI_OFFSET, val);
}

static void eth_lowrisc_irq_disable(mem_addr_t base)
{
	uint64_t val = eth_reg_read(base, MACHI_OFFSET);

	val &= ~MACHI_IRQ_EN;
	eth_reg_write(base, MACHI_OFFSET, val);
}

/*
 * ============================================================
 * MAC address management
 * ============================================================
 */

static void eth_lowrisc_write_mac(mem_addr_t base, const uint8_t *addr)
{
	uint32_t maclo;
	uint16_t machi;

	memcpy(&maclo, addr + 2, 4);
	machi = 0;
	memcpy(&machi, addr, 2);

	eth_reg_write(base, MACLO_OFFSET, sys_cpu_to_be32(maclo));

	/* Read-modify-write MACHI to preserve control bits (IRQ_EN, etc.) */
	uint64_t machi_reg = eth_reg_read(base, MACHI_OFFSET);
	machi_reg &= ~(uint64_t)MACHI_MACADDR_MASK;
	machi_reg |= sys_cpu_to_be16(machi);
	eth_reg_write(base, MACHI_OFFSET, machi_reg);

	/* Configure 8 RX buffers */
	eth_reg_write(base, RFCS_OFFSET, 8);
}

static void eth_lowrisc_read_mac(mem_addr_t base, uint8_t *addr)
{
	uint32_t maclo, machi;

	machi = sys_be16_to_cpu((uint16_t)(eth_reg_read(base, MACHI_OFFSET) &
					    MACHI_MACADDR_MASK));
	maclo = sys_be32_to_cpu((uint32_t)eth_reg_read(base, MACLO_OFFSET));

	memcpy(addr, &machi, 2);
	memcpy(addr + 2, &maclo, 4);
}

/*
 * ============================================================
 * RX processing
 * ============================================================
 */

// 

static void eth_lowrisc_rx(const struct device *dev)
{
	const struct eth_lowrisc_config *cfg = dev->config;
	struct eth_lowrisc_dev_data *data = dev->data;
	mem_addr_t base = cfg->base;
	uint32_t rsr;
	int buf;
	int budget = 8;

	rsr = (uint32_t)eth_reg_read(base, RSR_OFFSET);
	buf = rsr & RSR_RECV_FIRST_MASK;

	while ((rsr & RSR_RECV_DONE_MASK) && budget-- > 0) {
		int len = (int)eth_reg_read(base, RPLR_OFFSET +
					    ((buf & 7) << 3)) - 4;

		if (len >= 60 && len <= NET_ETH_MAX_FRAME_SIZE) {
			struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(
				data->iface, len, AF_UNSPEC, 0, K_NO_WAIT);
			if (pkt != NULL) {
				uint8_t *buf_ptr = net_buf_add(pkt->buffer, len);
				eth_copy_from_rxbuf(base, buf_ptr, len, buf);

				if (net_recv_data(data->iface, pkt) < 0) {
					net_pkt_unref(pkt);
				}
			}
		}

		eth_reg_write(base, RSR_OFFSET, (uint64_t)(++buf));
		rsr = (uint32_t)eth_reg_read(base, RSR_OFFSET);
	}
}

static void eth_lowrisc_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct eth_lowrisc_config *cfg = dev->config;
	struct eth_lowrisc_dev_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&data->rx_sem, K_FOREVER);
		eth_lowrisc_rx(dev);
		eth_lowrisc_irq_enable(cfg->base);
	}
}

/*
 * ============================================================
 * ISR
 * ============================================================
 */

static void eth_lowrisc_isr(const struct device *dev)
{
	const struct eth_lowrisc_config *cfg = dev->config;
	struct eth_lowrisc_dev_data *data = dev->data;

	eth_lowrisc_irq_disable(cfg->base);
	k_sem_give(&data->rx_sem);
}

/*
 * ============================================================
 * TX
 * ============================================================
 */

static int eth_lowrisc_send(const struct device *dev, struct net_pkt *pkt)
{
	const struct eth_lowrisc_config *cfg = dev->config;
	struct eth_lowrisc_dev_data *data = dev->data;
	mem_addr_t base = cfg->base;
	size_t len = net_pkt_get_len(pkt);
	uint8_t tx_frame[NET_ETH_MAX_FRAME_SIZE];
	uint64_t tplr;
	int ret = 0;

	if (len > NET_ETH_MAX_FRAME_SIZE) {
		LOG_ERR("TX frame too large: %zu", len);
		return -EINVAL;
	}

	k_sem_take(&data->tx_sem, K_FOREVER);

	tplr = eth_reg_read(base, TPLR_OFFSET);
	if (tplr & TPLR_BUSY_MASK) {
		bool ready = false;

		for (int i = 0; i < TX_BUSY_TIMEOUT_US / TX_BUSY_POLL_US; i++) {
			k_busy_wait(TX_BUSY_POLL_US);
			tplr = eth_reg_read(base, TPLR_OFFSET);
			if (!(tplr & TPLR_BUSY_MASK)) {
				ready = true;
				break;
			}
		}
		if (!ready) {
			LOG_ERR("TX timeout");
			ret = -ETIMEDOUT;
			goto out;
		}
	}

	if (net_pkt_read(pkt, tx_frame, len)) {
		LOG_ERR("Failed to read TX pkt");
		ret = -EIO;
		goto out;
	}

	eth_copy_to_txbuf(base, tx_frame, len);
	eth_reg_write(base, TPLR_OFFSET, (uint64_t)len);

out:
	k_sem_give(&data->tx_sem);
	return ret;
}

/*
 * ============================================================
 * Capabilities & config
 * ============================================================
 */

static enum ethernet_hw_caps eth_lowrisc_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_100BASE;
}

static int eth_lowrisc_set_config(const struct device *dev,
				  enum ethernet_config_type type,
				  const struct ethernet_config *config)
{
	const struct eth_lowrisc_config *cfg = dev->config;
	struct eth_lowrisc_dev_data *data = dev->data;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(data->mac_addr, config->mac_address.addr, 6);
		eth_lowrisc_write_mac(cfg->base, data->mac_addr);
		net_if_set_link_addr(data->iface, data->mac_addr,
				     sizeof(data->mac_addr),
				     NET_LINK_ETHERNET);
		return 0;
	default:
		return -ENOTSUP;
	}
}

/*
 * ============================================================
 * Interface init
 * ============================================================
 */

static void eth_lowrisc_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_lowrisc_dev_data *data = dev->data;

	data->iface = iface;

	net_if_set_link_addr(iface, data->mac_addr,
			     sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);

	ethernet_init(iface);
	net_if_carrier_on(iface);
}

/*
 * ============================================================
 * Device init
 * ============================================================
 */

static int eth_lowrisc_init(const struct device *dev)
{
	const struct eth_lowrisc_config *cfg = dev->config;
	struct eth_lowrisc_dev_data *data = dev->data;
	mem_addr_t base = cfg->base;

	k_sem_init(&data->tx_sem, 1, 1);
	k_sem_init(&data->rx_sem, 0, 1);

	/* Load MAC address from devicetree or read from HW */
	if (cfg->mac_addr[0] != 0 || cfg->mac_addr[1] != 0 ||
	    cfg->mac_addr[2] != 0 || cfg->mac_addr[3] != 0 ||
	    cfg->mac_addr[4] != 0 || cfg->mac_addr[5] != 0) {
		memcpy(data->mac_addr, cfg->mac_addr, 6);
	} else {
		eth_lowrisc_read_mac(base, data->mac_addr);
	}

	eth_lowrisc_write_mac(base, data->mac_addr);

	LOG_INF("MAC: %02x:%02x:%02x:%02x:%02x:%02x",
		data->mac_addr[0], data->mac_addr[1],
		data->mac_addr[2], data->mac_addr[3],
		data->mac_addr[4], data->mac_addr[5]);

	/* Create RX thread */
	k_tid_t tid = k_thread_create(
		&data->rx_thread,
		data->rx_thread_stack,
		K_KERNEL_STACK_SIZEOF(data->rx_thread_stack),
		eth_lowrisc_rx_thread,
		(void *)dev, NULL, NULL,
		CONFIG_ETH_LOWRISC_RX_THREAD_PRIORITY,
		0, K_NO_WAIT);
	k_thread_name_set(tid, "eth_lowrisc_rx");

	/* Configure and enable the interrupt */
	cfg->irq_config_func(dev);
	eth_lowrisc_irq_enable(base);

	LOG_INF("LowRISC Ethernet initialized at 0x%lx", (unsigned long)base);

	return 0;
}

/*
 * ============================================================
 * Ethernet API
 * ============================================================
 */

static const struct ethernet_api eth_lowrisc_api = {
	.iface_api.init     = eth_lowrisc_iface_init,
	.get_capabilities   = eth_lowrisc_caps,
	.set_config         = eth_lowrisc_set_config,
	.send               = eth_lowrisc_send,
};

/*
 * ============================================================
 * Device instantiation macros
 * ============================================================
 */

#define ETH_LOWRISC_IRQ_CONFIG(n)                                           \
	static void eth_lowrisc_irq_config_##n(const struct device *dev)    \
	{                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n),                               \
			    DT_INST_IRQ(n, priority),                       \
			    eth_lowrisc_isr,                                \
			    DEVICE_DT_INST_GET(n),                          \
			    0);                                             \
		irq_enable(DT_INST_IRQN(n));                               \
	}

#define ETH_LOWRISC_MAC_INIT(n)                                             \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, local_mac_address),           \
		    (.mac_addr = DT_INST_PROP(n, local_mac_address),),      \
		    (.mac_addr = {0},))

#define ETH_LOWRISC_INIT(n)                                                 \
	ETH_LOWRISC_IRQ_CONFIG(n)                                           \
                                                                            \
	static struct eth_lowrisc_dev_data eth_lowrisc_data_##n;            \
                                                                            \
	static const struct eth_lowrisc_config eth_lowrisc_cfg_##n = {      \
		.base = DT_INST_REG_ADDR(n),                                \
		.irq_config_func = eth_lowrisc_irq_config_##n,              \
		ETH_LOWRISC_MAC_INIT(n)                                     \
	};                                                                  \
                                                                            \
	ETH_NET_DEVICE_DT_INST_DEFINE(n,                                    \
				      eth_lowrisc_init,                      \
				      NULL,                                  \
				      &eth_lowrisc_data_##n,                 \
				      &eth_lowrisc_cfg_##n,                  \
				      CONFIG_ETH_INIT_PRIORITY,              \
				      &eth_lowrisc_api,                      \
				      NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_LOWRISC_INIT)