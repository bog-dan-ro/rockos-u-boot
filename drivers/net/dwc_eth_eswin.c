// SPDX-License-Identifier: GPL-2.0
/*
 * ESWIN eth driver
 *
 * Copyright 2024, Beijing ESWIN Computing Technology Co., Ltd.. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Authors: LiFei Fang <fanglifei@eswincomputing.com>
 */

#include <linux/kernel.h>
#include <clk.h>
#include <cpu_func.h>
#include <dm.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <miiphy.h>
#include <net.h>
#include <netdev.h>
#include <phy.h>
#include <reset.h>
#include <wait_bit.h>
#include <asm/cache.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <eth_phy.h>
#include <linux/bitops.h>
#include <linux/delay.h>

/* Core registers */

#define EQOS_MAC_REGS_BASE 0x000

struct eqos_mac_regs {
    uint32_t configuration;                    /* 0x000 */
    uint32_t unused_004[(0x070 - 0x004) / 4];  /* 0x004 */
    uint32_t q0_tx_flow_ctrl;                  /* 0x070 */
    uint32_t unused_070[(0x090 - 0x074) / 4];  /* 0x074 */
    uint32_t rx_flow_ctrl;                     /* 0x090 */
    uint32_t unused_094;                       /* 0x094 */
    uint32_t txq_prty_map0;                    /* 0x098 */
    uint32_t unused_09c;                       /* 0x09c */
    uint32_t rxq_ctrl0;                        /* 0x0a0 */
    uint32_t unused_0a4;                       /* 0x0a4 */
    uint32_t rxq_ctrl2;                        /* 0x0a8 */
    uint32_t unused_0ac[(0x0dc - 0x0ac) / 4];  /* 0x0ac */
    uint32_t us_tic_counter;                   /* 0x0dc */
    uint32_t unused_0e0[(0x11c - 0x0e0) / 4];  /* 0x0e0 */
    uint32_t hw_feature0;                      /* 0x11c */
    uint32_t hw_feature1;                      /* 0x120 */
    uint32_t hw_feature2;                      /* 0x124 */
    uint32_t unused_128[(0x200 - 0x128) / 4];  /* 0x128 */
    uint32_t mdio_address;                     /* 0x200 */
    uint32_t mdio_data;                        /* 0x204 */
    uint32_t unused_208[(0x300 - 0x208) / 4];  /* 0x208 */
    uint32_t address0_high;                    /* 0x300 */
    uint32_t address0_low;                     /* 0x304 */
};

#define EQOS_MAC_CONFIGURATION_GPSLCE           BIT(23)
#define EQOS_MAC_CONFIGURATION_CST              BIT(21)
#define EQOS_MAC_CONFIGURATION_ACS              BIT(20)
#define EQOS_MAC_CONFIGURATION_WD               BIT(19)
#define EQOS_MAC_CONFIGURATION_JD               BIT(17)
#define EQOS_MAC_CONFIGURATION_JE               BIT(16)
#define EQOS_MAC_CONFIGURATION_PS               BIT(15)
#define EQOS_MAC_CONFIGURATION_FES              BIT(14)
#define EQOS_MAC_CONFIGURATION_DM               BIT(13)
#define EQOS_MAC_CONFIGURATION_LM               BIT(12)
#define EQOS_MAC_CONFIGURATION_TE               BIT(1)
#define EQOS_MAC_CONFIGURATION_RE               BIT(0)

#define EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT       16
#define EQOS_MAC_Q0_TX_FLOW_CTRL_PT_MASK        0xffff
#define EQOS_MAC_Q0_TX_FLOW_CTRL_TFE            BIT(1)

#define EQOS_MAC_RX_FLOW_CTRL_RFE               BIT(0)

#define EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_SHIFT      0
#define EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_MASK       0xff

#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT         0
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_MASK          3
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_NOT_ENABLED   0
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB   2
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_AV    1

#define EQOS_MAC_RXQ_CTRL2_PSRQ0_SHIFT          0
#define EQOS_MAC_RXQ_CTRL2_PSRQ0_MASK           0xff

#define EQOS_MAC_HW_FEATURE0_MMCSEL_SHIFT       8
#define EQOS_MAC_HW_FEATURE0_HDSEL_SHIFT        2
#define EQOS_MAC_HW_FEATURE0_GMIISEL_SHIFT      1
#define EQOS_MAC_HW_FEATURE0_MIISEL_SHIFT       0

#define EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT   6
#define EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK    0x1f
#define EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT   0
#define EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK    0x1f

#define EQOS_MAC_HW_FEATURE3_ASP_SHIFT          28
#define EQOS_MAC_HW_FEATURE3_ASP_MASK           0x3

#define EQOS_MAC_MDIO_ADDRESS_PA_SHIFT          21
#define EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT         16
#define EQOS_MAC_MDIO_ADDRESS_CR_SHIFT          8
#define EQOS_MAC_MDIO_ADDRESS_CR_20_35          2
#define EQOS_MAC_MDIO_ADDRESS_CR_250_300        5
#define EQOS_MAC_MDIO_ADDRESS_SKAP              BIT(4)
#define EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT         2
#define EQOS_MAC_MDIO_ADDRESS_GOC_READ          3
#define EQOS_MAC_MDIO_ADDRESS_GOC_WRITE         1
#define EQOS_MAC_MDIO_ADDRESS_C45E              BIT(1)
#define EQOS_MAC_MDIO_ADDRESS_GB                BIT(0)

#define EQOS_MAC_MDIO_DATA_GD_MASK              0xffff

#define EQOS_MTL_REGS_BASE                      0xd00
struct eqos_mtl_regs {
    uint32_t txq0_operation_mode;               /* 0xd00 */
    uint32_t unused_d04;                        /* 0xd04 */
    uint32_t txq0_debug;                        /* 0xd08 */
    uint32_t unused_d0c[(0xd18 - 0xd0c) / 4];   /* 0xd0c */
    uint32_t txq0_quantum_weight;               /* 0xd18 */
    uint32_t unused_d1c[(0xd30 - 0xd1c) / 4];   /* 0xd1c */
    uint32_t rxq0_operation_mode;               /* 0xd30 */
    uint32_t unused_d34;                        /* 0xd34 */
    uint32_t rxq0_debug;                        /* 0xd38 */
};

#define EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT      16
#define EQOS_MTL_TXQ0_OPERATION_MODE_TQS_MASK       0x1ff
#define EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT    2
#define EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_MASK     3
#define EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_ENABLED  2
#define EQOS_MTL_TXQ0_OPERATION_MODE_TSF            BIT(1)
#define EQOS_MTL_TXQ0_OPERATION_MODE_FTQ            BIT(0)

#define EQOS_MTL_TXQ0_DEBUG_TXQSTS                  BIT(4)
#define EQOS_MTL_TXQ0_DEBUG_TRCSTS_SHIFT            1
#define EQOS_MTL_TXQ0_DEBUG_TRCSTS_MASK             3

#define EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT      20
#define EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK       0x3ff
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT      14
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFD_MASK       0x3f
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT      8
#define EQOS_MTL_RXQ0_OPERATION_MODE_RFA_MASK       0x3f
#define EQOS_MTL_RXQ0_OPERATION_MODE_EHFC           BIT(7)
#define EQOS_MTL_RXQ0_OPERATION_MODE_RSF            BIT(5)
#define EQOS_MTL_RXQ0_OPERATION_MODE_FEP            BIT(4)
#define EQOS_MTL_RXQ0_OPERATION_MODE_FUP            BIT(3)

#define EQOS_MTL_RXQ0_DEBUG_PRXQ_SHIFT              16
#define EQOS_MTL_RXQ0_DEBUG_PRXQ_MASK               0x7fff
#define EQOS_MTL_RXQ0_DEBUG_RXQSTS_SHIFT            4
#define EQOS_MTL_RXQ0_DEBUG_RXQSTS_MASK             3

#define EQOS_DMA_REGS_BASE                          0x1000
struct eqos_dma_regs {
    uint32_t mode;                                 /* 0x1000 */
    uint32_t sysbus_mode;                          /* 0x1004 */
    uint32_t unused_1008[(0x1100 - 0x1008) / 4];   /* 0x1008 */
    uint32_t ch0_control;                          /* 0x1100 */
    uint32_t ch0_tx_control;                       /* 0x1104 */
    uint32_t ch0_rx_control;                       /* 0x1108 */
    uint32_t unused_110c;                          /* 0x110c */
    uint32_t ch0_txdesc_list_haddress;             /* 0x1110 */
    uint32_t ch0_txdesc_list_address;              /* 0x1114 */
    uint32_t ch0_rxdesc_list_haddress;             /* 0x1118 */
    uint32_t ch0_rxdesc_list_address;              /* 0x111c */
    uint32_t ch0_txdesc_tail_pointer;              /* 0x1120 */
    uint32_t unused_1124;                          /* 0x1124 */
    uint32_t ch0_rxdesc_tail_pointer;              /* 0x1128 */
    uint32_t ch0_txdesc_ring_length;               /* 0x112c */
    uint32_t ch0_rxdesc_ring_length;               /* 0x1130 */
};

#define EQOS_DMA_MODE_SWR                           BIT(0)

#define EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT       16
#define EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_MASK        0xf
#define EQOS_DMA_SYSBUS_MODE_EAME                   BIT(11)
#define EQOS_DMA_SYSBUS_MODE_BLEN16                 BIT(3)
#define EQOS_DMA_SYSBUS_MODE_BLEN8                  BIT(2)
#define EQOS_DMA_SYSBUS_MODE_BLEN4                  BIT(1)

#define EQOS_DMA_CH0_CONTROL_PBLX8                  BIT(16)

#define EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT         16
#define EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK          0x3f
#define EQOS_DMA_CH0_TX_CONTROL_OSP                 BIT(4)
#define EQOS_DMA_CH0_TX_CONTROL_ST                  BIT(0)

#define EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT         16
#define EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK          0x3f
#define EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT          1
#define EQOS_DMA_CH0_RX_CONTROL_RBSZ_MASK           0x3fff
#define EQOS_DMA_CH0_RX_CONTROL_SR                  BIT(0)

#define EQOS_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD BIT(31)

#define EQOS_AUTO_CAL_CONFIG_START                  BIT(31)
#define EQOS_AUTO_CAL_CONFIG_ENABLE                 BIT(29)

#define EQOS_AUTO_CAL_STATUS_ACTIVE                 BIT(31)

/* Descriptors */

#define EQOS_DESCRIPTOR_WORDS   4
#define EQOS_DESCRIPTOR_SIZE    (EQOS_DESCRIPTOR_WORDS * 4)
/* We assume ARCH_DMA_MINALIGN >= 16; 16 is the EQOS HW minimum */
#define EQOS_DESCRIPTOR_ALIGN   ARCH_DMA_MINALIGN
#define EQOS_DESCRIPTORS_TX 4
#define EQOS_DESCRIPTORS_RX 4
#define EQOS_DESCRIPTORS_NUM    (EQOS_DESCRIPTORS_TX + EQOS_DESCRIPTORS_RX)
#define EQOS_DESCRIPTORS_SIZE   ALIGN(EQOS_DESCRIPTORS_NUM * \
                      EQOS_DESCRIPTOR_SIZE, ARCH_DMA_MINALIGN)
#define EQOS_BUFFER_ALIGN   ARCH_DMA_MINALIGN
#define EQOS_MAX_PACKET_SIZE    ALIGN(1568, ARCH_DMA_MINALIGN)
#define EQOS_RX_BUFFER_SIZE (EQOS_DESCRIPTORS_RX * EQOS_MAX_PACKET_SIZE)

/*
 * Warn if the cache-line size is larger than the descriptor size. In such
 * cases the driver will likely fail because the CPU needs to flush the cache
 * when requeuing RX buffers, therefore descriptors written by the hardware
 * may be discarded. Architectures with full IO coherence, such as x86, do not
 * experience this issue, and hence are excluded from this condition.
 *
 * This can be fixed by defining CONFIG_SYS_NONCACHED_MEMORY which will cause
 * the driver to allocate descriptors from a pool of non-cached memory.
 */
#if EQOS_DESCRIPTOR_SIZE < ARCH_DMA_MINALIGN
#if !defined(CONFIG_SYS_NONCACHED_MEMORY) && \
    !CONFIG_IS_ENABLED(SYS_DCACHE_OFF) && !defined(CONFIG_X86) && !defined(CONFIG_RISCV)
#warning Cache line size is larger than descriptor size
#endif
#endif

struct eqos_desc {
    u32 des0;
    u32 des1;
    u32 des2;
    u32 des3;
};

#define EQOS_DESC3_OWN      BIT(31)
#define EQOS_DESC3_FD       BIT(29)
#define EQOS_DESC3_LD       BIT(28)
#define EQOS_DESC3_BUF1V    BIT(24)


#define HSP_BASE_ADDR       ((void *)0x50440000)
#define SYSCRG_CSR_BASE     ((void *)0X51828000)


#define ETH_TX_CLK_SEL          BIT(16)
#define ETH_PHY_INTF_SELI       BIT(0)
#define ETH_CSYSREQ_VAL         BIT(0)
#define HSP_ACLK_CTRL_OFFSET     0x148
#define HSP_ACLK_CLKEN           BIT(31)
#define HSP_ACLK_DIVSOR          (0x2 << 4)

#define HSP_CFG_CTRL_OFFSET      0x14c
#define HSP_CFG_CLKEN            BIT(31)
#define SCU_HSP_PCLK_EN          BIT(30)
#define HSP_CFG_CTRL_REGSET      (HSP_CFG_CLKEN | SCU_HSP_PCLK_EN)

#define HSPDMA_RST_CTRL          0x41c

#define SPEED_10M_CFG            0x80000641
#define SPEED_100M_CFG           0x800000a1
#define SPEED_1000M_CFG          0x80000021

#define UBOOT_USING_ETH0
#ifdef UBOOT_USING_ETH0
#define ETH_PHY_CTRL_OFFSET      0x100
#define ETH_AXI_LP_CTRL_OFFSET   0x108
#define ETH_CTRL_OFFSET          0x158
#define HSP_ETH_ARSTN            BIT(26)

#define ETH0_TXEN_RXDV_DLY       0x118
#define ETH0_RXDATA_DLY          0x11c
#define ETH0_TXDATA_DLY          0x114
#else // UBOOT_USING_ETH0
#define ETH_PHY_CTRL_OFFSET      0x200
#define ETH_AXI_LP_CTRL_OFFSET   0x208
#define ETH_CTRL_OFFSET          0x15c
#define HSP_ETH_ARSTN            BIT(25)

#define ETH1_TXEN_RXDV_DLY       0x218
#define ETH1_RXDATA_DLY          0x21c
#define ETH1_TXDATA_DLY          0x214
#endif

#define GPIO_REG_BASE           ((void *)0x51600000)
#define SYSPORT_OFFSET      ((ulong)0xbf80000000)
#define ESWIN_FPGA
#define DWC_NET_PHYADDR  0x0

struct eqos_config {
    bool reg_access_always_ok;
    int mdio_wait;
    int swr_wait;
    int config_mac;
    int config_mac_mdio;
    phy_interface_t (*interface)(struct udevice *dev);
    struct eqos_ops *ops;
};

struct eqos_ops {
    void (*eqos_inval_desc)(void *desc);
    void (*eqos_flush_desc)(void *desc);
    void (*eqos_inval_buffer)(void *buf, size_t size);
    void (*eqos_flush_buffer)(void *buf, size_t size);
    int (*eqos_probe_resources)(struct udevice *dev);
    int (*eqos_remove_resources)(struct udevice *dev);
    int (*eqos_stop_resets)(struct udevice *dev);
    int (*eqos_start_resets)(struct udevice *dev);
    void (*eqos_stop_clks)(struct udevice *dev);
    int (*eqos_start_clks)(struct udevice *dev);
    int (*eqos_calibrate_pads)(struct udevice *dev);
    int (*eqos_disable_calibration)(struct udevice *dev);
    int (*eqos_set_tx_clk_speed)(struct udevice *dev);
    ulong (*eqos_get_tick_clk_rate)(struct udevice *dev);
};

struct eqos_priv {
    struct udevice *dev;
    const struct eqos_config *config;
    fdt_addr_t regs;
    struct eqos_mac_regs *mac_regs;
    struct eqos_mtl_regs *mtl_regs;
    struct eqos_dma_regs *dma_regs;
    struct reset_ctl reset_ctl;
    struct gpio_desc *phy_reset_gpio;
    struct clk clk_master_bus;
    struct clk clk_rx;
    struct clk clk_ptp_ref;
    struct clk clk_tx;
    struct clk clk_ck;
    struct clk clk_slave_bus;
    struct mii_dev *mii;
    struct phy_device *phy;
    int phyaddr;
    u32 max_speed;
    void *descs;
    struct eqos_desc *tx_descs;
    struct eqos_desc *rx_descs;
    int tx_desc_idx, rx_desc_idx;
    void *tx_dma_buf;
    void *rx_dma_buf;
    void *rx_pkt;
    bool started;
    bool reg_access_ok;
    unsigned int dly_param_1000m[3];
    unsigned int dly_param_100m[3];
    unsigned int dly_param_10m[3];
};


/*
 * TX and RX descriptors are 16 bytes. This causes problems with the cache
 * maintenance on CPUs where the cache-line size exceeds the size of these
 * descriptors. What will happen is that when the driver receives a packet
 * it will be immediately requeued for the hardware to reuse. The CPU will
 * therefore need to flush the cache-line containing the descriptor, which
 * will cause all other descriptors in the same cache-line to be flushed
 * along with it. If one of those descriptors had been written to by the
 * device those changes (and the associated packet) will be lost.
 *
 * To work around this, we make use of non-cached memory if available. If
 * descriptors are mapped uncached there's no need to manually flush them
 * or invalidate them.
 *
 * Note that this only applies to descriptors. The packet data buffers do
 * not have the same constraints since they are 1536 bytes large, so they
 * are unlikely to share cache-lines.
 */

static void *eqos_alloc_descs(unsigned int num)
{
#ifdef CONFIG_SYS_NONCACHED_MEMORY
    return (void *)noncached_alloc(EQOS_DESCRIPTORS_SIZE,
                      EQOS_DESCRIPTOR_ALIGN);
#else
    return memalign(EQOS_DESCRIPTOR_ALIGN, EQOS_DESCRIPTORS_SIZE);
#endif
}

static void eqos_free_descs(void *descs)
{
#ifdef CONFIG_SYS_NONCACHED_MEMORY
    /* FIXME: noncached_alloc() has no opposite */
#else
    free(descs);
#endif
}

static void eqos_inval_desc_generic(void *desc)
{
#ifndef CONFIG_SYS_NONCACHED_MEMORY
    unsigned long start = rounddown((unsigned long)desc, ARCH_DMA_MINALIGN);
    unsigned long end = roundup((unsigned long)desc + EQOS_DESCRIPTOR_SIZE,
                    ARCH_DMA_MINALIGN);

    invalidate_dcache_range(start, end);
#endif
}

static void eqos_flush_desc_generic(void *desc)
{
#ifndef CONFIG_SYS_NONCACHED_MEMORY
    unsigned long start = rounddown((unsigned long)desc, ARCH_DMA_MINALIGN);
    unsigned long end = roundup((unsigned long)desc + EQOS_DESCRIPTOR_SIZE,
                    ARCH_DMA_MINALIGN);

    flush_dcache_range(start, end);
#endif
}

static void eqos_inval_buffer_generic(void *buf, size_t size)
{
#ifndef CONFIG_SYS_NONCACHED_MEMORY
    unsigned long start = rounddown((unsigned long)buf, ARCH_DMA_MINALIGN);
    unsigned long end = roundup((unsigned long)buf + size,
                    ARCH_DMA_MINALIGN);

    invalidate_dcache_range(start, end);
#endif
}

static void eqos_flush_buffer_generic(void *buf, size_t size)
{
    unsigned long start = rounddown((unsigned long)buf, ARCH_DMA_MINALIGN);
    unsigned long end = roundup((unsigned long)buf + size,
                    ARCH_DMA_MINALIGN);

    flush_dcache_range(start, end);
}

static int eqos_mdio_wait_idle(struct eqos_priv *eqos)
{
    return wait_for_bit_le32(&eqos->mac_regs->mdio_address,
                 EQOS_MAC_MDIO_ADDRESS_GB, false,
                 1000000, true);
}

static int eqos_mdio_read(struct mii_dev *bus, int mdio_addr, int mdio_devad,
              int mdio_reg)
{
    struct eqos_priv *eqos = bus->priv;
    u32 val;
    int ret;

    debug("%s(dev=%p, addr=%x, reg=%d):\n", __func__, eqos->dev, mdio_addr,
          mdio_reg);

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        pr_err("MDIO not idle at entry");
        return ret;
    }

    val = readl(&eqos->mac_regs->mdio_address);
    val &= EQOS_MAC_MDIO_ADDRESS_SKAP |
        EQOS_MAC_MDIO_ADDRESS_C45E;
    val |= (mdio_addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT) |
        (mdio_reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT) |
        (eqos->config->config_mac_mdio <<
         EQOS_MAC_MDIO_ADDRESS_CR_SHIFT) |
        (EQOS_MAC_MDIO_ADDRESS_GOC_READ <<
         EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT) |
        EQOS_MAC_MDIO_ADDRESS_GB;
    writel(val, &eqos->mac_regs->mdio_address);

    udelay(eqos->config->mdio_wait);

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        pr_err("MDIO read didn't complete");
        return ret;
    }

    val = readl(&eqos->mac_regs->mdio_data);
    val &= EQOS_MAC_MDIO_DATA_GD_MASK;

    debug("%s: val=%x\n", __func__, val);

    return val;
}

static int eqos_mdio_write(struct mii_dev *bus, int mdio_addr, int mdio_devad,
               int mdio_reg, u16 mdio_val)
{
    struct eqos_priv *eqos = bus->priv;
    u32 val;
    int ret;

    debug("%s(dev=%p, addr=%x, reg=%d, val=%x):\n", __func__, eqos->dev,
          mdio_addr, mdio_reg, mdio_val);

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        pr_err("MDIO not idle at entry");
        return ret;
    }

    writel(mdio_val, &eqos->mac_regs->mdio_data);

    val = readl(&eqos->mac_regs->mdio_address);
    val &= EQOS_MAC_MDIO_ADDRESS_SKAP |
        EQOS_MAC_MDIO_ADDRESS_C45E;
    val |= (mdio_addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT) |
        (mdio_reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT) |
        (eqos->config->config_mac_mdio <<
         EQOS_MAC_MDIO_ADDRESS_CR_SHIFT) |
        (EQOS_MAC_MDIO_ADDRESS_GOC_WRITE <<
         EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT) |
        EQOS_MAC_MDIO_ADDRESS_GB;
    writel(val, &eqos->mac_regs->mdio_address);

    udelay(eqos->config->mdio_wait);

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        pr_err("MDIO read didn't complete");
        return ret;
    }

    return 0;
}

static int eqos_start_clks_eswin(struct udevice *dev)
{
#ifdef ESWIN_ETH_CLK
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret;

    debug("%s(dev=%p):\n", __func__, dev);

    ret = clk_enable(&eqos->clk_master_bus);
    if (ret < 0) {
        pr_err("clk_enable(clk_master_bus) failed: %d", ret);
        goto err;
    }

    ret = clk_enable(&eqos->clk_rx);
    if (ret < 0) {
        pr_err("clk_enable(clk_rx) failed: %d", ret);
        goto err_disable_clk_master_bus;
    }

    ret = clk_enable(&eqos->clk_tx);
    if (ret < 0) {
        pr_err("clk_enable(clk_tx) failed: %d", ret);
        goto err_disable_clk_rx;
    }

    if (clk_valid(&eqos->clk_ck)) {
        ret = clk_enable(&eqos->clk_ck);
        if (ret < 0) {
            pr_err("clk_enable(clk_ck) failed: %d", ret);
            goto err_disable_clk_tx;
        }
    }
#endif

    debug("%s: OK\n", __func__);
    return 0;

#ifdef ESWIN_ETH_CLK
err_disable_clk_tx:
    clk_disable(&eqos->clk_tx);
err_disable_clk_rx:
    clk_disable(&eqos->clk_rx);
err_disable_clk_master_bus:
    clk_disable(&eqos->clk_master_bus);
err:
    debug("%s: FAILED: %d\n", __func__, ret);
    return ret;
#endif
}


static void eqos_stop_clks_eswin(struct udevice *dev)
{
#ifdef ESWIN_ETH_CLK
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    clk_disable(&eqos->clk_tx);
    clk_disable(&eqos->clk_rx);
    clk_disable(&eqos->clk_master_bus);
    if (clk_valid(&eqos->clk_ck))
        clk_disable(&eqos->clk_ck);
#endif

    debug("%s: OK\n", __func__);
}

static int eqos_null_ops(struct udevice *dev)
{
    return 0;
}

static ulong eqos_get_tick_clk_rate_eswin(struct udevice *dev)
{
#ifdef ESWIN_ETH_CLK
    struct eqos_priv *eqos = dev_get_priv(dev);

    return clk_get_rate(&eqos->clk_master_bus);
#else
    return 0;
#endif
}
__weak u32 imx_get_eqos_csr_clk(void)
{
    return 100 * 1000000;
}
__weak int imx_eqos_txclk_set_rate(unsigned long rate)
{
    return 0;
}

static int eqos_calibrate_pads_eswin(struct udevice *dev)
{
    return 0;
}

static int eqos_disable_calibration_eswin(struct udevice *dev)
{
    return 0;
}

static int eqos_set_full_duplex(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    setbits_le32(&eqos->mac_regs->configuration, EQOS_MAC_CONFIGURATION_DM);

    return 0;
}

static int eqos_set_half_duplex(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    clrbits_le32(&eqos->mac_regs->configuration, EQOS_MAC_CONFIGURATION_DM);

    /* WAR: Flush TX queue when switching to half-duplex */
    setbits_le32(&eqos->mtl_regs->txq0_operation_mode,
             EQOS_MTL_TXQ0_OPERATION_MODE_FTQ);

    return 0;
}

static int eqos_set_gmii_speed(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    clrbits_le32(&eqos->mac_regs->configuration,
             EQOS_MAC_CONFIGURATION_PS | EQOS_MAC_CONFIGURATION_FES);

    return 0;
}

static int eqos_set_mii_speed_100(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    setbits_le32(&eqos->mac_regs->configuration,
             EQOS_MAC_CONFIGURATION_PS | EQOS_MAC_CONFIGURATION_FES);

    return 0;
}

static int eqos_set_mii_speed_10(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    clrsetbits_le32(&eqos->mac_regs->configuration,
            EQOS_MAC_CONFIGURATION_FES, EQOS_MAC_CONFIGURATION_PS);

    return 0;
}

static int eqos_set_tx_clk_speed_eswin(struct udevice *dev)
{
    return 0;
}

static int eqos_adjust_link(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret;
    int speed_cfg;
    bool en_calibration;

    debug("%s(dev=%p):\n", __func__, dev);

    if (eqos->phy->duplex)
        ret = eqos_set_full_duplex(dev);
    else
        ret = eqos_set_half_duplex(dev);
    if (ret < 0) {
        pr_err("eqos_set_*_duplex() failed: %d", ret);
        return ret;
    }

    switch (eqos->phy->speed) {
    case SPEED_1000:
        en_calibration = true;
        speed_cfg = SPEED_1000M_CFG;
        ret = eqos_set_gmii_speed(dev);
#ifdef UBOOT_USING_ETH0
        writel(eqos->dly_param_1000m[0], (HSP_BASE_ADDR + ETH0_TXDATA_DLY));
        writel(eqos->dly_param_1000m[1], (HSP_BASE_ADDR + ETH0_TXEN_RXDV_DLY));
        writel(eqos->dly_param_1000m[2], (HSP_BASE_ADDR + ETH0_RXDATA_DLY));
#else
        writel(eqos->dly_param_1000m[0], (HSP_BASE_ADDR + ETH1_TXDATA_DLY));
        writel(eqos->dly_param_1000m[1], (HSP_BASE_ADDR + ETH1_TXEN_RXDV_DLY));
        writel(eqos->dly_param_1000m[2], (HSP_BASE_ADDR + ETH1_RXDATA_DLY));
#endif
        break;
    case SPEED_100:
        en_calibration = true;
        speed_cfg = SPEED_100M_CFG;
        ret = eqos_set_mii_speed_100(dev);
#ifdef UBOOT_USING_ETH0
        writel(eqos->dly_param_100m[0], (HSP_BASE_ADDR + ETH0_TXDATA_DLY));
        writel(eqos->dly_param_100m[1], (HSP_BASE_ADDR + ETH0_TXEN_RXDV_DLY));
        writel(eqos->dly_param_100m[2], (HSP_BASE_ADDR + ETH0_RXDATA_DLY));
#else
        writel(eqos->dly_param_100m[0], (HSP_BASE_ADDR + ETH1_TXDATA_DLY));
        writel(eqos->dly_param_100m[1], (HSP_BASE_ADDR + ETH1_TXEN_RXDV_DLY));
        writel(eqos->dly_param_100m[2], (HSP_BASE_ADDR + ETH1_RXDATA_DLY));
#endif
        break;
    case SPEED_10:
        en_calibration = false;
        speed_cfg = SPEED_10M_CFG;
#ifdef UBOOT_USING_ETH0
        writel(eqos->dly_param_10m[0], (HSP_BASE_ADDR + ETH0_TXDATA_DLY));
        writel(eqos->dly_param_10m[1], (HSP_BASE_ADDR + ETH0_TXEN_RXDV_DLY));
        writel(eqos->dly_param_10m[2], (HSP_BASE_ADDR + ETH0_RXDATA_DLY));
#else
        writel(eqos->dly_param_10m[0], (HSP_BASE_ADDR + ETH1_TXDATA_DLY));
        writel(eqos->dly_param_10m[1], (HSP_BASE_ADDR + ETH1_TXEN_RXDV_DLY));
        writel(eqos->dly_param_10m[2], (HSP_BASE_ADDR + ETH1_RXDATA_DLY));
#endif
        ret = eqos_set_mii_speed_10(dev);
        break;
    default:
        pr_err("invalid speed %d", eqos->phy->speed);
        return -EINVAL;
    }
    if (ret < 0) {
        pr_err("eqos_set_*mii_speed*() failed: %d", ret);
        return ret;
    }

#ifdef ESWIN_FPGA
    writel(speed_cfg, SYSCRG_CSR_BASE + ETH_CTRL_OFFSET);
#endif

    if (en_calibration) {
        ret = eqos->config->ops->eqos_calibrate_pads(dev);
        if (ret < 0) {
            pr_err("eqos_calibrate_pads() failed: %d",
                   ret);
            return ret;
        }
    } else {
        ret = eqos->config->ops->eqos_disable_calibration(dev);
        if (ret < 0) {
            pr_err("eqos_disable_calibration() failed: %d",
                   ret);
            return ret;
        }
    }
    ret = eqos->config->ops->eqos_set_tx_clk_speed(dev);
    if (ret < 0) {
        pr_err("eqos_set_tx_clk_speed() failed: %d", ret);
        return ret;
    }

    return 0;
}

static int eqos_write_hwaddr(struct udevice *dev)
{
    struct eth_pdata *plat = dev_get_plat(dev);
    struct eqos_priv *eqos = dev_get_priv(dev);
    uint32_t val;

    /*
     * This function may be called before start() or after stop(). At that
     * time, on at least some configurations of the EQoS HW, all clocks to
     * the EQoS HW block will be stopped, and a reset signal applied. If
     * any register access is attempted in this state, bus timeouts or CPU
     * hangs may occur. This check prevents that.
     *
     * A simple solution to this problem would be to not implement
     * write_hwaddr(), since start() always writes the MAC address into HW
     * anyway. However, it is desirable to implement write_hwaddr() to
     * support the case of SW that runs subsequent to U-Boot which expects
     * the MAC address to already be programmed into the EQoS registers,
     * which must happen irrespective of whether the U-Boot user (or
     * scripts) actually made use of the EQoS device, and hence
     * irrespective of whether start() was ever called.
     *
     * Note that this requirement by subsequent SW is not valid for
     * Tegra186, and is likely not valid for any non-PCI instantiation of
     * the EQoS HW block. This function is implemented solely as
     * future-proofing with the expectation the driver will eventually be
     * ported to some system where the expectation above is true.
     */
    if (!eqos->config->reg_access_always_ok && !eqos->reg_access_ok)
        return 0;

    /* Update the MAC address */
    val = (plat->enetaddr[5] << 8) |
        (plat->enetaddr[4]);
    writel(val, &eqos->mac_regs->address0_high);
    val = (plat->enetaddr[3] << 24) |
        (plat->enetaddr[2] << 16) |
        (plat->enetaddr[1] << 8) |
        (plat->enetaddr[0]);
    writel(val, &eqos->mac_regs->address0_low);

    return 0;
}

static int eqos_read_rom_hwaddr(struct udevice *dev)
{
    struct eth_pdata *pdata = dev_get_plat(dev);

    return !is_valid_ethaddr(pdata->enetaddr);
}

static int eqos_start(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret, i;
    ulong rate;
    u32 val, tx_fifo_sz, rx_fifo_sz, tqs, rqs, pbl;
    ulong last_rx_desc;

    debug("%s(dev=%p):\n", __func__, dev);

    eqos->tx_desc_idx = 0;
    eqos->rx_desc_idx = 0;

    ret = eqos->config->ops->eqos_start_clks(dev);
    if (ret < 0) {
        pr_err("eqos_start_clks() failed: %d", ret);
        goto err;
    }

    ret = eqos->config->ops->eqos_start_resets(dev);
    if (ret < 0) {
        pr_err("eqos_start_resets() failed: %d", ret);
        goto err_stop_clks;
    }

    udelay(10);

    eqos->reg_access_ok = true;

    ret = wait_for_bit_le32(&eqos->dma_regs->mode,
                EQOS_DMA_MODE_SWR, false,
                eqos->config->swr_wait, false);
    if (ret) {
        pr_err("EQOS_DMA_MODE_SWR stuck");
        goto err_stop_resets;
    }

    ret = eqos->config->ops->eqos_calibrate_pads(dev);
    if (ret < 0) {
        pr_err("eqos_calibrate_pads() failed: %d", ret);
        goto err_stop_resets;
    }
    rate = eqos->config->ops->eqos_get_tick_clk_rate(dev);

    val = (rate / 1000000) - 1;
    writel(val, &eqos->mac_regs->us_tic_counter);

    /*
     * if PHY was already connected and configured,
     * don't need to reconnect/reconfigure again
     */
    if (!eqos->phy) {
        int addr = -1;
        char *s = NULL;
        unsigned long eth_speed = 0;

#ifdef DWC_NET_PHYADDR
        addr = DWC_NET_PHYADDR;
#endif
#ifdef CONFIG_DM_ETH_PHY
        addr = eth_phy_get_addr(dev);
#endif
        eqos->phy = phy_connect(eqos->mii, addr, dev,
                    eqos->config->interface(dev));
        if (!eqos->phy) {
            pr_err("phy_connect() failed");
            goto err_stop_resets;
        }

        if (eqos->max_speed) {
            ret = phy_set_supported(eqos->phy, eqos->max_speed);
            if (ret) {
                pr_err("phy_set_supported() failed: %d", ret);
                goto err_shutdown_phy;
            }
        }

        s = env_get("eth_speed");
        if (s)
            eth_speed = simple_strtoul(s, NULL, 10);
        if (eth_speed == 10) {
            eqos->phy->autoneg = AUTONEG_DISABLE;
            eqos->phy->speed = SPEED_10;
        } else if (eth_speed == 100) {
            eqos->phy->autoneg = AUTONEG_DISABLE;
            eqos->phy->speed = SPEED_100;
        } else if (eth_speed == 1000) {
            eqos->phy->autoneg = AUTONEG_DISABLE;
            eqos->phy->speed = SPEED_1000;
        }

        ret = phy_config(eqos->phy);
        if (ret < 0) {
            pr_err("phy_config() failed: %d", ret);
            goto err_shutdown_phy;
        }
    }

    ret = phy_startup(eqos->phy);
    if (ret < 0) {
        pr_err("phy_startup() failed: %d", ret);
        goto err_shutdown_phy;
    }

    if (!eqos->phy->link && eqos->phy->autoneg) {
        pr_err("No link");
        goto err_shutdown_phy;
    }

    ret = eqos_adjust_link(dev);
    if (ret < 0) {
        pr_err("eqos_adjust_link() failed: %d", ret);
        goto err_shutdown_phy;
    }

    /* Configure MTL */
    writel(0x60, &eqos->mtl_regs->txq0_quantum_weight - 0x100);

    /* Enable Store and Forward mode for TX */
    /* Program Tx operating mode */
    setbits_le32(&eqos->mtl_regs->txq0_operation_mode,
             EQOS_MTL_TXQ0_OPERATION_MODE_TSF |
             (EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_ENABLED <<
              EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT));

    /* Transmit Queue weight */
    writel(0x10, &eqos->mtl_regs->txq0_quantum_weight);

    /* Enable Store and Forward mode for RX, since no jumbo frame */
    setbits_le32(&eqos->mtl_regs->rxq0_operation_mode,
             EQOS_MTL_RXQ0_OPERATION_MODE_RSF |
             EQOS_MTL_RXQ0_OPERATION_MODE_FEP |
             EQOS_MTL_RXQ0_OPERATION_MODE_FUP);

    /* Transmit/Receive queue fifo size; use all RAM for 1 queue */
    val = readl(&eqos->mac_regs->hw_feature1);
    tx_fifo_sz = (val >> EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT) &
        EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK;
    rx_fifo_sz = (val >> EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT) &
        EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK;

    /*
     * r/tx_fifo_sz is encoded as log2(n / 128). Undo that by shifting.
     * r/tqs is encoded as (n / 256) - 1.
     */
    tqs = (128 << tx_fifo_sz) / 256 - 1;
    rqs = (128 << rx_fifo_sz) / 256 - 1;

    clrsetbits_le32(&eqos->mtl_regs->txq0_operation_mode,
            EQOS_MTL_TXQ0_OPERATION_MODE_TQS_MASK <<
            EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT,
            tqs << EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
    clrsetbits_le32(&eqos->mtl_regs->rxq0_operation_mode,
            EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK <<
            EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT,
            rqs << EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT);

    /* Flow control used only if each channel gets 4KB or more FIFO */
    if (rqs >= ((4096 / 256) - 1)) {
        u32 rfd, rfa;

        setbits_le32(&eqos->mtl_regs->rxq0_operation_mode,
                 EQOS_MTL_RXQ0_OPERATION_MODE_EHFC);

        /*
         * Set Threshold for Activating Flow Contol space for min 2
         * frames ie, (1500 * 1) = 1500 bytes.
         *
         * Set Threshold for Deactivating Flow Contol for space of
         * min 1 frame (frame size 1500bytes) in receive fifo
         */
        if (rqs == ((4096 / 256) - 1)) {
            /*
             * This violates the above formula because of FIFO size
             * limit therefore overflow may occur inspite of this.
             */
            rfd = 0x3;  /* Full-3K */
            rfa = 0x1;  /* Full-1.5K */
        } else if (rqs == ((8192 / 256) - 1)) {
            rfd = 0x6;  /* Full-4K */
            rfa = 0xa;  /* Full-6K */
        } else if (rqs == ((16384 / 256) - 1)) {
            rfd = 0x6;  /* Full-4K */
            rfa = 0x12; /* Full-10K */
        } else {
            rfd = 0x6;  /* Full-4K */
            rfa = 0x1E; /* Full-16K */
        }

        clrsetbits_le32(&eqos->mtl_regs->rxq0_operation_mode,
                (EQOS_MTL_RXQ0_OPERATION_MODE_RFD_MASK <<
                 EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
                (EQOS_MTL_RXQ0_OPERATION_MODE_RFA_MASK <<
                 EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT),
                (rfd <<
                 EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
                (rfa <<
                 EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT));
    }

    /* Configure MAC */

    clrsetbits_le32(&eqos->mac_regs->rxq_ctrl0,
            EQOS_MAC_RXQ_CTRL0_RXQ0EN_MASK <<
            EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT,
            eqos->config->config_mac <<
            EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT);

    /* Multicast and Broadcast Queue Enable */
    setbits_le32(&eqos->mac_regs->unused_0a4,
             0x00100000);
    /* enable promise mode */
    setbits_le32(&eqos->mac_regs->unused_004[1],
             0x1);

    /* Set TX flow control parameters */
    /* Set Pause Time */
    setbits_le32(&eqos->mac_regs->q0_tx_flow_ctrl,
             0xffff << EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT);
    /* Assign priority for TX flow control */
    clrbits_le32(&eqos->mac_regs->txq_prty_map0,
             EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_MASK <<
             EQOS_MAC_TXQ_PRTY_MAP0_PSTQ0_SHIFT);
    /* Assign priority for RX flow control */
    clrbits_le32(&eqos->mac_regs->rxq_ctrl2,
             EQOS_MAC_RXQ_CTRL2_PSRQ0_MASK <<
             EQOS_MAC_RXQ_CTRL2_PSRQ0_SHIFT);
    /* Enable flow control */
    setbits_le32(&eqos->mac_regs->q0_tx_flow_ctrl,
             EQOS_MAC_Q0_TX_FLOW_CTRL_TFE);
    setbits_le32(&eqos->mac_regs->rx_flow_ctrl,
             EQOS_MAC_RX_FLOW_CTRL_RFE);

    clrsetbits_le32(&eqos->mac_regs->configuration,
            EQOS_MAC_CONFIGURATION_GPSLCE |
            EQOS_MAC_CONFIGURATION_WD |
            EQOS_MAC_CONFIGURATION_JD |
            EQOS_MAC_CONFIGURATION_JE,
            EQOS_MAC_CONFIGURATION_CST |
            EQOS_MAC_CONFIGURATION_ACS);

    eqos_write_hwaddr(dev);

    /* Configure DMA */

    /* Enable OSP mode */
    setbits_le32(&eqos->dma_regs->ch0_tx_control,
             EQOS_DMA_CH0_TX_CONTROL_OSP);

    /* RX buffer size. Must be a multiple of bus width */
    clrsetbits_le32(&eqos->dma_regs->ch0_rx_control,
            EQOS_DMA_CH0_RX_CONTROL_RBSZ_MASK <<
            EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT,
            EQOS_MAX_PACKET_SIZE <<
            EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT);

    setbits_le32(&eqos->dma_regs->ch0_control,
             EQOS_DMA_CH0_CONTROL_PBLX8);

    /*
     * Burst length must be < 1/2 FIFO size.
     * FIFO size in tqs is encoded as (n / 256) - 1.
     * Each burst is n * 8 (PBLX8) * 16 (AXI width) == 128 bytes.
     * Half of n * 256 is n * 128, so pbl == tqs, modulo the -1.
     */
    pbl = tqs + 1;
    if (pbl > 32)
        pbl = 32;
    clrsetbits_le32(&eqos->dma_regs->ch0_tx_control,
            EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK <<
            EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT,
            pbl << EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT);

    clrsetbits_le32(&eqos->dma_regs->ch0_rx_control,
            EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK <<
            EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT,
            8 << EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT);

    /* DMA performance configuration */
    val = (2 << EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT) |
        EQOS_DMA_SYSBUS_MODE_EAME | EQOS_DMA_SYSBUS_MODE_BLEN16 |
        EQOS_DMA_SYSBUS_MODE_BLEN8 | EQOS_DMA_SYSBUS_MODE_BLEN4;
    writel(val, &eqos->dma_regs->sysbus_mode);

    /* Set up descriptors */

    memset(eqos->descs, 0, EQOS_DESCRIPTORS_SIZE);
    for (i = 0; i < EQOS_DESCRIPTORS_RX; i++) {
        struct eqos_desc *rx_desc = &(eqos->rx_descs[i]);
        rx_desc->des0 = (u32)(ulong)(eqos->rx_dma_buf +
                         (i * EQOS_MAX_PACKET_SIZE));
        rx_desc->des3 = EQOS_DESC3_OWN | EQOS_DESC3_BUF1V;
        mb();
        eqos->config->ops->eqos_flush_desc(rx_desc);
        eqos->config->ops->eqos_inval_buffer(eqos->rx_dma_buf +
                        (i * EQOS_MAX_PACKET_SIZE),
                        EQOS_MAX_PACKET_SIZE);
    }

    writel(0, &eqos->dma_regs->ch0_txdesc_list_haddress);
    writel((ulong)eqos->tx_descs, &eqos->dma_regs->ch0_txdesc_list_address);
    writel(EQOS_DESCRIPTORS_TX - 1,
           &eqos->dma_regs->ch0_txdesc_ring_length);

    writel(0, &eqos->dma_regs->ch0_rxdesc_list_haddress);
    writel((ulong)eqos->rx_descs, &eqos->dma_regs->ch0_rxdesc_list_address);
    writel(EQOS_DESCRIPTORS_RX - 1,
           &eqos->dma_regs->ch0_rxdesc_ring_length);

    /* Enable everything */
    setbits_le32(&eqos->mac_regs->configuration,
             EQOS_MAC_CONFIGURATION_TE | EQOS_MAC_CONFIGURATION_RE);
    setbits_le32(&eqos->dma_regs->ch0_tx_control,
             EQOS_DMA_CH0_TX_CONTROL_ST);
    setbits_le32(&eqos->dma_regs->ch0_rx_control,
             EQOS_DMA_CH0_RX_CONTROL_SR);

    /* TX tail pointer not written until we need to TX a packet */
    /*
     * Point RX tail pointer at last descriptor. Ideally, we'd point at the
     * first descriptor, implying all descriptors were available. However,
     * that's not distinguishable from none of the descriptors being
     * available.
     */
    last_rx_desc = (ulong)&(eqos->rx_descs[(EQOS_DESCRIPTORS_RX - 1)]);
    writel(last_rx_desc, &eqos->dma_regs->ch0_rxdesc_tail_pointer);

    eqos->started = true;

    debug("%s: OK\n", __func__);

    return 0;

err_shutdown_phy:
    phy_shutdown(eqos->phy);
err_stop_resets:
    eqos->config->ops->eqos_stop_resets(dev);
err_stop_clks:
    eqos->config->ops->eqos_stop_clks(dev);
err:
    pr_err("FAILED: %d", ret);
    return ret;
}

static void eqos_stop(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int i;

    debug("%s(dev=%p):\n", __func__, dev);

    if (!eqos->started)
        return;
    eqos->started = false;
    eqos->reg_access_ok = false;

    /* Disable TX DMA */
    clrbits_le32(&eqos->dma_regs->ch0_tx_control,
             EQOS_DMA_CH0_TX_CONTROL_ST);

    /* Wait for TX all packets to drain out of MTL */
    for (i = 0; i < 1000000; i++) {
        u32 val = readl(&eqos->mtl_regs->txq0_debug);
        u32 trcsts = (val >> EQOS_MTL_TXQ0_DEBUG_TRCSTS_SHIFT) &
            EQOS_MTL_TXQ0_DEBUG_TRCSTS_MASK;
        u32 txqsts = val & EQOS_MTL_TXQ0_DEBUG_TXQSTS;
        if ((trcsts != 1) && (!txqsts))
            break;
    }

    /* Turn off MAC TX and RX */
    clrbits_le32(&eqos->mac_regs->configuration,
             EQOS_MAC_CONFIGURATION_TE | EQOS_MAC_CONFIGURATION_RE);

    /* Wait for all RX packets to drain out of MTL */
    for (i = 0; i < 1000000; i++) {
        u32 val = readl(&eqos->mtl_regs->rxq0_debug);
        u32 prxq = (val >> EQOS_MTL_RXQ0_DEBUG_PRXQ_SHIFT) &
            EQOS_MTL_RXQ0_DEBUG_PRXQ_MASK;
        u32 rxqsts = (val >> EQOS_MTL_RXQ0_DEBUG_RXQSTS_SHIFT) &
            EQOS_MTL_RXQ0_DEBUG_RXQSTS_MASK;
        if ((!prxq) && (!rxqsts))
            break;
    }

    /* Turn off RX DMA */
    clrbits_le32(&eqos->dma_regs->ch0_rx_control,
             EQOS_DMA_CH0_RX_CONTROL_SR);

    if (eqos->phy) {
        phy_shutdown(eqos->phy);
    }
    eqos->config->ops->eqos_stop_resets(dev);
    eqos->config->ops->eqos_stop_clks(dev);

    debug("%s: OK\n", __func__);
}

static int eqos_send(struct udevice *dev, void *packet, int length)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    struct eqos_desc *tx_desc;
    int i;
    debug("%s(dev=%p, packet=%p, length=%d):\n", __func__, dev, packet,length);

#ifdef SYSPORT_OFFSET
    memcpy((void *)(((ulong)eqos->tx_dma_buf) + SYSPORT_OFFSET), packet, length);
#else
    memcpy(eqos->tx_dma_buf, packet, length);
//    eqos->config->ops->eqos_flush_buffer(eqos->tx_dma_buf, length);
#endif

    tx_desc = &(eqos->tx_descs[eqos->tx_desc_idx]);
#ifdef SYSPORT_OFFSET
    tx_desc =(struct eqos_desc *)((ulong)(tx_desc) + SYSPORT_OFFSET);
#else
    tx_desc =(struct eqos_desc *)((ulong)(tx_desc));
#endif
    eqos->tx_desc_idx++;
    eqos->tx_desc_idx %= EQOS_DESCRIPTORS_TX;

    tx_desc->des0 =(ulong)(ulong)eqos->tx_dma_buf;
    tx_desc->des1 = 0;
    tx_desc->des2 = length;

#ifndef SYSPORT_OFFSET
//    eqos->config->ops->eqos_inval_desc(tx_desc);
#endif
    /*
     * Make sure that if HW sees the _OWN write below, it will see all the
     * writes to the rest of the descriptor too.
     */
    mb(); //DMA拥有描述符后开始搬运数据
    tx_desc->des3 = EQOS_DESC3_OWN | EQOS_DESC3_FD | EQOS_DESC3_LD | length;
#ifndef SYSPORT_OFFSET
 //   eqos->config->ops->eqos_flush_desc(tx_desc);
#endif
    writel((ulong)(&(eqos->tx_descs[eqos->tx_desc_idx])),
        &eqos->dma_regs->ch0_txdesc_tail_pointer);

    for (i = 0; i < 100000; i++) {
        if (!((readl(&tx_desc->des3)) & EQOS_DESC3_OWN))
            return 0;
        udelay(1);
    }

    debug("%s: TX timeout\n", __func__);

    return -ETIMEDOUT;
}

static int eqos_recv(struct udevice *dev, int flags, uchar **packetp)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    struct eqos_desc *rx_desc;
    int length;

    debug("%s(dev=%p, flags=%x):\n", __func__, dev, flags);

    rx_desc = &(eqos->rx_descs[eqos->rx_desc_idx]);
#ifdef SYSPORT_OFFSET
    rx_desc = (struct eqos_desc *)((ulong)(rx_desc) + SYSPORT_OFFSET);
#else
    rx_desc = (struct eqos_desc *)((ulong)(rx_desc));
//    eqos->config->ops->eqos_inval_desc(rx_desc);
#endif
    if (rx_desc->des3 & EQOS_DESC3_OWN) {
        debug("%s: RX packet not available\n", __func__);
        return -EAGAIN;
    }

    *packetp = eqos->rx_dma_buf +
        (eqos->rx_desc_idx * EQOS_MAX_PACKET_SIZE);
    length = rx_desc->des3 & 0x7fff;
    debug("%s: *packetp=%p, length=%d\n", __func__, *packetp, length);
#ifdef SYSPORT_OFFSET
    *packetp =(uchar *)((ulong)(*packetp) + SYSPORT_OFFSET);
#else
    *packetp =(uchar *)((ulong)(*packetp));
//    eqos->config->ops->eqos_inval_buffer(*packetp, length);
#endif
    return length;
}

static int eqos_free_pkt(struct udevice *dev, uchar *packet, int length)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    uchar *packet_expected;
    struct eqos_desc *rx_desc;

    debug("%s(packet=%p, length=%d)\n", __func__, packet, length);

    packet_expected = eqos->rx_dma_buf +
        (eqos->rx_desc_idx * EQOS_MAX_PACKET_SIZE);
#ifdef SYSPORT_OFFSET
    packet =(uchar *)((ulong)packet - SYSPORT_OFFSET);
#else
    packet =(uchar *)((ulong)packet);
#endif
    if (packet != packet_expected) {
        debug("%s: Unexpected packet (expected %p)\n", __func__,
              packet_expected);
        return -EINVAL;
    }

    eqos->config->ops->eqos_inval_buffer(packet, length);

    rx_desc = &(eqos->rx_descs[eqos->rx_desc_idx]);
#ifdef SYSPORT_OFFSET
    rx_desc = (struct eqos_desc *)((u64)(rx_desc) + SYSPORT_OFFSET);
#else
    rx_desc = (struct eqos_desc *)((u64)(rx_desc));
#endif
    rx_desc->des0 = 0;
    mb();
#ifndef SYSPORT_OFFSET
//    eqos->config->ops->eqos_flush_desc(rx_desc);
//    eqos->config->ops->eqos_inval_buffer(packet, length);
#endif
    rx_desc->des0 = (ulong)packet;
    rx_desc->des1 = 0;
    rx_desc->des2 = 0;
    /*
     * Make sure that if HW sees the _OWN write below, it will see all the
     * writes to the rest of the descriptor too.
     */
    mb();
    rx_desc->des3 = EQOS_DESC3_OWN | EQOS_DESC3_BUF1V;
#ifndef SYSPORT_OFFSET
//    eqos->config->ops->eqos_flush_desc(rx_desc);
#endif
    writel((ulong)(&(eqos->rx_descs[eqos->rx_desc_idx])), &eqos->dma_regs->ch0_rxdesc_tail_pointer);

    eqos->rx_desc_idx++;
    eqos->rx_desc_idx %= EQOS_DESCRIPTORS_RX;

    return 0;
}

static int eqos_probe_resources_core(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret;

    debug("%s(dev=%p):\n", __func__, dev);

    eqos->descs = eqos_alloc_descs(EQOS_DESCRIPTORS_TX +
                       EQOS_DESCRIPTORS_RX);
    if (!eqos->descs) {
        debug("%s: eqos_alloc_descs() failed\n", __func__);
        ret = -ENOMEM;
        goto err;
    }

    eqos->tx_descs = (struct eqos_desc *)eqos->descs;
    eqos->rx_descs = (eqos->tx_descs + EQOS_DESCRIPTORS_TX);
    debug("%s: tx_descs=%p, rx_descs=%p\n", __func__, eqos->tx_descs,
          eqos->rx_descs);

    eqos->tx_dma_buf = memalign(EQOS_BUFFER_ALIGN, EQOS_MAX_PACKET_SIZE);
    if (!eqos->tx_dma_buf) {
        debug("%s: memalign(tx_dma_buf) failed\n", __func__);
        ret = -ENOMEM;
        goto err_free_descs;
    }
    debug("%s: tx_dma_buf=%p\n", __func__, eqos->tx_dma_buf);

    eqos->rx_dma_buf = memalign(EQOS_BUFFER_ALIGN, EQOS_RX_BUFFER_SIZE);
    if (!eqos->rx_dma_buf) {
        debug("%s: memalign(rx_dma_buf) failed\n", __func__);
        ret = -ENOMEM;
        goto err_free_tx_dma_buf;
    }
    debug("%s: rx_dma_buf=%p\n", __func__, eqos->rx_dma_buf);

    eqos->rx_pkt = malloc(EQOS_MAX_PACKET_SIZE);
    if (!eqos->rx_pkt) {
        debug("%s: malloc(rx_pkt) failed\n", __func__);
        ret = -ENOMEM;
        goto err_free_rx_dma_buf;
    }
    debug("%s: rx_pkt=%p\n", __func__, eqos->rx_pkt);

    eqos->config->ops->eqos_inval_buffer(eqos->rx_dma_buf,
            EQOS_MAX_PACKET_SIZE * EQOS_DESCRIPTORS_RX);

    debug("%s: OK\n", __func__);
    return 0;

err_free_rx_dma_buf:
    free(eqos->rx_dma_buf);
err_free_tx_dma_buf:
    free(eqos->tx_dma_buf);
err_free_descs:
    eqos_free_descs(eqos->descs);
err:

    debug("%s: returns %d\n", __func__, ret);
    return ret;
}

static int eqos_remove_resources_core(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    free(eqos->rx_pkt);
    free(eqos->rx_dma_buf);
    free(eqos->tx_dma_buf);
    eqos_free_descs(eqos->descs);

    debug("%s: OK\n", __func__);
    return 0;
}

/* board-specific Ethernet Interface initializations. */
__weak int board_interface_eth_init(struct udevice *dev,
                    phy_interface_t interface_type)
{
    return 0;
}

static phy_interface_t eqos_get_interface_eswin(struct udevice *dev)
{
    phy_interface_t interface = PHY_INTERFACE_MODE_MAX;

    debug("%s(dev=%p):\n", __func__, dev);

    // const char *phy_mode;
    // phy_mode = dev_read_prop(dev, "phy-mode", NULL);
    // if (phy_mode)
    //     interface = phy_get_interface_by_name(phy_mode);
    interface = dev_read_phy_mode(dev);

    return interface;
}

static int eqos_probe_resources_eswin(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret;
    phy_interface_t interface;
    struct ofnode_phandle_args phandle_args;
    /* Hardcode for now */
    eqos->phyaddr = -1;

    debug("%s(dev=%p):\n", __func__, dev);

    interface = eqos->config->interface(dev);

    if (interface == PHY_INTERFACE_MODE_MAX) {
        pr_err("Invalid PHY interface\n");
        return -EINVAL;
    }

    ret = board_interface_eth_init(dev, interface);
    if (ret)
        return -EINVAL;

    eqos->max_speed = dev_read_u32_default(dev, "max-speed", 0);

#ifdef ESWIN_ETH_CLK
    ret = clk_get_by_name(dev, "eswinmaceth", &eqos->clk_master_bus);
    if (ret) {
        pr_err("clk_get_by_name(master_bus) failed: %d", ret);
        goto err_probe;
    }

    ret = clk_get_by_name(dev, "mac-clk-rx", &eqos->clk_rx);
    if (ret) {
        pr_err("clk_get_by_name(rx) failed: %d", ret);
        goto err_probe;
    }


    ret = clk_get_by_name(dev, "mac-clk-tx", &eqos->clk_tx);
    if (ret) {
        pr_err("clk_get_by_name(tx) failed: %d", ret);
        goto err_probe;
    }

    ret = clk_get_by_name(dev, "eth-ck", &eqos->clk_ck);
    if (ret)
        pr_warn("No phy clock provided %d", ret);
#endif
    ret = dev_read_phandle_with_args(dev, "phy-handle", NULL, 0, 0,
                     &phandle_args);
    if (!ret) {
        eqos->phyaddr = ofnode_read_u32_default(phandle_args.node,
                            "reg", -1);
    }

    debug("%s: OK\n", __func__);
    return 0;
#ifdef ESWIN_ETH_CLK
err_probe:
    debug("%s: returns %d\n", __func__, ret);
    return ret;
#endif
}


static int eqos_remove_resources_eswin(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

#ifdef ESWIN_ETH_CLK
    if (clk_valid(&eqos->clk_ck))
        clk_free(&eqos->clk_ck);
#endif
    if (dm_gpio_is_valid(eqos->phy_reset_gpio))
        dm_gpio_free(dev, eqos->phy_reset_gpio);

    debug("%s: OK\n", __func__);
    return 0;
}

static int eqos_probe(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);
    int ret;
    int phy_crtl_val;
    int aclk_ctrl_val;
    int hspdma_rst_ctrl;

    debug("%s(dev=%p):\n", __func__, dev);

    ret = dev_read_u32_array(dev, "dly-param-1000m", eqos->dly_param_1000m, 3);
    if (ret) {
        pr_err("error:eth get dly-param-1000m failed, ret=%d\n", ret);
        return ret;
    }
    ret = dev_read_u32_array(dev, "dly-param-100m", eqos->dly_param_100m, 3);
    if (ret) {
        pr_err("error:eth get dly-param-100m failed, ret=%d\n", ret);
        return ret;
    }
    ret = dev_read_u32_array(dev, "dly-param-10m", eqos->dly_param_10m, 3);
    if (ret) {
        pr_err("error:eth get dly-param-10m failed, ret=%d\n", ret);
        return ret;
    }

    eqos->phy_reset_gpio = devm_gpiod_get_optional(dev, "rst", GPIOD_IS_OUT | GPIOD_ACTIVE_LOW);
    if (eqos->phy_reset_gpio) {
        ret = dm_gpio_set_value(eqos->phy_reset_gpio, 1);
        if (ret) {
            pr_err("eth gpio set failed\n");
            return ret;
        }

        udelay(10);

        ret = dm_gpio_set_value(eqos->phy_reset_gpio, 0);
        if (ret) {
            pr_err("eth gpio set failed\n");
            return ret;
        }
    }

    // voltage change from 3.3v to 1.8v
    writel(0x3, (GPIO_REG_BASE + 0x310));
    writel(0x3, (GPIO_REG_BASE + 0x314));

#ifdef ESWIN_FPGA
    writel(ETH_CSYSREQ_VAL, HSP_BASE_ADDR + ETH_AXI_LP_CTRL_OFFSET);
    phy_crtl_val = readl(HSP_BASE_ADDR + ETH_PHY_CTRL_OFFSET);
    phy_crtl_val |= (phy_crtl_val | ETH_TX_CLK_SEL);
    writel(phy_crtl_val, HSP_BASE_ADDR + ETH_PHY_CTRL_OFFSET);
    /* 10M */
    writel(SPEED_10M_CFG, SYSCRG_CSR_BASE + ETH_CTRL_OFFSET);

    aclk_ctrl_val = readl(SYSCRG_CSR_BASE + HSP_ACLK_CTRL_OFFSET);
    aclk_ctrl_val |= (HSP_ACLK_CLKEN | HSP_ACLK_CLKEN);
    writel(aclk_ctrl_val, SYSCRG_CSR_BASE + HSP_ACLK_CTRL_OFFSET);
    //hsp pclk enable
    writel(HSP_CFG_CTRL_REGSET, SYSCRG_CSR_BASE + HSP_CFG_CTRL_OFFSET);
    //hsp rstn
    hspdma_rst_ctrl = readl(SYSCRG_CSR_BASE + HSPDMA_RST_CTRL);
    hspdma_rst_ctrl |= HSP_ETH_ARSTN;
    writel(hspdma_rst_ctrl, SYSCRG_CSR_BASE + HSPDMA_RST_CTRL);
#endif

    eqos->dev = dev;

    eqos->config = (void *)dev_get_driver_data(dev);

    eqos->regs = dev_read_addr(dev);
    if (eqos->regs == FDT_ADDR_T_NONE) {
        pr_err("dev_read_addr() failed");
        return -ENODEV;
    }
    eqos->mac_regs = (void *)(eqos->regs + EQOS_MAC_REGS_BASE);
    eqos->mtl_regs = (void *)(eqos->regs + EQOS_MTL_REGS_BASE);
    eqos->dma_regs = (void *)(eqos->regs + EQOS_DMA_REGS_BASE);

    ret = eqos_probe_resources_core(dev);
    if (ret < 0) {
        pr_err("eqos_probe_resources_core() failed: %d", ret);
        return ret;
    }

    ret = eqos->config->ops->eqos_probe_resources(dev);
    if (ret < 0) {
        pr_err("eqos_probe_resources() failed: %d", ret);
        goto err_remove_resources_core;
    }

#ifdef CONFIG_DM_ETH_PHY
    eqos->mii = eth_phy_get_mdio_bus(dev);
#endif
    if (!eqos->mii) {
        eqos->mii = mdio_alloc();
        if (!eqos->mii) {
            pr_err("mdio_alloc() failed");
            ret = -ENOMEM;
            goto err_remove_resources_tegra;
        }
        eqos->mii->read = eqos_mdio_read;
        eqos->mii->write = eqos_mdio_write;
        eqos->mii->priv = eqos;
        strcpy(eqos->mii->name, dev->name);

        ret = mdio_register(eqos->mii);
        if (ret < 0) {
            pr_err("mdio_register() failed: %d", ret);
            goto err_free_mdio;
        }
    }

#ifdef CONFIG_DM_ETH_PHY
    eth_phy_set_mdio_bus(dev, eqos->mii);
#endif

    debug("%s: OK\n", __func__);
    return 0;

err_free_mdio:
    mdio_free(eqos->mii);
err_remove_resources_tegra:
    eqos->config->ops->eqos_remove_resources(dev);
err_remove_resources_core:
    eqos_remove_resources_core(dev);

    debug("%s: returns %d\n", __func__, ret);
    return ret;
}

static int eqos_remove(struct udevice *dev)
{
    struct eqos_priv *eqos = dev_get_priv(dev);

    debug("%s(dev=%p):\n", __func__, dev);

    mdio_unregister(eqos->mii);
    mdio_free(eqos->mii);
    eqos->config->ops->eqos_remove_resources(dev);

    eqos_probe_resources_core(dev);

    debug("%s: OK\n", __func__);
    return 0;
}

static const struct eth_ops eqos_ops = {
    .start = eqos_start,
    .stop = eqos_stop,
    .send = eqos_send,
    .recv = eqos_recv,
    .free_pkt = eqos_free_pkt,
    .write_hwaddr = eqos_write_hwaddr,
    .read_rom_hwaddr = eqos_read_rom_hwaddr,
};

static struct eqos_ops eqos_eswin_ops = {
    .eqos_inval_desc = eqos_inval_desc_generic,
    .eqos_flush_desc = eqos_flush_desc_generic,
    .eqos_inval_buffer = eqos_inval_buffer_generic,
    .eqos_flush_buffer = eqos_flush_buffer_generic,
    .eqos_probe_resources = eqos_probe_resources_eswin,
    .eqos_remove_resources = eqos_remove_resources_eswin,
    .eqos_stop_resets = eqos_null_ops,
    .eqos_start_resets = eqos_null_ops,
    .eqos_stop_clks = eqos_stop_clks_eswin,
    .eqos_start_clks = eqos_start_clks_eswin,
    .eqos_calibrate_pads = eqos_calibrate_pads_eswin,
    .eqos_disable_calibration = eqos_disable_calibration_eswin,
    .eqos_set_tx_clk_speed = eqos_set_tx_clk_speed_eswin,
    .eqos_get_tick_clk_rate = eqos_get_tick_clk_rate_eswin
};

static const struct eqos_config eqos_eswin_config = {
    .reg_access_always_ok = false,
    .mdio_wait = 10000,
    .swr_wait = 50,
    .config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB,
    .config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_250_300,
    .interface = eqos_get_interface_eswin,
    .ops = &eqos_eswin_ops
};

static const struct udevice_id eqos_ids[] = {
    {
        .compatible = "eswin,eth-eqos",
        .data = (ulong)&eqos_eswin_config
    },
    { }
};

U_BOOT_DRIVER(eth_eqos) = {
    .name = "eth_eqos",
    .id = UCLASS_ETH,
    .of_match = of_match_ptr(eqos_ids),
    .probe = eqos_probe,
    .remove = eqos_remove,
    .ops = &eqos_ops,
    .priv_auto = sizeof(struct eqos_priv),
    .plat_auto = sizeof(struct eth_pdata),
};
