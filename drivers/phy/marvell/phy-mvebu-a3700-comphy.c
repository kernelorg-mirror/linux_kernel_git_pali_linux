// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Marvell
 *
 * Authors:
 *   Evan Wang <xswang@marvell.com>
 *   Miquèl Raynal <miquel.raynal@bootlin.com>
 *
 * Structure inspired from phy-mvebu-cp110-comphy.c written by Antoine Tenart.
 * SMC call initial support done by Grzegorz Jaszczyk.
 * Comphy code from ARM Trusted Firmware ported by Pali Rohár <pali@kernel.org>.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define PLL_SET_DELAY_US			600
#define COMPHY_PLL_SLEEP			1000
#define COMPHY_PLL_TIMEOUT			150000

#define COMPHY_SELECTOR_PHY_REG			0xFC
/* bit0: 0: Lane0 is GBE0; 1: Lane1 is PCIE */
#define COMPHY_SELECTOR_PCIE_GBE0_SEL_BIT	BIT(0)
/* bit4: 0: Lane1 is GBE1; 1: Lane1 is USB3 */
#define COMPHY_SELECTOR_USB3_GBE1_SEL_BIT	BIT(4)
/* bit8: 0: Lane1 is USB, Lane2 is SATA; 1: Lane2 is USB3 */
#define COMPHY_SELECTOR_USB3_PHY_SEL_BIT	BIT(8)

/* SATA PHY register offset */
#define SATAPHY_LANE2_REG_BASE_OFFSET		0x200

/* USB3 PHY offset compared to SATA PHY */
#define USB3PHY_LANE2_REG_BASE_OFFSET		0x200

/* Comphy lane2 indirect access register offset */
#define COMPHY_LANE2_INDIR_ADDR_OFFSET		0x0
#define COMPHY_LANE2_INDIR_DATA_OFFSET		0x4

/* PHY shift to get related register address */
enum {
	PCIE = 1,
	USB3,
};

#define PCIEPHY_SHFT			2
#define USB3PHY_SHFT			2
#define PHY_SHFT(unit)			((unit == PCIE) ? PCIEPHY_SHFT : USB3PHY_SHFT)

/* PHY register */
#define COMPHY_POWER_PLL_CTRL		0x01
#define PWR_PLL_CTRL_ADDR(unit)		(COMPHY_POWER_PLL_CTRL * PHY_SHFT(unit))
#define PU_IVREF_BIT			BIT(15)
#define PU_PLL_BIT			BIT(14)
#define PU_RX_BIT			BIT(13)
#define PU_TX_BIT			BIT(12)
#define PU_TX_INTP_BIT			BIT(11)
#define PU_DFE_BIT			BIT(10)
#define RESET_DTL_RX_BIT		BIT(9)
#define PLL_LOCK_BIT			BIT(8)
#define REF_FREF_SEL_OFFSET		0
#define REF_FREF_SEL_MASK		(0x1F << REF_FREF_SEL_OFFSET)
#define REF_CLOCK_SPEED_25M		(0x1 << REF_FREF_SEL_OFFSET)
#define REF_CLOCK_SPEED_30M		(0x2 << REF_FREF_SEL_OFFSET)
#define PCIE_REF_CLOCK_SPEED_25M	REF_CLOCK_SPEED_30M
#define USB3_REF_CLOCK_SPEED_25M	REF_CLOCK_SPEED_30M
#define REF_CLOCK_SPEED_40M		(0x3 << REF_FREF_SEL_OFFSET)
#define REF_CLOCK_SPEED_50M		(0x4 << REF_FREF_SEL_OFFSET)
#define COMPHY_MODE_OFFSET		5
#define COMPHY_MODE_MASK		(7 << COMPHY_MODE_OFFSET)
#define COMPHY_MODE_SATA		(0x0 << COMPHY_MODE_OFFSET)
#define COMPHY_MODE_PCIE		(0x3 << COMPHY_MODE_OFFSET)
#define COMPHY_MODE_SERDES		(0x4 << COMPHY_MODE_OFFSET)
#define COMPHY_MODE_USB3		(0x5 << COMPHY_MODE_OFFSET)

#define COMPHY_KVCO_CAL_CTRL		0x02
#define KVCO_CAL_CTRL_ADDR(unit)	(COMPHY_KVCO_CAL_CTRL * PHY_SHFT(unit))
#define USE_MAX_PLL_RATE_BIT		BIT(12)
#define SPEED_PLL_OFFSET		2
#define SPEED_PLL_MASK			(0x3F << SPEED_PLL_OFFSET)
#define SPEED_PLL_VALUE_16		(0x10 << SPEED_PLL_OFFSET)

#define COMPHY_RESERVED_REG		0x0E
#define PHYCTRL_FRM_PIN_BIT		BIT(13)

#define COMPHY_LOOPBACK_REG0		0x23
#define DIG_LB_EN_ADDR(unit)		(COMPHY_LOOPBACK_REG0 * PHY_SHFT(unit))
#define SEL_DATA_WIDTH_OFFSET		10
#define SEL_DATA_WIDTH_MASK		(0x3 << SEL_DATA_WIDTH_OFFSET)
#define DATA_WIDTH_10BIT		(0x0 << SEL_DATA_WIDTH_OFFSET)
#define DATA_WIDTH_20BIT		(0x1 << SEL_DATA_WIDTH_OFFSET)
#define DATA_WIDTH_40BIT		(0x2 << SEL_DATA_WIDTH_OFFSET)
#define PLL_READY_TX_BIT		BIT(4)

#define COMPHY_SYNC_PATTERN_REG		0x24
#define SYNC_PATTERN_REG_ADDR(unit)	(COMPHY_SYNC_PATTERN_REG * PHY_SHFT(unit))
#define TXD_INVERT_BIT			BIT(10)
#define RXD_INVERT_BIT			BIT(11)

#define COMPHY_SYNC_MASK_GEN_REG	0x25
#define PHY_GEN_MAX_OFFSET		10
#define PHY_GEN_MAX_MASK		(3 << PHY_GEN_MAX_OFFSET)
#define PHY_GEN_USB3_5G			(1 << PHY_GEN_MAX_OFFSET)

#define COMPHY_ISOLATION_CTRL_REG	0x26
#define ISOLATION_CTRL_REG_ADDR(unit)	(COMPHY_ISOLATION_CTRL_REG * PHY_SHFT(unit))
#define PHY_ISOLATE_MODE		BIT(15)

#define COMPHY_MISC_REG0_ADDR		0x4F
#define MISC_REG0_ADDR(unit)		(COMPHY_MISC_REG0_ADDR * PHY_SHFT(unit))
#define CLK100M_125M_EN			BIT(4)
#define TXDCLK_2X_SEL			BIT(6)
#define CLK500M_EN			BIT(7)
#define PHY_REF_CLK_SEL			BIT(10)
#define MISC_REG0_DEFAULT_VALUE		0xA00D

#define COMPHY_REG_GEN2_SET_2		0x3e
#define GEN2_SETTING_2_ADDR(unit)	(COMPHY_REG_GEN2_SET_2 * PHY_SHFT(unit))
#define G2_TX_SSC_AMP_VALUE_20		BIT(14)
#define G2_TX_SSC_AMP_OFF		9
#define G2_TX_SSC_AMP_LEN		7
#define G2_TX_SSC_AMP_MASK		(((1 << G2_TX_SSC_AMP_LEN) - 1) << G2_TX_SSC_AMP_OFF)

#define COMPHY_REG_GEN2_SET_3		0x3f
#define GEN2_SETTING_3_ADDR(unit)	(COMPHY_REG_GEN2_SET_3 * PHY_SHFT(unit))
#define G3_TX_SSC_AMP_OFF		9
#define G3_TX_SSC_AMP_LEN		7
#define G3_TX_SSC_AMP_MASK		(((1 << G2_TX_SSC_AMP_LEN) - 1) << G2_TX_SSC_AMP_OFF)
#define G3_VREG_RXTX_MAS_ISET_OFF	7
#define G3_VREG_RXTX_MAS_ISET_60U	(0 << G3_VREG_RXTX_MAS_ISET_OFF)
#define G3_VREG_RXTX_MAS_ISET_80U	(1 << G3_VREG_RXTX_MAS_ISET_OFF)
#define G3_VREG_RXTX_MAS_ISET_100U	(2 << G3_VREG_RXTX_MAS_ISET_OFF)
#define G3_VREG_RXTX_MAS_ISET_120U	(3 << G3_VREG_RXTX_MAS_ISET_OFF)
#define G3_VREG_RXTX_MAS_ISET_MASK	(BIT(7) | BIT(8))
#define RSVD_PH03FH_6_0_OFF		0
#define RSVD_PH03FH_6_0_LEN		7
#define RSVD_PH03FH_6_0_MASK		(((1 << RSVD_PH03FH_6_0_LEN) - 1) << RSVD_PH03FH_6_0_OFF)

#define COMPHY_REG_UNIT_CTRL_ADDR	0x48
#define UNIT_CTRL_ADDR(unit)		(COMPHY_REG_UNIT_CTRL_ADDR * PHY_SHFT(unit))
#define IDLE_SYNC_EN			BIT(12)
#define UNIT_CTRL_DEFAULT_VALUE		0x60

#define COMPHY_MISC_REG1_ADDR		0x73
#define MISC_REG1_ADDR(unit)		(COMPHY_MISC_REG1_ADDR * PHY_SHFT(unit))
#define SEL_BITS_PCIE_FORCE		BIT(15)

#define COMPHY_REG_GEN3_SETTINGS_3	0x112
#define COMPHY_GEN_FFE_CAP_SEL_MASK	0xF
#define COMPHY_GEN_FFE_CAP_SEL_VALUE	0xF

#define COMPHY_REG_LANE_CFG0_ADDR	0x180
#define LANE_CFG0_ADDR(unit)		(COMPHY_REG_LANE_CFG0_ADDR * PHY_SHFT(unit))
#define PRD_TXDEEMPH0_MASK		BIT(0)
#define PRD_TXMARGIN_MASK		(BIT(1) | BIT(2) | BIT(3))
#define PRD_TXSWING_MASK		BIT(4)
#define CFG_TX_ALIGN_POS_MASK		(BIT(5) | BIT(6) | BIT(7) | BIT(8))

#define COMPHY_REG_LANE_CFG1_ADDR	0x181
#define LANE_CFG1_ADDR(unit)		(COMPHY_REG_LANE_CFG1_ADDR * PHY_SHFT(unit))
#define PRD_TXDEEMPH1_MASK		BIT(15)
#define USE_MAX_PLL_RATE_EN		BIT(9)
#define TX_DET_RX_MODE			BIT(6)
#define GEN2_TX_DATA_DLY_MASK		(BIT(3) | BIT(4))
#define GEN2_TX_DATA_DLY_DEFT		(2 << 3)
#define TX_ELEC_IDLE_MODE_EN		BIT(0)

#define COMPHY_REG_LANE_STATUS1_ADDR	0x183
#define LANE_STATUS1_ADDR(unit)		(COMPHY_REG_LANE_STATUS1_ADDR * PHY_SHFT(unit))
#define TXDCLK_PCLK_EN			BIT(0)

#define COMPHY_REG_LANE_CFG4_ADDR	0x188
#define LANE_CFG4_ADDR(unit)		(COMPHY_REG_LANE_CFG4_ADDR * PHY_SHFT(unit))
#define SPREAD_SPECTRUM_CLK_EN		BIT(7)

#define COMPHY_REG_GLOB_PHY_CTRL0_ADDR	0x1C1
#define GLOB_PHY_CTRL0_ADDR(unit)	(COMPHY_REG_GLOB_PHY_CTRL0_ADDR * PHY_SHFT(unit))
#define SOFT_RESET			BIT(0)
#define MODE_REFDIV			0x30
#define MODE_CORE_CLK_FREQ_SEL		BIT(9)
#define MODE_PIPE_WIDTH_32		BIT(3)
#define MODE_REFDIV_OFFSET		4
#define MODE_REFDIV_LEN			2
#define MODE_REFDIV_MASK		(0x3 << MODE_REFDIV_OFFSET)
#define MODE_REFDIV_BY_4		(0x2 << MODE_REFDIV_OFFSET)

#define COMPHY_REG_TEST_MODE_CTRL_ADDR	0x1C2
#define TEST_MODE_CTRL_ADDR(unit)	(COMPHY_REG_TEST_MODE_CTRL_ADDR * PHY_SHFT(unit))
#define MODE_MARGIN_OVERRIDE		BIT(2)

#define COMPHY_REG_GLOB_CLK_SRC_LO_ADDR	0x1C3
#define GLOB_CLK_SRC_LO_ADDR(unit)	(COMPHY_REG_GLOB_CLK_SRC_LO_ADDR * PHY_SHFT(unit))
#define MODE_CLK_SRC			BIT(0)
#define BUNDLE_PERIOD_SEL		BIT(1)
#define BUNDLE_PERIOD_SCALE		(BIT(2) | BIT(3))
#define BUNDLE_SAMPLE_CTRL		BIT(4)
#define PLL_READY_DLY			(BIT(5) | BIT(6) | BIT(7))
#define CFG_SEL_20B			BIT(15)

#define COMPHY_REG_PWR_MGM_TIM1_ADDR	0x1D0
#define PWR_MGM_TIM1_ADDR(unit)		(COMPHY_REG_PWR_MGM_TIM1_ADDR * PHY_SHFT(unit))
#define CFG_PM_OSCCLK_WAIT_OFF		12
#define CFG_PM_OSCCLK_WAIT_LEN		4
#define CFG_PM_OSCCLK_WAIT_MASK		(((1 << CFG_PM_OSCCLK_WAIT_LEN) - 1) << CFG_PM_OSCCLK_WAIT_OFF)
#define CFG_PM_RXDEN_WAIT_OFF		8
#define CFG_PM_RXDEN_WAIT_LEN		4
#define CFG_PM_RXDEN_WAIT_MASK		(((1 << CFG_PM_RXDEN_WAIT_LEN) - 1) << CFG_PM_RXDEN_WAIT_OFF)
#define CFG_PM_RXDEN_WAIT_1_UNIT	(1 << CFG_PM_RXDEN_WAIT_OFF)
#define CFG_PM_RXDLOZ_WAIT_OFF		0
#define CFG_PM_RXDLOZ_WAIT_LEN		8
#define CFG_PM_RXDLOZ_WAIT_MASK		(((1 << CFG_PM_RXDLOZ_WAIT_LEN) - 1) << CFG_PM_RXDLOZ_WAIT_OFF)
#define CFG_PM_RXDLOZ_WAIT_7_UNIT	(7 << CFG_PM_RXDLOZ_WAIT_OFF)
#define CFG_PM_RXDLOZ_WAIT_12_UNIT	(0xC << CFG_PM_RXDLOZ_WAIT_OFF)

/* SERDES */
#define COMPHY_PHY_CFG1_OFFSET(lane)	((1 - (lane)) * 0x28)
#define PIN_PU_IVEREF_BIT		BIT(1)
#define PIN_RESET_CORE_BIT		BIT(11)
#define PIN_RESET_COMPHY_BIT		BIT(12)
#define PIN_PU_PLL_BIT			BIT(16)
#define PIN_PU_RX_BIT			BIT(17)
#define PIN_PU_TX_BIT			BIT(18)
#define PIN_TX_IDLE_BIT			BIT(19)
#define GEN_RX_SEL_OFFSET		22
#define GEN_RX_SEL_MASK			(0xF << GEN_RX_SEL_OFFSET)
#define GEN_TX_SEL_OFFSET		26
#define GEN_TX_SEL_MASK			(0xF << GEN_TX_SEL_OFFSET)
#define PHY_RX_INIT_BIT			BIT(30)
#define SERDES_SPEED_1_25_G		0x6
#define SERDES_SPEED_2_5_G		0x8

/* COMPHY status reg:
 * lane0: PCIe/GbE0 PHY Status 1
 * lane1: USB3/GbE1 PHY Status 1
 */
#define COMPHY_PHY_STATUS_OFFSET(lane)	(0x18 + (1 - (lane)) * 0x28)
#define PHY_RX_INIT_DONE_BIT		BIT(0)
#define PHY_PLL_READY_RX_BIT		BIT(2)
#define PHY_PLL_READY_TX_BIT		BIT(3)

#define GBE_PHY_ADDR(base, off)		((base) + (((off) & 0x00007FF) * 2))

struct mvebu_a3700_comphy_conf {
	unsigned int lane;
	enum phy_mode mode;
	int submode;
};

#define MVEBU_A3700_COMPHY_CONF(_lane, _mode, _smode)			\
	{								\
		.lane = _lane,						\
		.mode = _mode,						\
		.submode = _smode,					\
	}

#define MVEBU_A3700_COMPHY_CONF_GEN(_lane, _mode) \
	MVEBU_A3700_COMPHY_CONF(_lane, _mode, PHY_INTERFACE_MODE_NA)

#define MVEBU_A3700_COMPHY_CONF_ETH(_lane, _smode) \
	MVEBU_A3700_COMPHY_CONF(_lane, PHY_MODE_ETHERNET, _smode)

static const struct mvebu_a3700_comphy_conf mvebu_a3700_comphy_modes[] = {
	/* lane 0 */
	MVEBU_A3700_COMPHY_CONF_GEN(0, PHY_MODE_USB_HOST_SS),
	MVEBU_A3700_COMPHY_CONF_ETH(0, PHY_INTERFACE_MODE_SGMII),
	MVEBU_A3700_COMPHY_CONF_ETH(0, PHY_INTERFACE_MODE_1000BASEX),
	MVEBU_A3700_COMPHY_CONF_ETH(0, PHY_INTERFACE_MODE_2500BASEX),
	/* lane 1 */
	MVEBU_A3700_COMPHY_CONF_GEN(1, PHY_MODE_PCIE),
	MVEBU_A3700_COMPHY_CONF_ETH(1, PHY_INTERFACE_MODE_SGMII),
	MVEBU_A3700_COMPHY_CONF_ETH(1, PHY_INTERFACE_MODE_1000BASEX),
	MVEBU_A3700_COMPHY_CONF_ETH(1, PHY_INTERFACE_MODE_2500BASEX),
	/* lane 2 */
	MVEBU_A3700_COMPHY_CONF_GEN(2, PHY_MODE_SATA),
	MVEBU_A3700_COMPHY_CONF_GEN(2, PHY_MODE_USB_HOST_SS),
};

struct mvebu_a3700_comphy_priv {
	void __iomem *comphy_regs;
	void __iomem *pcie_gbe0_phy_regs;
	void __iomem *usb3_gbe1_phy_regs;
	void __iomem *sata_usb3_phy_regs;
	bool xtal_is_40m;
};

struct mvebu_a3700_comphy_lane {
	struct mvebu_a3700_comphy_priv *priv;
	struct device *dev;
	unsigned int id;
	enum phy_mode mode;
	int submode;
	bool invert_tx;
	bool invert_rx;
};

struct gbe_phy_init_data_fix {
	u16 addr;
	u16 value;
};

/* Changes to 40M1G25 mode data required for running 40M3G125 init mode */
static struct gbe_phy_init_data_fix gbe_phy_init_fix[] = {
	{0x005, 0x07CC}, {0x015, 0x0000}, {0x01B, 0x0000}, {0x01D, 0x0000},
	{0x01E, 0x0000}, {0x01F, 0x0000}, {0x020, 0x0000}, {0x021, 0x0030},
	{0x026, 0x0888}, {0x04D, 0x0152}, {0x04F, 0xA020}, {0x050, 0x07CC},
	{0x053, 0xE9CA}, {0x055, 0xBD97}, {0x071, 0x3015}, {0x076, 0x03AA},
	{0x07C, 0x0FDF}, {0x0C2, 0x3030}, {0x0C3, 0x8000}, {0x0E2, 0x5550},
	{0x0E3, 0x12A4}, {0x0E4, 0x7D00}, {0x0E6, 0x0C83}, {0x101, 0xFCC0},
	{0x104, 0x0C10}
};

/* 40M1G25 mode init data */
static u16 gbe_phy_init[512] = {
	/* 0       1       2       3       4       5       6       7 */
	/*-----------------------------------------------------------*/
	/* 8       9       A       B       C       D       E       F */
	0x3110, 0xFD83, 0x6430, 0x412F, 0x82C0, 0x06FA, 0x4500, 0x6D26,	/* 00 */
	0xAFC0, 0x8000, 0xC000, 0x0000, 0x2000, 0x49CC, 0x0BC9, 0x2A52,	/* 08 */
	0x0BD2, 0x0CDE, 0x13D2, 0x0CE8, 0x1149, 0x10E0, 0x0000, 0x0000,	/* 10 */
	0x0000, 0x0000, 0x0000, 0x0001, 0x0000, 0x4134, 0x0D2D, 0xFFFF,	/* 18 */
	0xFFE0, 0x4030, 0x1016, 0x0030, 0x0000, 0x0800, 0x0866, 0x0000,	/* 20 */
	0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,	/* 28 */
	0xFFFF, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* 30 */
	0x0000, 0x0000, 0x000F, 0x6A62, 0x1988, 0x3100, 0x3100, 0x3100,	/* 38 */
	0x3100, 0xA708, 0x2430, 0x0830, 0x1030, 0x4610, 0xFF00, 0xFF00,	/* 40 */
	0x0060, 0x1000, 0x0400, 0x0040, 0x00F0, 0x0155, 0x1100, 0xA02A,	/* 48 */
	0x06FA, 0x0080, 0xB008, 0xE3ED, 0x5002, 0xB592, 0x7A80, 0x0001,	/* 50 */
	0x020A, 0x8820, 0x6014, 0x8054, 0xACAA, 0xFC88, 0x2A02, 0x45CF,	/* 58 */
	0x000F, 0x1817, 0x2860, 0x064F, 0x0000, 0x0204, 0x1800, 0x6000,	/* 60 */
	0x810F, 0x4F23, 0x4000, 0x4498, 0x0850, 0x0000, 0x000E, 0x1002,	/* 68 */
	0x9D3A, 0x3009, 0xD066, 0x0491, 0x0001, 0x6AB0, 0x0399, 0x3780,	/* 70 */
	0x0040, 0x5AC0, 0x4A80, 0x0000, 0x01DF, 0x0000, 0x0007, 0x0000,	/* 78 */
	0x2D54, 0x00A1, 0x4000, 0x0100, 0xA20A, 0x0000, 0x0000, 0x0000,	/* 80 */
	0x0000, 0x0000, 0x0000, 0x7400, 0x0E81, 0x1000, 0x1242, 0x0210,	/* 88 */
	0x80DF, 0x0F1F, 0x2F3F, 0x4F5F, 0x6F7F, 0x0F1F, 0x2F3F, 0x4F5F,	/* 90 */
	0x6F7F, 0x4BAD, 0x0000, 0x0000, 0x0800, 0x0000, 0x2400, 0xB651,	/* 98 */
	0xC9E0, 0x4247, 0x0A24, 0x0000, 0xAF19, 0x1004, 0x0000, 0x0000,	/* A0 */
	0x0000, 0x0013, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* A8 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/* B0 */
	0x0000, 0x0000, 0x0000, 0x0060, 0x0000, 0x0000, 0x0000, 0x0000,	/* B8 */
	0x0000, 0x0000, 0x3010, 0xFA00, 0x0000, 0x0000, 0x0000, 0x0003,	/* C0 */
	0x1618, 0x8200, 0x8000, 0x0400, 0x050F, 0x0000, 0x0000, 0x0000,	/* C8 */
	0x4C93, 0x0000, 0x1000, 0x1120, 0x0010, 0x1242, 0x1242, 0x1E00,	/* D0 */
	0x0000, 0x0000, 0x0000, 0x00F8, 0x0000, 0x0041, 0x0800, 0x0000,	/* D8 */
	0x82A0, 0x572E, 0x2490, 0x14A9, 0x4E00, 0x0000, 0x0803, 0x0541,	/* E0 */
	0x0C15, 0x0000, 0x0000, 0x0400, 0x2626, 0x0000, 0x0000, 0x4200,	/* E8 */
	0x0000, 0xAA55, 0x1020, 0x0000, 0x0000, 0x5010, 0x0000, 0x0000,	/* F0 */
	0x0000, 0x0000, 0x5000, 0x0000, 0x0000, 0x0000, 0x02F2, 0x0000,	/* F8 */
	0x101F, 0xFDC0, 0x4000, 0x8010, 0x0110, 0x0006, 0x0000, 0x0000,	/*100 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*108 */
	0x04CF, 0x0000, 0x04CF, 0x0000, 0x04CF, 0x0000, 0x04C6, 0x0000,	/*110 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*118 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*120 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*128 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*130 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*138 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*140 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*148 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*150 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*158 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*160 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*168 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*170 */
	0x0000, 0x0000, 0x0000, 0x00F0, 0x08A2, 0x3112, 0x0A14, 0x0000,	/*178 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*180 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*188 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*190 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*198 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1A0 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1A8 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1B0 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1B8 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1C0 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1C8 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1D0 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1D8 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1E0 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1E8 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,	/*1F0 */
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000	/*1F8 */
};

static inline void reg_set(void __iomem *addr, u32 data, u32 mask)
{
	u32 reg_data;

	reg_data = readl(addr);
	reg_data &= ~mask;
	reg_data |= data;
	writel(reg_data, addr);
}

static inline void reg_set16(void __iomem *addr, u16 data, u16 mask)
{
	u16 reg_data;

	reg_data = readw(addr);
	reg_data &= ~mask;
	reg_data |= data;
	writew(reg_data, addr);
}

/* PHY selector configures with corresponding modes */
static int mvebu_a3700_comphy_set_phy_selector(struct mvebu_a3700_comphy_lane *lane)
{
	u32 reg;

	reg = readl(lane->priv->comphy_regs + COMPHY_SELECTOR_PHY_REG);
	switch (lane->mode) {
	case PHY_MODE_SATA:
		/* SATA must be in Lane2 */
		if (lane->id == 2)
			reg &= ~COMPHY_SELECTOR_USB3_PHY_SEL_BIT;
		else
			goto error;
		break;

	case PHY_MODE_ETHERNET:
		if (lane->id == 0)
			reg &= ~COMPHY_SELECTOR_USB3_GBE1_SEL_BIT;
		else if (lane->id == 1)
			reg &= ~COMPHY_SELECTOR_PCIE_GBE0_SEL_BIT;
		else
			goto error;
		break;

	case PHY_MODE_USB_HOST_SS:
		if (lane->id == 2)
			reg |= COMPHY_SELECTOR_USB3_PHY_SEL_BIT;
		else if (lane->id == 0)
			reg |= COMPHY_SELECTOR_USB3_GBE1_SEL_BIT;
		else
			goto error;
		break;

	case PHY_MODE_PCIE:
		/* PCIE must be in Lane1 */
		if (lane->id == 1)
			reg |= COMPHY_SELECTOR_PCIE_GBE0_SEL_BIT;
		else
			goto error;
		break;

	default:
		goto error;
	}

	writel(reg, lane->priv->comphy_regs + COMPHY_SELECTOR_PHY_REG);
	return 0;
error:
	dev_err(lane->dev, "COMPHY[%d] mode[%d] is invalid\n", lane->id, lane->mode);
	return -EINVAL;
}

/* It is only used for SATA and USB3 on comphy lane2. */
static void comphy_set_indirect(void __iomem *base_addr, u32 offset, u16 data,
				u16 mask, bool is_sata)
{
	/*
	 * When Lane 2 PHY is for USB3, access the PHY registers
	 * through indirect Address and Data registers:
	 * INDIR_ACC_PHY_ADDR (RD00E0178h [31:0]),
	 * INDIR_ACC_PHY_DATA (RD00E017Ch [31:0]),
	 * within the SATA Host Controller registers, Lane 2 base register
	 * offset is 0x200
	 */
	if (is_sata)
		writel(offset, base_addr + COMPHY_LANE2_INDIR_ADDR_OFFSET);
	else
		writel(USB3PHY_LANE2_REG_BASE_OFFSET + offset,
		       base_addr + COMPHY_LANE2_INDIR_ADDR_OFFSET);

	reg_set(base_addr + COMPHY_LANE2_INDIR_DATA_OFFSET, data, mask);
}

/* It is only used for SATA on comphy lane2. */
static void comphy_sata_set_indirect(void __iomem *base_addr, u32 reg_offset,
				     u16 data, u16 mask)
{
	comphy_set_indirect(base_addr, reg_offset, data, mask, true);
}

/* It is only used for USB3 indirect access on comphy lane2. */
static void comphy_usb3_set_indirect(void __iomem *base_addr, u32 reg_offset,
				     u16 data, u16 mask)
{
	comphy_set_indirect(base_addr, reg_offset, data, mask, false);
}

/* It is only used for USB3 direct access not on comphy lane2. */
static void comphy_usb3_set_direct(void __iomem *base_addr, u32 reg_offset,
				   u16 data, u16 mask)
{
	reg_set16(base_addr + reg_offset * PHY_SHFT(USB3), data, mask);
}

static void comphy_gbe_phy_init(void __iomem *gbe_phy_base, bool is_1gbps)
{
	const int fix_arr_sz = ARRAY_SIZE(gbe_phy_init_fix);
	int addr, fix_idx;
	u16 val;

	fix_idx = 0;
	for (addr = 0; addr < 512; addr++) {
		/*
		 * All PHY register values are defined in full for 3.125Gbps
		 * SERDES speed. The values required for 1.25 Gbps are almost
		 * the same and only few registers should be "fixed" in
		 * comparison to 3.125 Gbps values. These register values are
		 * stored in "gbe_phy_init_fix" array.
		 */
		if (!is_1gbps && gbe_phy_init_fix[fix_idx].addr == addr) {
			/* Use new value */
			val = gbe_phy_init_fix[fix_idx].value;
			if (fix_idx < fix_arr_sz)
				fix_idx++;
		} else {
			val = gbe_phy_init[addr];
		}

		reg_set16(GBE_PHY_ADDR(gbe_phy_base, addr), val, 0xFFFF);
	}
}

static int mvebu_a3700_comphy_sata_power_on(struct mvebu_a3700_comphy_lane *lane)
{
	void __iomem *sata_phy_base = lane->priv->sata_usb3_phy_regs;
	u32 offset, data = 0, ref_clk;
	int ret;

	/* Configure phy selector for SATA */
	ret = mvebu_a3700_comphy_set_phy_selector(lane);
	if (ret)
		return ret;

	/* Clear phy isolation mode to make it work in normal mode */
	offset = SATAPHY_LANE2_REG_BASE_OFFSET + COMPHY_ISOLATION_CTRL_REG;
	comphy_sata_set_indirect(sata_phy_base, offset, 0, PHY_ISOLATE_MODE);

	/* 0. Check the Polarity invert bits */
	if (lane->invert_tx)
		data |= TXD_INVERT_BIT;
	if (lane->invert_rx)
		data |= RXD_INVERT_BIT;

	offset = SATAPHY_LANE2_REG_BASE_OFFSET + COMPHY_SYNC_PATTERN_REG;
	comphy_sata_set_indirect(sata_phy_base, offset, data, TXD_INVERT_BIT |
				 RXD_INVERT_BIT);

	/* 1. Select 40-bit data width width */
	offset = SATAPHY_LANE2_REG_BASE_OFFSET + COMPHY_LOOPBACK_REG0;
	comphy_sata_set_indirect(sata_phy_base, offset, DATA_WIDTH_40BIT,
				 SEL_DATA_WIDTH_MASK);

	/* 2. Select reference clock(25M) and PHY mode (SATA) */
	offset = SATAPHY_LANE2_REG_BASE_OFFSET + COMPHY_POWER_PLL_CTRL;
	if (lane->priv->xtal_is_40m)
		ref_clk = REF_CLOCK_SPEED_40M;
	else
		ref_clk = REF_CLOCK_SPEED_25M;

	comphy_sata_set_indirect(sata_phy_base, offset, ref_clk | COMPHY_MODE_SATA,
				 REF_FREF_SEL_MASK | COMPHY_MODE_MASK);

	/* 3. Use maximum PLL rate (no power save) */
	offset = SATAPHY_LANE2_REG_BASE_OFFSET + COMPHY_KVCO_CAL_CTRL;
	comphy_sata_set_indirect(sata_phy_base, offset, USE_MAX_PLL_RATE_BIT,
				 USE_MAX_PLL_RATE_BIT);

	/* 4. Reset reserved bit */
	comphy_sata_set_indirect(sata_phy_base, COMPHY_RESERVED_REG, 0,
				 PHYCTRL_FRM_PIN_BIT);

	/* 5. Set vendor-specific configuration (It is done in sata driver) */
	/* XXX: in U-Boot below sequence was executed in this place, in Linux
	 * not.  Now it is done only in U-Boot before this comphy
	 * initialization - tests shows that it works ok, but in case of any
	 * future problem it is left for reference.
	 *   reg_set(MVEBU_REGS_BASE + 0xe00a0, 0, 0xffffffff);
	 *   reg_set(MVEBU_REGS_BASE + 0xe00a4, BIT(6), BIT(6));
	 */

	/* Wait for > 55 us to allow PLL be enabled */
	udelay(PLL_SET_DELAY_US);

	/* Polling status */
	writel(SATAPHY_LANE2_REG_BASE_OFFSET + COMPHY_LOOPBACK_REG0,
	       sata_phy_base + COMPHY_LANE2_INDIR_ADDR_OFFSET);

	ret = readl_poll_timeout(sata_phy_base + COMPHY_LANE2_INDIR_DATA_OFFSET,
				 data, data & PLL_READY_TX_BIT,
				 COMPHY_PLL_SLEEP, COMPHY_PLL_TIMEOUT);
	if (ret) {
		dev_err(lane->dev, "Failed to lock SATA PLL\n");
		return ret;
	}

	return 0;
}

static int mvebu_a3700_comphy_ethernet_power_on(struct mvebu_a3700_comphy_lane *lane)
{
	void __iomem *comphy_base = lane->priv->comphy_regs;
	void __iomem *gbe_phy_base;
	u32 mask, data;
	int ret;

	/* Set selector */
	ret = mvebu_a3700_comphy_set_phy_selector(lane);
	if (ret)
		return ret;

	/* GBE PHY Base address
	 * COMPHY Lane0 -- USB3/GBE1
	 * COMPHY Lane1 -- PCIe/GBE0
	 */
	if (lane->id == 0) {
		/* Get usb3 and gbe register base address */
		gbe_phy_base = lane->priv->usb3_gbe1_phy_regs;
	} else {
		/* Get pcie and gbe register base address */
		gbe_phy_base = lane->priv->pcie_gbe0_phy_regs;
	}

	/*
	 * 1. Reset PHY by setting PHY input port PIN_RESET=1.
	 * 2. Set PHY input port PIN_TX_IDLE=1, PIN_PU_IVREF=1 to keep
	 *    PHY TXP/TXN output to idle state during PHY initialization
	 * 3. Set PHY input port PIN_PU_PLL=0, PIN_PU_RX=0, PIN_PU_TX=0.
	 */
	data = PIN_PU_IVEREF_BIT | PIN_TX_IDLE_BIT | PIN_RESET_COMPHY_BIT;
	mask = PIN_RESET_CORE_BIT | PIN_PU_PLL_BIT | PIN_PU_RX_BIT |
		PIN_PU_TX_BIT | PHY_RX_INIT_BIT;
	reg_set(comphy_base + COMPHY_PHY_CFG1_OFFSET(lane->id), data, mask);

	/* 4. Release reset to the PHY by setting PIN_RESET=0. */
	data = 0;
	mask = PIN_RESET_COMPHY_BIT;
	reg_set(comphy_base + COMPHY_PHY_CFG1_OFFSET(lane->id), data, mask);

	/*
	 * 5. Set PIN_PHY_GEN_TX[3:0] and PIN_PHY_GEN_RX[3:0] to decide COMPHY
	 * bit rate
	 */
	switch (lane->submode) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		/* SGMII 1G, SerDes speed 1.25G */
		data |= SERDES_SPEED_1_25_G << GEN_RX_SEL_OFFSET;
		data |= SERDES_SPEED_1_25_G << GEN_TX_SEL_OFFSET;
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		/* 2500Base-X, SerDes speed 3.125G */
		data |= SERDES_SPEED_2_5_G << GEN_RX_SEL_OFFSET;
		data |= SERDES_SPEED_2_5_G << GEN_TX_SEL_OFFSET;
		break;
	default:
		/* Other rates are not supported */
		dev_err(lane->dev, "unsupported phy speed %d on comphy lane%d\n",
			lane->submode, lane->id);
		return -EINVAL;
	}
	mask = GEN_RX_SEL_MASK | GEN_TX_SEL_MASK;
	reg_set(comphy_base + COMPHY_PHY_CFG1_OFFSET(lane->id), data, mask);

	/*
	 * 6. Wait 10mS for bandgap and reference clocks to stabilize; then
	 * start SW programming.
	 */
	mdelay(10);

	/* 7. Program COMPHY register PHY_MODE */
	data = COMPHY_MODE_SERDES;
	mask = COMPHY_MODE_MASK;
	reg_set16(GBE_PHY_ADDR(gbe_phy_base, COMPHY_POWER_PLL_CTRL), data, mask);

	/*
	 * 8. Set COMPHY register REFCLK_SEL to select the correct REFCLK
	 * source
	 */
	data = 0;
	mask = PHY_REF_CLK_SEL;
	reg_set16(GBE_PHY_ADDR(gbe_phy_base, COMPHY_MISC_REG0_ADDR), data, mask);

	/*
	 * 9. Set correct reference clock frequency in COMPHY register
	 * REF_FREF_SEL.
	 */
	if (lane->priv->xtal_is_40m)
		data = REF_CLOCK_SPEED_50M;
	else
		data = REF_CLOCK_SPEED_25M;

	mask = REF_FREF_SEL_MASK;
	reg_set16(GBE_PHY_ADDR(gbe_phy_base, COMPHY_POWER_PLL_CTRL), data, mask);

	/* 10. Program COMPHY register PHY_GEN_MAX[1:0]
	 * This step is mentioned in the flow received from verification team.
	 * However the PHY_GEN_MAX value is only meaningful for other interfaces
	 * (not SERDES). For instance, it selects SATA speed 1.5/3/6 Gbps or PCIe
	 * speed 2.5/5 Gbps
	 */

	/*
	 * 11. Program COMPHY register SEL_BITS to set correct parallel data
	 * bus width
	 */
	data = DATA_WIDTH_10BIT;
	mask = SEL_DATA_WIDTH_MASK;
	reg_set16(GBE_PHY_ADDR(gbe_phy_base, COMPHY_LOOPBACK_REG0), data, mask);

	/*
	 * 12. As long as DFE function needs to be enabled in any mode,
	 * COMPHY register DFE_UPDATE_EN[5:0] shall be programmed to 0x3F
	 * for real chip during COMPHY power on.
	 * The step 14 exists (and empty) in the original initialization flow
	 * obtained from the verification team. According to the functional
	 * specification DFE_UPDATE_EN already has the default value 0x3F
	 */

	/*
	 * 13. Program COMPHY GEN registers.
	 * These registers should be programmed based on the lab testing result
	 * to achieve optimal performance. Please contact the CEA group to get
	 * the related GEN table during real chip bring-up. We only required to
	 * run though the entire registers programming flow defined by
	 * "comphy_gbe_phy_init" when the REF clock is 40 MHz. For REF clock
	 * 25 MHz the default values stored in PHY registers are OK.
	 */
	dev_dbg(lane->dev, "Running C-DPI phy init %s mode\n",
		lane->submode == PHY_INTERFACE_MODE_2500BASEX ? "2G5" : "1G");
	if (lane->priv->xtal_is_40m)
		comphy_gbe_phy_init(gbe_phy_base, lane->submode != PHY_INTERFACE_MODE_2500BASEX);

	/*
	 * 14. [Simulation Only] should not be used for real chip.
	 * By pass power up calibration by programming EXT_FORCE_CAL_DONE
	 * (R02h[9]) to 1 to shorten COMPHY simulation time.
	 */

	/*
	 * 15. [Simulation Only: should not be used for real chip]
	 * Program COMPHY register FAST_DFE_TIMER_EN=1 to shorten RX training
	 * simulation time.
	 */

	/*
	 * 16. Check the PHY Polarity invert bit
	 */
	data = 0x0;
	if (lane->invert_tx)
		data |= TXD_INVERT_BIT;
	if (lane->invert_rx)
		data |= RXD_INVERT_BIT;
	mask = TXD_INVERT_BIT | RXD_INVERT_BIT;
	reg_set16(GBE_PHY_ADDR(gbe_phy_base, COMPHY_SYNC_PATTERN_REG), data, mask);

	/*
	 * 17. Set PHY input ports PIN_PU_PLL, PIN_PU_TX and PIN_PU_RX to 1 to
	 * start PHY power up sequence. All the PHY register programming should
	 * be done before PIN_PU_PLL=1. There should be no register programming
	 * for normal PHY operation from this point.
	 */
	reg_set(comphy_base + COMPHY_PHY_CFG1_OFFSET(lane->id),
		PIN_PU_PLL_BIT | PIN_PU_RX_BIT | PIN_PU_TX_BIT,
		PIN_PU_PLL_BIT | PIN_PU_RX_BIT | PIN_PU_TX_BIT);

	/*
	 * 18. Wait for PHY power up sequence to finish by checking output ports
	 * PIN_PLL_READY_TX=1 and PIN_PLL_READY_RX=1.
	 */
	ret = readl_poll_timeout(comphy_base + COMPHY_PHY_STATUS_OFFSET(lane->id),
				 data, (data & (PHY_PLL_READY_TX_BIT | PHY_PLL_READY_RX_BIT)) ==
				               (PHY_PLL_READY_TX_BIT | PHY_PLL_READY_RX_BIT),
				 COMPHY_PLL_SLEEP, COMPHY_PLL_TIMEOUT);
	if (ret) {
		dev_err(lane->dev, "Failed to lock PLL for SERDES PHY %d\n", lane->id);
		return ret;
	}

	/*
	 * 19. Set COMPHY input port PIN_TX_IDLE=0
	 */
	reg_set(comphy_base + COMPHY_PHY_CFG1_OFFSET(lane->id),
		0x0, PIN_TX_IDLE_BIT);

	/*
	 * 20. After valid data appear on PIN_RXDATA bus, set PIN_RX_INIT=1. To
	 * start RX initialization. PIN_RX_INIT_DONE will be cleared to 0 by the
	 * PHY After RX initialization is done, PIN_RX_INIT_DONE will be set to
	 * 1 by COMPHY Set PIN_RX_INIT=0 after PIN_RX_INIT_DONE= 1. Please
	 * refer to RX initialization part for details.
	 */
	reg_set(comphy_base + COMPHY_PHY_CFG1_OFFSET(lane->id),
		PHY_RX_INIT_BIT, PHY_RX_INIT_BIT);

	ret = readl_poll_timeout(comphy_base + COMPHY_PHY_STATUS_OFFSET(lane->id),
				 data, (data & (PHY_PLL_READY_TX_BIT | PHY_PLL_READY_RX_BIT)) ==
				               (PHY_PLL_READY_TX_BIT | PHY_PLL_READY_RX_BIT),
				 COMPHY_PLL_SLEEP, COMPHY_PLL_TIMEOUT);
	if (ret) {
		dev_err(lane->dev, "Failed to lock PLL for SERDES PHY %d\n", lane->id);
		return ret;
	}

	ret = readl_poll_timeout(comphy_base + COMPHY_PHY_STATUS_OFFSET(lane->id),
				 data, data & PHY_RX_INIT_DONE_BIT,
				 COMPHY_PLL_SLEEP, COMPHY_PLL_TIMEOUT);
	if (ret) {
		dev_err(lane->dev, "Failed to init RX of SERDES PHY %d\n", lane->id);
		return ret;
	}

	return 0;
}

static int mvebu_a3700_comphy_usb3_power_on(struct mvebu_a3700_comphy_lane *lane)
{
	void (*usb3_reg_set)(void __iomem *addr, u32 offset, u16 data, u16 mask);
	u32 mask, data, cfg, ref_clk;
	void __iomem *usb3_phy_base;
	int ret;

	/* Set phy seclector */
	ret = mvebu_a3700_comphy_set_phy_selector(lane);
	if (ret)
		return ret;

	/* Set usb3 reg access func, Lane2 is indirect access */
	if (lane->id == 2) {
		usb3_reg_set = &comphy_usb3_set_indirect;
		usb3_phy_base = lane->priv->sata_usb3_phy_regs;
	} else {
		/* Get the direct access register resource and map */
		usb3_reg_set = &comphy_usb3_set_direct;
		usb3_phy_base = lane->priv->usb3_gbe1_phy_regs;
	}

	/*
	 * 0. Set PHY OTG Control(0x5d034), bit 4, Power up OTG module The
	 * register belong to UTMI module, so it is set in UTMI phy driver.
	 */

	/*
	 * 1. Set PRD_TXDEEMPH (3.5db de-emph)
	 */
	mask = PRD_TXDEEMPH0_MASK | PRD_TXMARGIN_MASK | PRD_TXSWING_MASK |
		CFG_TX_ALIGN_POS_MASK;
	usb3_reg_set(usb3_phy_base, COMPHY_REG_LANE_CFG0_ADDR, PRD_TXDEEMPH0_MASK,
		     mask);

	/*
	 * 2. Set BIT0: enable transmitter in high impedance mode
	 *    Set BIT[3:4]: delay 2 clock cycles for HiZ off latency
	 *    Set BIT6: Tx detect Rx at HiZ mode
	 *    Unset BIT15: set to 0 to set USB3 De-emphasize level to -3.5db
	 *            together with bit 0 of COMPHY_REG_LANE_CFG0_ADDR register
	 */
	mask = PRD_TXDEEMPH1_MASK | TX_DET_RX_MODE | GEN2_TX_DATA_DLY_MASK |
		TX_ELEC_IDLE_MODE_EN;
	data = TX_DET_RX_MODE | GEN2_TX_DATA_DLY_DEFT | TX_ELEC_IDLE_MODE_EN;
	usb3_reg_set(usb3_phy_base, COMPHY_REG_LANE_CFG1_ADDR, data, mask);

	/*
	 * 3. Set Spread Spectrum Clock Enabled
	 */
	usb3_reg_set(usb3_phy_base, COMPHY_REG_LANE_CFG4_ADDR,
		     SPREAD_SPECTRUM_CLK_EN, SPREAD_SPECTRUM_CLK_EN);

	/*
	 * 4. Set Override Margining Controls From the MAC:
	 *    Use margining signals from lane configuration
	 */
	usb3_reg_set(usb3_phy_base, COMPHY_REG_TEST_MODE_CTRL_ADDR,
		     MODE_MARGIN_OVERRIDE, 0xFFFF);

	/*
	 * 5. Set Lane-to-Lane Bundle Clock Sampling Period = per PCLK cycles
	 *    set Mode Clock Source = PCLK is generated from REFCLK
	 */
	usb3_reg_set(usb3_phy_base, COMPHY_REG_GLOB_CLK_SRC_LO_ADDR, 0x0,
		     (MODE_CLK_SRC | BUNDLE_PERIOD_SEL | BUNDLE_PERIOD_SCALE |
		      BUNDLE_SAMPLE_CTRL | PLL_READY_DLY));

	/*
	 * 6. Set G2 Spread Spectrum Clock Amplitude at 4K
	 */
	usb3_reg_set(usb3_phy_base, COMPHY_REG_GEN2_SET_2,
		     G2_TX_SSC_AMP_VALUE_20, G2_TX_SSC_AMP_MASK);

	/*
	 * 7. Unset G3 Spread Spectrum Clock Amplitude
	 *    set G3 TX and RX Register Master Current Select
	 */
	mask = G3_TX_SSC_AMP_MASK | G3_VREG_RXTX_MAS_ISET_MASK |
		RSVD_PH03FH_6_0_MASK;
	usb3_reg_set(usb3_phy_base, COMPHY_REG_GEN2_SET_3,
		     G3_VREG_RXTX_MAS_ISET_60U, mask);

	/*
	 * 8. Check crystal jumper setting and program the Power and PLL Control
	 * accordingly Change RX wait
	 */
	if (lane->priv->xtal_is_40m) {
		ref_clk = REF_CLOCK_SPEED_40M;
		cfg = CFG_PM_RXDLOZ_WAIT_12_UNIT;
	} else {
		ref_clk = USB3_REF_CLOCK_SPEED_25M;
		cfg = CFG_PM_RXDLOZ_WAIT_7_UNIT;
	}

	mask = PU_IVREF_BIT | PU_PLL_BIT | PU_RX_BIT | PU_TX_BIT |
		PU_TX_INTP_BIT | PU_DFE_BIT | PLL_LOCK_BIT | COMPHY_MODE_MASK |
		REF_FREF_SEL_MASK;
	data = PU_IVREF_BIT | PU_PLL_BIT | PU_RX_BIT | PU_TX_BIT |
		PU_TX_INTP_BIT | PU_DFE_BIT | COMPHY_MODE_USB3 | ref_clk;
	usb3_reg_set(usb3_phy_base, COMPHY_POWER_PLL_CTRL, data, mask);

	mask = CFG_PM_OSCCLK_WAIT_MASK | CFG_PM_RXDEN_WAIT_MASK |
		CFG_PM_RXDLOZ_WAIT_MASK;
	data = CFG_PM_RXDEN_WAIT_1_UNIT | cfg;
	usb3_reg_set(usb3_phy_base, COMPHY_REG_PWR_MGM_TIM1_ADDR, data, mask);

	/*
	 * 9. Enable idle sync
	 */
	data = UNIT_CTRL_DEFAULT_VALUE | IDLE_SYNC_EN;
	usb3_reg_set(usb3_phy_base, COMPHY_REG_UNIT_CTRL_ADDR, data, 0xFFFF);

	/*
	 * 10. Enable the output of 500M clock
	 */
	data = MISC_REG0_DEFAULT_VALUE | CLK500M_EN;
	usb3_reg_set(usb3_phy_base, COMPHY_MISC_REG0_ADDR, data, 0xFFFF);

	/*
	 * 11. Set 20-bit data width
	 */
	usb3_reg_set(usb3_phy_base, COMPHY_LOOPBACK_REG0, DATA_WIDTH_20BIT, 0xFFFF);

	/*
	 * 12. Override Speed_PLL value and use MAC PLL
	 */
	usb3_reg_set(usb3_phy_base, COMPHY_KVCO_CAL_CTRL,
		     (SPEED_PLL_VALUE_16 | USE_MAX_PLL_RATE_BIT),
		     0xFFFF);

	/*
	 * 13. Check the Polarity invert bit
	 */
	data = 0U;
	if (lane->invert_tx)
		data |= TXD_INVERT_BIT;
	if (lane->invert_rx)
		data |= RXD_INVERT_BIT;
	mask = TXD_INVERT_BIT | RXD_INVERT_BIT;
	usb3_reg_set(usb3_phy_base, COMPHY_SYNC_PATTERN_REG, data, mask);

	/*
	 * 14. Set max speed generation to USB3.0 5Gbps
	 */
	usb3_reg_set(usb3_phy_base, COMPHY_SYNC_MASK_GEN_REG, PHY_GEN_USB3_5G,
		     PHY_GEN_MAX_MASK);

	/*
	 * 15. Set capacitor value for FFE gain peaking to 0xF
	 */
	usb3_reg_set(usb3_phy_base, COMPHY_REG_GEN3_SETTINGS_3,
		     COMPHY_GEN_FFE_CAP_SEL_VALUE, COMPHY_GEN_FFE_CAP_SEL_MASK);

	/*
	 * 16. Release SW reset
	 */
	data = MODE_CORE_CLK_FREQ_SEL | MODE_PIPE_WIDTH_32 | MODE_REFDIV_BY_4;
	usb3_reg_set(usb3_phy_base, COMPHY_REG_GLOB_PHY_CTRL0_ADDR, data, 0xFFFF);

	/* Wait for > 55 us to allow PCLK be enabled */
	udelay(PLL_SET_DELAY_US);

	if (lane->id == 2) {
		data = USB3PHY_LANE2_REG_BASE_OFFSET + COMPHY_REG_LANE_STATUS1_ADDR;
		writel(data, usb3_phy_base + COMPHY_LANE2_INDIR_ADDR_OFFSET);

		ret = readl_poll_timeout(usb3_phy_base + COMPHY_LANE2_INDIR_DATA_OFFSET,
					 data, data & TXDCLK_PCLK_EN,
					 COMPHY_PLL_SLEEP, COMPHY_PLL_TIMEOUT);
	} else {
		ret = readw_poll_timeout(usb3_phy_base + LANE_STATUS1_ADDR(USB3),
					 data, data & TXDCLK_PCLK_EN,
					 COMPHY_PLL_SLEEP, COMPHY_PLL_TIMEOUT);
	}
	if (ret) {
		dev_err(lane->dev, "Failed to lock USB3 PLL\n");
		return ret;
	}

	return 0;
}

static int mvebu_a3700_comphy_pcie_power_on(struct mvebu_a3700_comphy_lane *lane)
{
	void __iomem *pcie_phy_base = lane->priv->pcie_gbe0_phy_regs;
	u32 mask, data;
	u32 ref_clk;
	int ret;

	/* Configure phy selector for PCIe */
	ret = mvebu_a3700_comphy_set_phy_selector(lane);
	if (ret)
		return ret;

	/* 1. Enable max PLL. */
	reg_set16(pcie_phy_base + LANE_CFG1_ADDR(PCIE),
		  USE_MAX_PLL_RATE_EN, USE_MAX_PLL_RATE_EN);

	/* 2. Select 20 bit SERDES interface. */
	reg_set16(pcie_phy_base + GLOB_CLK_SRC_LO_ADDR(PCIE),
		  CFG_SEL_20B, CFG_SEL_20B);

	/* 3. Force to use reg setting for PCIe mode */
	reg_set16(pcie_phy_base + MISC_REG1_ADDR(PCIE),
		  SEL_BITS_PCIE_FORCE, SEL_BITS_PCIE_FORCE);

	/* 4. Change RX wait */
	reg_set16(pcie_phy_base + PWR_MGM_TIM1_ADDR(PCIE),
		  CFG_PM_RXDEN_WAIT_1_UNIT | CFG_PM_RXDLOZ_WAIT_12_UNIT,
		  (CFG_PM_OSCCLK_WAIT_MASK | CFG_PM_RXDEN_WAIT_MASK |
		   CFG_PM_RXDLOZ_WAIT_MASK));

	/* 5. Enable idle sync */
	reg_set16(pcie_phy_base + UNIT_CTRL_ADDR(PCIE),
		  UNIT_CTRL_DEFAULT_VALUE | IDLE_SYNC_EN, 0xFFFF);

	/* 6. Enable the output of 100M/125M/500M clock */
	reg_set16(pcie_phy_base + MISC_REG0_ADDR(PCIE),
		  MISC_REG0_DEFAULT_VALUE | CLK500M_EN | TXDCLK_2X_SEL | CLK100M_125M_EN,
		  0xFFFF);

	/*
	 * 7. Enable TX, PCIE global register, 0xd0074814, it is done in
	 * PCI-E driver
	 */

	/*
	 * 8. Check crystal jumper setting and program the Power and PLL
	 * Control accordingly
	 */

	if (lane->priv->xtal_is_40m)
		ref_clk = REF_CLOCK_SPEED_40M;
	else
		ref_clk = PCIE_REF_CLOCK_SPEED_25M;

	reg_set16(pcie_phy_base + PWR_PLL_CTRL_ADDR(PCIE),
		  (PU_IVREF_BIT | PU_PLL_BIT | PU_RX_BIT | PU_TX_BIT |
		   PU_TX_INTP_BIT | PU_DFE_BIT | ref_clk | COMPHY_MODE_PCIE),
		  0xFFFF);

	/* 9. Override Speed_PLL value and use MAC PLL */
	reg_set16(pcie_phy_base + KVCO_CAL_CTRL_ADDR(PCIE),
		  SPEED_PLL_VALUE_16 | USE_MAX_PLL_RATE_BIT, 0xFFFF);

	/* 10. Check the Polarity invert bit */
	data = 0U;
	if (lane->invert_tx)
		data |= TXD_INVERT_BIT;
	if (lane->invert_rx)
		data |= RXD_INVERT_BIT;
	mask = TXD_INVERT_BIT | RXD_INVERT_BIT;
	reg_set16(pcie_phy_base + SYNC_PATTERN_REG_ADDR(PCIE), data, mask);

	/* 11. Release SW reset */
	reg_set16(pcie_phy_base + GLOB_PHY_CTRL0_ADDR(PCIE),
		  MODE_CORE_CLK_FREQ_SEL | MODE_PIPE_WIDTH_32,
		  SOFT_RESET | MODE_REFDIV);

	/* Wait for > 55 us to allow PCLK be enabled */
	udelay(PLL_SET_DELAY_US);

	ret = readw_poll_timeout(pcie_phy_base + LANE_STATUS1_ADDR(PCIE),
				 data, data & TXDCLK_PCLK_EN,
				 COMPHY_PLL_SLEEP, COMPHY_PLL_TIMEOUT);
	if (ret) {
		dev_err(lane->dev, "Failed to lock PCIE PLL\n");
		return ret;
	}

	return 0;
}

static void mvebu_a3700_comphy_usb3_power_off(struct mvebu_a3700_comphy_lane *lane)
{
	/*
	 * Currently the USB3 MAC will control the USB3 PHY to set it to low
	 * state, thus do not need to power off USB3 PHY again.
	 */
}

static void mvebu_a3700_comphy_sata_power_off(struct mvebu_a3700_comphy_lane *lane)
{
	void __iomem *sata_phy_base = lane->priv->sata_usb3_phy_regs;
	u32 offset;

	/* Set phy isolation mode */
	offset = SATAPHY_LANE2_REG_BASE_OFFSET + COMPHY_ISOLATION_CTRL_REG;
	comphy_sata_set_indirect(sata_phy_base, offset, PHY_ISOLATE_MODE,
				 PHY_ISOLATE_MODE);

	/* Power off PLL, Tx, Rx */
	offset = SATAPHY_LANE2_REG_BASE_OFFSET + COMPHY_POWER_PLL_CTRL;
	comphy_sata_set_indirect(sata_phy_base, offset, 0,
				 PU_PLL_BIT | PU_RX_BIT | PU_TX_BIT);
}

static void mvebu_a3700_comphy_ethernet_power_off(struct mvebu_a3700_comphy_lane *lane)
{
	void __iomem *comphy_base = lane->priv->comphy_regs;
	u32 mask, data;

	data = PIN_RESET_CORE_BIT | PIN_RESET_COMPHY_BIT | PIN_PU_IVEREF_BIT | PHY_RX_INIT_BIT;
	mask = data;
	reg_set(comphy_base + COMPHY_PHY_CFG1_OFFSET(lane->id), data, mask);
}

static void mvebu_a3700_comphy_pcie_power_off(struct mvebu_a3700_comphy_lane *lane)
{
	/* Currently do nothing */
}

static bool mvebu_a3700_comphy_check_mode(int lane,
					  enum phy_mode mode,
					  int submode)
{
	int i, n = ARRAY_SIZE(mvebu_a3700_comphy_modes);

	/* Unused PHY mux value is 0x0 */
	if (mode == PHY_MODE_INVALID)
		return false;

	for (i = 0; i < n; i++) {
		if (mvebu_a3700_comphy_modes[i].lane == lane &&
		    mvebu_a3700_comphy_modes[i].mode == mode &&
		    mvebu_a3700_comphy_modes[i].submode == submode)
			break;
	}

	if (i == n)
		return false;

	return true;
}

static int mvebu_a3700_comphy_set_mode(struct phy *phy, enum phy_mode mode,
				       int submode)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);

	if (!mvebu_a3700_comphy_check_mode(lane->id, mode, submode)) {
		dev_err(lane->dev, "invalid COMPHY mode\n");
		return -EINVAL;
	}

	/* Just remember the mode, ->power_on() will do the real setup */
	lane->mode = mode;
	lane->submode = submode;

	return 0;
}

static int mvebu_a3700_comphy_power_on(struct phy *phy)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);

	if (!mvebu_a3700_comphy_check_mode(lane->id, lane->mode, lane->submode)) {
		dev_err(lane->dev, "invalid COMPHY mode\n");
		return -EINVAL;
	}

	switch (lane->mode) {
	case PHY_MODE_USB_HOST_SS:
		dev_dbg(lane->dev, "set lane %d to USB3 host mode\n", lane->id);
		return mvebu_a3700_comphy_usb3_power_on(lane);
	case PHY_MODE_SATA:
		dev_dbg(lane->dev, "set lane %d to SATA mode\n", lane->id);
		return mvebu_a3700_comphy_sata_power_on(lane);
	case PHY_MODE_ETHERNET:
		dev_dbg(lane->dev, "set lane %d to Ethernet mode\n", lane->id);
		return mvebu_a3700_comphy_ethernet_power_on(lane);
	case PHY_MODE_PCIE:
		dev_dbg(lane->dev, "set lane %d to PCIe mode\n", lane->id);
		return mvebu_a3700_comphy_pcie_power_on(lane);
	default:
		dev_err(lane->dev, "unsupported PHY mode (%d)\n", lane->mode);
		return -ENOTSUPP;
	}
}

static int mvebu_a3700_comphy_power_off(struct phy *phy)
{
	struct mvebu_a3700_comphy_lane *lane = phy_get_drvdata(phy);

	switch (lane->id) {
	case 0:
		mvebu_a3700_comphy_usb3_power_off(lane);
		mvebu_a3700_comphy_ethernet_power_off(lane);
		return 0;
	case 1:
		mvebu_a3700_comphy_pcie_power_off(lane);
		mvebu_a3700_comphy_ethernet_power_off(lane);
		return 0;
	case 2:
		mvebu_a3700_comphy_usb3_power_off(lane);
		mvebu_a3700_comphy_sata_power_off(lane);
		return 0;
	default:
		dev_err(lane->dev, "invalid COMPHY mode\n");
		return -EINVAL;
	}
}

static const struct phy_ops mvebu_a3700_comphy_ops = {
	.power_on	= mvebu_a3700_comphy_power_on,
	.power_off	= mvebu_a3700_comphy_power_off,
	.set_mode	= mvebu_a3700_comphy_set_mode,
	.owner		= THIS_MODULE,
};

static struct phy *mvebu_a3700_comphy_xlate(struct device *dev,
					    struct of_phandle_args *args)
{
	struct mvebu_a3700_comphy_lane *lane;
	unsigned int port;
	struct phy *phy;

	phy = of_phy_simple_xlate(dev, args);
	if (IS_ERR(phy))
		return phy;

	lane = phy_get_drvdata(phy);

	port = args->args[0];
	if (port != 0 && (port != 1 || lane->id != 0)) {
		dev_err(lane->dev, "invalid port number %u\n", port);
		return ERR_PTR(-EINVAL);
	}

	lane->invert_tx = args->args[1] & BIT(0);
	lane->invert_rx = args->args[1] & BIT(1);

	return phy;
}

static int mvebu_a3700_comphy_probe(struct platform_device *pdev)
{
	struct mvebu_a3700_comphy_priv *priv;
	struct phy_provider *provider;
	struct device_node *child;
	struct resource *res;
	struct clk *clk;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "comphy");
	priv->comphy_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->comphy_regs))
		return PTR_ERR(priv->comphy_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lane1_pcie_gbe");
	priv->pcie_gbe0_phy_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->pcie_gbe0_phy_regs))
		return PTR_ERR(priv->pcie_gbe0_phy_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lane0_usb3_gbe");
	priv->usb3_gbe1_phy_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->usb3_gbe1_phy_regs))
		return PTR_ERR(priv->usb3_gbe1_phy_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lane2_sata_usb3");
	priv->sata_usb3_phy_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->sata_usb3_phy_regs))
		return PTR_ERR(priv->sata_usb3_phy_regs);

	/*
	 * Driver needs to know if reference xtal clock is 40MHz or 25MHz.
	 * Old DT bindings do not have xtal clk present. So do not fail here
	 * and expects that default 25MHz reference clock is used.
	 */
	clk = clk_get(&pdev->dev, "xtal");
	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(&pdev->dev, "missing 'xtal' clk (%ld)\n", PTR_ERR(clk));
	} else {
		ret = clk_prepare_enable(clk);
		if (ret) {
			dev_warn(&pdev->dev, "enabling xtal clk failed (%d)\n", ret);
		} else {
			if (clk_get_rate(clk) == 40000000)
				priv->xtal_is_40m = true;
			clk_disable_unprepare(clk);
		}
		clk_put(clk);
	}

	dev_set_drvdata(&pdev->dev, priv);

	for_each_available_child_of_node(pdev->dev.of_node, child) {
		struct mvebu_a3700_comphy_lane *lane;
		struct phy *phy;
		int ret;
		u32 lane_id;

		ret = of_property_read_u32(child, "reg", &lane_id);
		if (ret < 0) {
			dev_err(&pdev->dev, "missing 'reg' property (%d)\n",
				ret);
			continue;
		}

		if (lane_id >= 3) {
			dev_err(&pdev->dev, "invalid 'reg' property\n");
			continue;
		}

		lane = devm_kzalloc(&pdev->dev, sizeof(*lane), GFP_KERNEL);
		if (!lane) {
			of_node_put(child);
			return -ENOMEM;
		}

		phy = devm_phy_create(&pdev->dev, child,
				      &mvebu_a3700_comphy_ops);
		if (IS_ERR(phy)) {
			of_node_put(child);
			return PTR_ERR(phy);
		}

		lane->priv = priv;
		lane->dev = &pdev->dev;
		lane->mode = PHY_MODE_INVALID;
		lane->submode = PHY_INTERFACE_MODE_NA;
		lane->id = lane_id;
		lane->invert_tx = false;
		lane->invert_rx = false;
		phy_set_drvdata(phy, lane);

		/*
		 * To avoid relying on the bootloader/firmware configuration,
		 * power off all comphys.
		 */
		mvebu_a3700_comphy_power_off(phy);
	}

	provider = devm_of_phy_provider_register(&pdev->dev,
						 mvebu_a3700_comphy_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id mvebu_a3700_comphy_of_match_table[] = {
	{ .compatible = "marvell,comphy-a3700" },
	{ },
};
MODULE_DEVICE_TABLE(of, mvebu_a3700_comphy_of_match_table);

static struct platform_driver mvebu_a3700_comphy_driver = {
	.probe	= mvebu_a3700_comphy_probe,
	.driver	= {
		.name = "mvebu-a3700-comphy",
		.of_match_table = mvebu_a3700_comphy_of_match_table,
	},
};
module_platform_driver(mvebu_a3700_comphy_driver);

MODULE_AUTHOR("Miquèl Raynal <miquel.raynal@bootlin.com>");
MODULE_AUTHOR("Pali Rohár <pali@kernel.org>");
MODULE_DESCRIPTION("Common PHY driver for A3700");
MODULE_LICENSE("GPL v2");
