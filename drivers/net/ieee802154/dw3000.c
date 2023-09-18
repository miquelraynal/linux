// SPDX-License-Identifier: GPL-2.0-only
/*
 * DecaWave DW3000 driver
 *
 * Copyright (C) 2021 Qorvo US, Inc
 * Authors:
 *   - David Girault <david.girault@qorvo.com>
 *   - Romual Despres <romuald.despres@qorvo.com>
 *   - Miquel Raynal <miquel.raynal@bootlin.com>
 */

#define DEBUG

/* TODO: Handle concurrency */

/* TODO: This is currently a hack to switch between E0 and D0,
 * we need to decide whether supporting D0 chips mainline is relevant or not.
 */
#define D0

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/units.h>
#include <net/mac802154.h>

/* Hard reset settle time */
#define DW3000_HARD_RESET_DELAY_US 10000
/* Soft reset settle time */
#define DW3000_SOFT_RESET_DELAY_US 1000
/* Sometimes the device should not be accessed at higher than 3MHz */
#define DW3000_SPI_SLOW_HZ (3 * HZ_PER_MHZ)
/* PGF calibration settle time (with margin) */
#define DW3000_PGFCAL_DELAY_US 1000
/* Number of times PLL locking might be retried */
#define DW3000_MAX_RETRIES_FOR_PLL 50
/* E0 chips PLL calibration delay */
#define DW3000_D0_E0_PLL_CALIBRATION_DELAY_US 400
/* E0 chips loops for noise threshold monitoring */
#define DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS 4
/* E0 chips ADC calibration delay */
#define DW3000_E0_ADC_CALIBRATION_DELAY_US 200
/* E0 chips loops for noise threshold monitoring */
#define DW3000_E0_ADC_THRESHOLD_TRIES 2

/* DW3000 spi interface */
#define DW3000_CMD(cmd) (((cmd) & 0x1F) << 1)
#define   DW3000_CMD_TXRXOFF 0x0
#define   DW3000_CMD_RX 0x2
#define   DW3000_CMD_TX_W4R 0xc
#define   DW3000_CMD_SEMA_RESET 0x18
#define DW3000_16B_ADDR BIT(6)
#define DW3000_WNR(wnr) ((wnr) ? BIT(7) : 0)
#define DW3000_FAST_CMD(cmd) (DW3000_WNR(1) | DW3000_CMD(cmd) | BIT(0))
#define DW3000_REG_OFF_HI(off) (((off) & 0x40) ? BIT(0) : 0)
#define DW3000_REG_OFF_LO(off) (((off) & 0x3F) << 2)
#define DW3000_ADDR_HI(fid, off) ((((fid) & 0x1F) << 1) | DW3000_REG_OFF_HI(off))
#define DW3000_ADDR_LO_OFF(off) DW3000_REG_OFF_LO(off)
#define DW3000_ADDR_LO_DATA 0
#define DW3000_ADDR_LO_ANDOR8 BIT(0)
#define DW3000_ADDR_LO_ANDOR16 BIT(1)
#define DW3000_ADDR_LO_ANDOR32 GENMASK(1, 0)

/* DW3000 register interface: file IDs, offsets and bitfields */
#define DW3000_GENERAL_REG0_FID 0x0
#define   DW3000_DEV_ID_OFF 0x0
#define   DW3000_SYS_CFG_OFF 0x10
/*          PHY Header (PHR) mode */
#define     DW3000_SYS_CFG_PHR_MODE(phrm) FIELD_PREP(BIT(4), (phrm))
#define       DW3000_PHRMODE_STD 0x0 /* Standard PHR mode */
#define       DW3000_PHRMODE_EXT 0x1 /* Decawave extended frames PHR mode */
/*          Bit Rate for the PHR */
#define     DW3000_SYS_CFG_PHR_RATE(phrr) FIELD_PREP(BIT(5), (phrr))
#define       DW3000_PHRRATE_STD 0x0 /* Standard PHR rate, 850 kbits/s */
#define       DW3000_PHRRATE_DTA 0x1 /* PHR at data rate 6.8 Mbits/s */
#define     DW3000_SYS_CFG_SPI_CRCEN BIT(6)
#define     DW3000_SYS_CFG_CP_SPC(sts) FIELD_PREP(GENMASK(13, 12), (sts))
#define       DW3000_STS_MODE_OFF 0x0 /* STS is off */
#define       DW3000_STS_MODE_1 0x1 /* STS mode 1 */
#define       DW3000_STS_MODE_2 0x2 /* STS mode 2 */
#define       DW3000_STS_MODE_ND 0x3 /* STS with no data */
#define     DW3000_SYS_CFG_SDC(sdc) FIELD_PREP(BIT(15), (sdc))
#define     DW3000_SYS_CFG_PDOA_MODE(pdoa) FIELD_PREP(GENMASK(17, 16), (pdoa))
#define       DW3000_PDOA_OFF 0x0 /* PDOA mode is off */
#define       DW3000_PDOA_M1 0x1 /* PDOA mode 1 */
#define       DW3000_PDOA_M2 0x2 /* PDOA mode 2 (not supported) */
#define       DW3000_PDOA_M3 0x3 /* PDOA mode 3 */
#define   DW3000_TX_FCTRL_LOW_OFF 0x20
/*          UWB data bit rate */
#define     DW3000_TX_FCTRL_TXBR(x) FIELD_PREP(BIT(10), (x))
#define       DW3000_TXBR_850K 0
#define       DW3000_TXBR_6M8 1
/*          Tx preamble length (in symbols) */
#define     DW3000_TX_FCTRL_TXPSR(x) FIELD_PREP(GENMASK(15, 12), (x))
#define       DW3000_PLEN_64 0x01
#define       DW3000_PLEN_1024 0x02
#define       DW3000_PLEN_4096 0x03
#define       DW3000_PLEN_32 0x04 /* Non-standard */
#define       DW3000_PLEN_128 0x05 /* Non-standard */
#define       DW3000_PLEN_1536 0x06 /* Non-standard */
#define       DW3000_PLEN_72 0x07 /* Non-standard */
#define       DW3000_PLEN_256 0x09 /* Non-standard */
#define       DW3000_PLEN_2048 0x0a /* Non-standard */
#define       DW3000_PLEN_512 0x0d /* Non-standard */
#define   DW3000_TX_FCTRL_HIGH_OFF 0x24
#define       DW3000_TX_FINE_PLEN(fplen) FIELD_PREP(GENMASK(15, 8), (fplen))
#define   DW3000_SYS_ENABLE_LOW_OFF 0x3c
#define   DW3000_SYS_ENABLE_HIGH_OFF 0x40
#define   DW3000_SYS_STATUS_OFF 0x44
#define     DW3000_SYS_STATUS_CPLOCK BIT(1)
#define     DW3000_SYS_STATUS_SPICRCE BIT(2)
#define     DW3000_SYS_STATUS_TXFRS BIT(7)
#define     DW3000_SYS_STATUS_RXPRD BIT(8)
#define     DW3000_SYS_STATUS_RXSFDD BIT(9)
#define     DW3000_SYS_STATUS_CIA_DONE BIT(10)
#define     DW3000_SYS_STATUS_RXPHD BIT(11)
#define     DW3000_SYS_STATUS_RXPHE BIT(12)
#define     DW3000_SYS_STATUS_RXFR BIT(13)
#define     DW3000_SYS_STATUS_RXFCG BIT(14)
#define     DW3000_SYS_STATUS_RXFCE BIT(15)
#define     DW3000_SYS_STATUS_RXFSL BIT(16)
#define     DW3000_SYS_STATUS_RXFTO BIT(17)
#define     DW3000_SYS_STATUS_CIAERR BIT(18)
#define     DW3000_SYS_STATUS_RXPTO BIT(21)
#define     DW3000_SYS_STATUS_LCSSERR BIT(22)
#define     DW3000_SYS_STATUS_RCINIT BIT(24)
#define     DW3000_SYS_STATUS_RXSTO BIT(26)
#define     DW3000_SYS_STATUS_CPERR BIT(28)
#define     DW3000_SYS_STATUS_ARFE BIT(29)
/*          All Rx events after a correct packet reception mask */
#define     DW3000_SYS_STATUS_ALL_RX_GOOD_MASK				\
	      (DW3000_SYS_STATUS_RXFR |					\
	       DW3000_SYS_STATUS_RXFCG |				\
	       DW3000_SYS_STATUS_RXPRD |				\
	       DW3000_SYS_STATUS_RXSFDD |				\
	       DW3000_SYS_STATUS_RXPHD |				\
	       DW3000_SYS_STATUS_CIA_DONE)
/*          All Rx errors mask */
#define     DW3000_SYS_STATUS_ALL_RX_ERR_MASK				\
	      (DW3000_SYS_STATUS_RXPHE |				\
	       DW3000_SYS_STATUS_RXFCE |				\
	       DW3000_SYS_STATUS_RXFSL |				\
	       DW3000_SYS_STATUS_RXSTO |				\
	       DW3000_SYS_STATUS_ARFE |					\
	       DW3000_SYS_STATUS_CIAERR |				\
	       DW3000_SYS_STATUS_CPERR |				\
	       DW3000_SYS_STATUS_LCSSERR)
/*          All Rx timeouts mask */
#define     DW3000_SYS_STATUS_ALL_RX_TO_MASK				\
	      (DW3000_SYS_STATUS_RXFTO |				\
	       DW3000_SYS_STATUS_RXPTO)
#define     DW3000_SYS_STATUS_ALL GENMASK(31, 0)
#define   DW3000_RX_FINFO_OFF 0x4c
#define     DW3000_RX_FINFO_LEN(finfo) FIELD_GET(GENMASK(9, 0), finfo)
#define   DW3000_TX_ANTD_OFF 0x7c

#define DW3000_GENERAL_REG1_FID 0x01
#define   DW3000_TX_POWER_OFF 0x04
#define   DW3000_CHAN_CTRL_OFF 0x30
#define     DW3000_CHAN_CTRL_CHAN5 0
#define     DW3000_CHAN_CTRL_CHAN9 BIT(0)
/*          Start of Frame Delimiter (SFD) */
#define     DW3000_CHAN_CTRL_SFD_TYPE(type) FIELD_PREP(GENMASK(2, 1), (type))
#define       DW3000_SFD_TYPE_STD 0 /* Standard short IEEE802154 8-symbols SFD */
#define       DW3000_SFD_TYPE_DW_8 1 /* Decawave 8-symbols SFD */
#define       DW3000_SFD_TYPE_DW_16 2 /* Decawave 16-symbols SFD */
#define       DW3000_SFD_TYPE_4Z 3 /* Standard IEEE802154z 8-symbols SFD */
#define     DW3000_CHAN_CTRL_TX_PCODE(code) FIELD_PREP(GENMASK(7, 3), (code))
#define     DW3000_CHAN_CTRL_RX_PCODE(code) FIELD_PREP(GENMASK(12, 8), (code))

#define DW3000_RX_TUNE_FID 0x03
#define   DW3000_MRX_CFG_OFF 0x0
#define     DW3000_MRX_CFG_EN BIT(0)
#define   DW3000_ADC_THRESH_CFG_OFF 0x10
#define   DW3000_AGC_CFG_OFF 0x14
#define     DW3000_AGC_EN BIT(0)
#define     DW3000_PGF_CTRL(gain) FIELD_PREP(GENMASK(5, 3), (gain))
#define     DW3000_PGF_SET_GAIN BIT(6)
#define   DW3000_DGC_CFG_OFF 0x18
#define     DW3000_RX_TUNE_EN BIT(0)
#define     DW3000_DGC_CFG_THR_64(thr) FIELD_PREP(GENMASK(14, 9), (thr))
#define     DW3000_DGC_CFG_THR_64_DEF 0x38
#define   DW3000_DGC_BASE_OFF 0x1C
#if defined E0
#define     DW3000_DGC_SZ 3
#elif defined D0
#define     DW3000_DGC_SZ 2
#endif
#if defined E0
#define   DW3000_LUT_BASE_OFF 0x2C
#elif defined D0
#define   DW3000_LUT_BASE_OFF 0x38
#endif
#define     DW3000_LUT_SZ 7
#define   DW3000_ADC_THRESH_DBG_OFF 0x4c

#define DW3000_EC_CTRL_FID 0x04
#define   DW3000_RX_CAL_OFF 0x0c
#define     DW3000_RX_CAL_MODE(mode) FIELD_PREP(GENMASK(1, 0), (mode))
#define     DW3000_RX_CAL_MODE_EN 1
#define     DW3000_RX_CAL_EN BIT(4)
#define   DW3000_RX_CAL_OFF_2 0x0e
#define     DW3000_RC_CAL_COMP_DLY 0x2
#define   DW3000_RX_CAL_RESI_OFF 0x14
#define   DW3000_RX_CAL_RESQ_OFF 0x1c
#define     DW3000_RX_CAL_RES_INVALID 0x1fffffff
#define   DW3000_RX_CAL_STS_OFF 0x20
#define     DW3000_RX_CAL_STS_DONE BIT(0)

#define DW3000_DRX_CONF_FID 0x06
#define   DW3000_DTUNE0_OFF 0x00
/*          Preamble Acquisition Chunk (PAC) size in symbols */
#define     DW3000_DTUNE0_PAC(pac) FIELD_PREP(GENMASK(1, 0), (pac))
#define       DW3000_PAC8 0 /* Recommended for RX of preamble length  128 and below */
#define       DW3000_PAC16 1 /* Recommended for RX of preamble length 256 */
#define       DW3000_PAC32 2 /* Recommended for RX of preamble length 512 */
#define       DW3000_PAC4 3 /* Recommended for RX of preamble length < 127 */
#define   DW3000_RX_SFDTOC_OFF 0x02
#define   DW3000_DTUNE4_OFF 0x10 /* This one is undocumented */
#define     DW3000_SFD_HLDOFF(sh) FIELD_PREP(GENMASK(31, 24), (sh))

#define DW3000_RF_CONF_FID 0x07
#define   DW3000_RX_TX_CTRL1_OFF 0x1a
#define     DW3000_RX_TX_CTRL1_DEF 0xe
#define   DW3000_RF_SWITCH_OFF 0x14
#define     DW3000_TRXSW_CTRL(ctrl) FIELD_PREP(GENMASK(29, 24), (ctrl))
#define     DW3000_TRXSW_CTRL_RX_CH5 0x1c
#define     DW3000_TRXSW_EN BIT(16)
#define   DW3000_RF_TX_CTRL_HI_OFF 0x1c
#define     DW3000_RF_TX_CTRL_CH5 0x1C071134
#define     DW3000_RF_TX_CTRL_CH9 0x1C010034
#define   DW3000_LDO_CONTROL_OFF 0x48
#define     DW3000_LDO_CONTROL_DIS 0x105
#define   DW3000_LDO_RLOAD_OFF 0x51
#define     DW3000_LDO_RLOAD_DEF 0x14

#define DW3000_FS_CTRL_FID 0x09
#define   DW3000_PLL_CFG_OFF 0x00
#define     DW3000_PLL_CFG_CH5 0x1f3c
#define     DW3000_PLL_CFG_CH9 0x0f3c
#define   DW3000_PLL_CC_OFF 0x04
#define     DW3000_PLL_COARSE_CODE_CH9_RCAS 24 /* Undocummented */
#define     DW3000_PLL_COARSE_CODE_CH9_ICAS 25 /* Undocummented */
#define   DW3000_PLL_CAL_OFF 0x8
#define     DW3000_PLL_CFG_LD(cal) FIELD_PREP(GENMASK(7, 4), (cal))
#define     DW3000_PLL_CFG_LD_DEF 0x8
#if defined E0
#define     DW3000_PLL_CAL_EN BIT(8)
#elif defined D0
#define     DW3000_PLL_CAL_EN BIT(1)
#endif
#define   DW3000_XTAL_OFF 0x14

#define DW3000_AON_FID 0x0a
#define   DW3000_AON_DIG_CFG 0x0
#define   DW3000_AON_CTRL 0x4
#define     DW3000_AON_CTRL_SAVE BIT(1)
#define   DW3000_AON_CFGAON_CFG 0x14

#define DW3000_OTP_FID 0x0b
#define   DW3000_OTP_ADDR_OFF 0x4
#define     DW3000_OTP_ADDR(addr) FIELD_PREP(GENMASK(10, 0), (addr))
#define     DW3000_OTP_ADDR_LDO_TUNE_LO 0x4
#define     DW3000_OTP_ADDR_LDO_TUNE_HI 0x5
#define     DW3000_OTP_ADDR_BIAS_TUNE 0xa
#define       DW3000_OTP_BIAS_TUNE(bias) FIELD_GET(GENMASK(20, 16), (bias))
#define     DW3000_OTP_ADDR_XTAL_TRIM 0x1e
#define       DW3000_XTAL_TRIM(xtal_trim) FIELD_GET(GENMASK(5, 0), (xtal_trim))
#define       DW3000_XTAL_TRIM_DEFAULT GENMASK(4, 0)
#define     DW3000_OTP_ADDR_DGC_TUNE 0x20
#define       DW3000_OTP_DGC_CFG0 0x10000240
#define     DW3000_OTP_ADDR_COARSE_CODE 0x35

#define   DW3000_OTP_CFG_OFF 0x8
#define     DW3000_OTP_CFG_MANUAL BIT(0)
#define     DW3000_OTP_CFG_READ BIT(1)
#define     DW3000_OTP_CFG_DGC_SEL BIT(7)
#define     DW3000_OTP_CFG_DGC_KICK BIT(8)
#define     DW3000_OTP_CFG_LDO_KICK BIT(9)
#define     DW3000_OTP_CFG_BIAS_KICK BIT(10)
#define   DW3000_OTP_RDATA_OFF 0x10

#define DW3000_CIA3_FID 0x0e
#define   DW3000_RX_ANTENNA_DELAY_OFF 0x0

#define DW3000_DIAG_IFACE_FID 0x0f
#define   DW3000_SYS_STATE_OFF 0x30
#define     DW3000_SYS_STATE(sys_state) FIELD_GET(GENMASK(23, 16), (sys_state))
#define     DW3000_SYS_STATE_IDLE 0x3

#define DW3000_PMSC_FID 0x11
#define   DW3000_CLK_CTRL_OFF 0x4
#define     DW3000_SYS_CLK_AUTO 0
#define     DW3000_SYS_CLK_FORCE_FASTRC GENMASK(1, 0)
#define   DW3000_SEQ_CTRL_OFF 0x8
#define     DW3000_SEQ_CTRL_AINIT2IDLE BIT(8)
#define     DW3000_SEQ_CTRL_FORCE2INIT BIT(23)

#define DW3000_RX_BUFFER_A_FID 0x12
#define DW3000_TX_BUFFER_FID 0x14
#define   DW3000_TX_BUFFER_OFF 0x0

enum dw3000_operational_state {
	DW3000_OP_STATE_OFF = 0,
	DW3000_OP_STATE_DEEP_SLEEP,
	DW3000_OP_STATE_SLEEP,
	DW3000_OP_STATE_WAKE_UP,
	DW3000_OP_STATE_INIT_RC,
	DW3000_OP_STATE_IDLE_RC,
	DW3000_OP_STATE_IDLE_PLL,
	DW3000_OP_STATE_TX_WAIT,
	DW3000_OP_STATE_TX,
	DW3000_OP_STATE_RX_WAIT,
	DW3000_OP_STATE_RX,
	DW3000_OP_STATE_MAX,
};

/**
 * struct dw3000_config - Structure holding current device configuration
 * @preamble_length: DW3000_PLEN_64..DW3000_PLEN_4096
 * @sfd_type: SFD type (0 for short IEEE 8b standard, 1 for DW 8b, 2 for DW 16b, 3 for 4z BPRF)
 * @data_rate: Data rate {DW3000_TXBR_850K or DW3000_TXBR_6M8}
 * @phr_mode: PHR mode {0x0 - standard DW3000_PHRMODE_STD, 0x3 - extended frames DW3000_PHRMODE_EXT}
 * @phr_rate: PHR rate {0x0 - standard DW3000_PHRRATE_STD, 0x1 - at datarate DW3000_PHRRATE_DTA}
 * @sfd_timeout: SFD timeout value (in symbols)
 * todo: this and following kdocs
 */
struct dw3000_config {
	u8 preamble_length;
	u8 sfd_type;
	u8 data_rate;
	u8 phr_mode;
	u8 phr_rate;
	u16 sfd_timeout;
	u8 pdoa_mode;
	u8 sts_mode;
	u32 power;
//TODO	bool autoack;
};

struct dw3000_otp {
	u32 ldo_tune_lo;
	u32 ldo_tune_hi;
	u32 bias_tune;
	u32 xtal_trim;
	u32 dgc_addr;
	u32 pll_coarse_code;
};

struct dw3000 {
	struct ieee802154_hw *hw;
	struct spi_device *spi;
	struct device *dev;
	struct sk_buff *tx_skb;
	/* Configuration */
	struct dw3000_config config;
	struct dw3000_otp otp;
	/* Regulators (2v5, 1v8, vdd) */
	struct regulator *regulator_2v5;
	struct regulator *regulator_1v8;
	struct regulator *regulator_vdd;
	/* Reset GPIO */
	struct gpio_desc *reset_gpiod;
	/* Internal operational state of the chip */
	enum dw3000_operational_state operational_state;
	wait_queue_head_t operational_state_wq;
};

/* Preamble length related information. */
struct dw3000_plen_pac_info {
	unsigned int plen; /* Preamble length in symbols */
	unsigned int pac_sz; /* PAC size in symbols */
	u8 plen_reg_val; /* Register value for the preamble length */
	u8 pac_sz_reg_val; /* Register value for the PAC size */
};

static const struct dw3000_plen_pac_info dw3000_plen_pac_lut[] = {
	{ },	/* Reserved */
	{
		.plen = 64,
		.pac_sz = 8,
		.plen_reg_val = DW3000_PLEN_64,
		.pac_sz_reg_val = DW3000_PAC8,
	},
	{
		.plen = 1024,
		.pac_sz = 32,
		.plen_reg_val = DW3000_PLEN_1024,
		.pac_sz_reg_val = DW3000_PAC32,
	},
	{
		.plen = 4096,
		.pac_sz = 64,
		.plen_reg_val = DW3000_PLEN_4096,
		.pac_sz_reg_val = DW3000_PAC32,
	},
	{
		.plen = 32,
		.pac_sz = 8,
		.plen_reg_val = DW3000_PLEN_32,
		.pac_sz_reg_val = DW3000_PAC8,
	},
	{
		.plen = 128,
		.pac_sz = 8,
		.plen_reg_val = DW3000_PLEN_128,
		.pac_sz_reg_val = DW3000_PAC8,
	},
	{
		.plen = 1536,
		.pac_sz = 64,
		.plen_reg_val = DW3000_PLEN_1536,
		.pac_sz_reg_val = DW3000_PAC32,
	},
	{
		.plen = 72,
		.pac_sz = 8,
		.plen_reg_val = DW3000_PLEN_72,
		.pac_sz_reg_val = DW3000_PAC8,
	},
	{ },	/* Reserved */
	{
		.plen = 256,
		.pac_sz = 16,
		.plen_reg_val = DW3000_PLEN_256,
		.pac_sz_reg_val = DW3000_PAC16,
	},
	{
		.plen = 2048,
		.pac_sz = 64,
		.plen_reg_val = DW3000_PLEN_2048,
		.pac_sz_reg_val = DW3000_PAC32,
	},
	{ },	/* Reserved */
	{ },	/* Reserved */
	{
		.plen = 512,
		.pac_sz = 16,
		.plen_reg_val = DW3000_PLEN_512,
		.pac_sz_reg_val = DW3000_PAC16,
	},
};

/**
 * dw3000_spi_fast_cmd() - Send a fast command to the device
 * @dw: the DW device on which the SPI transfer will occurs
 * @cmd: the fast command to send to the device
 *
 * Return: 0 on success, a negative error code otherwise.
 */
static int dw3000_spi_fast_cmd(struct dw3000 *dw, u8 cmd)
{
	u8 hdr = DW3000_FAST_CMD(cmd);
	struct spi_transfer trans = {
		.tx_buf = &hdr,
		.len = 1,
	};
	struct spi_message msg;

	spi_message_init_with_transfers(&msg, &trans, 1);

	return spi_sync(dw->spi, &msg);
}

/**
 * dw3000_spi_msg() - Data exchange with the device
 * @dw: the DW device on which the SPI transfer will occurs
 * @wnr: 1 for a write, 0 for a write
 * @reg_fileid: the register file ID
 * @reg_off: the register offset within the file ID
 * @buf: the buffer to store the data to read/write
 * @length: the buffer length in bytes
 *
 * Return: 0 on success, a negative error code otherwise.
 */
static int dw3000_spi_msg(struct dw3000 *dw, bool wnr, unsigned int reg_fileid,
			  unsigned int reg_off, void *buf, unsigned int length)
{
	u8 hdr[2] = {
		DW3000_WNR(wnr) | DW3000_ADDR_HI(reg_fileid, reg_off),
		DW3000_ADDR_LO_OFF(reg_off) | DW3000_ADDR_LO_DATA
	};
	struct spi_transfer trans[2] = {
		{
			.tx_buf = hdr,
			.len = 1,
		},
		{
			.tx_buf = wnr ? buf : NULL,
			.rx_buf = wnr ? NULL : buf,
			.len = length,
		},
	};
	struct spi_message msg;

	if (reg_off) {
		hdr[0] |= DW3000_16B_ADDR;
		trans[0].len = 2;
	}

	spi_message_init_with_transfers(&msg, trans, 2);

	return spi_sync(dw->spi, &msg);
}

static int dw3000_reg_read8(struct dw3000 *dw, unsigned int reg_fileid,
			    unsigned int reg_off, u8 *buf)
{
	return dw3000_spi_msg(dw, false, reg_fileid, reg_off, buf, 1);
}

static int dw3000_reg_read16(struct dw3000 *dw, unsigned int reg_fileid,
			     unsigned int reg_off, u16 *buf)
{
	__le16 tmp;
	int ret;

	ret = dw3000_spi_msg(dw, false, reg_fileid, reg_off, &tmp, 2);
	if (ret)
		return ret;

	*buf = le16_to_cpu(tmp);

	return 0;
}

static int dw3000_reg_read32(struct dw3000 *dw, unsigned int reg_fileid,
			     unsigned int reg_off, u32 *buf)
{
	__le32 tmp;
	int ret;

	ret = dw3000_spi_msg(dw, false, reg_fileid, reg_off, &tmp, 4);
	if (ret)
		return ret;

	*buf = le32_to_cpu(tmp);

	return 0;
}

static int dw3000_reg_write8(struct dw3000 *dw, unsigned int reg_fileid,
			     unsigned int reg_off, u8 val)
{
	return dw3000_spi_msg(dw, true, reg_fileid, reg_off, &val, 1);
}

static int dw3000_reg_write16(struct dw3000 *dw, unsigned int reg_fileid,
			      unsigned int reg_off, u16 val)
{
	__le16 tmp = cpu_to_le16(val);

	return dw3000_spi_msg(dw, true, reg_fileid, reg_off, &tmp, 2);
}

static int dw3000_reg_write32(struct dw3000 *dw, unsigned int reg_fileid,
			      unsigned int reg_off, u32 val)
{
	__le32 tmp = cpu_to_le32(val);

	return dw3000_spi_msg(dw, true, reg_fileid, reg_off, &tmp, 2);
}

static int dw3000_reg_modify8(struct dw3000 *dw, unsigned int reg_fileid,
			      unsigned int reg_off, u8 and, u8 or)
{
	u8 hdr[2] = {
		DW3000_WNR(1) | DW3000_16B_ADDR | DW3000_ADDR_HI(reg_fileid, reg_off),
		DW3000_ADDR_LO_OFF(reg_off) | DW3000_ADDR_LO_ANDOR32
	};
	u8 tmp[2];
	struct spi_transfer trans[2] = {
		{
			.tx_buf = hdr,
			.len = 2,
		},
		{
			.tx_buf = tmp,
			.len = 2 * sizeof(*tmp),
		},
	};
	struct spi_message msg;

	tmp[0] = and;
	tmp[1] = or;
	spi_message_init_with_transfers(&msg, trans, 2);

	return spi_sync(dw->spi, &msg);
}

static int dw3000_reg_modify16(struct dw3000 *dw, unsigned int reg_fileid,
			       unsigned int reg_off, u16 and, u16 or)
{
	u8 hdr[2] = {
		DW3000_WNR(1) | DW3000_16B_ADDR | DW3000_ADDR_HI(reg_fileid, reg_off),
		DW3000_ADDR_LO_OFF(reg_off) | DW3000_ADDR_LO_ANDOR32
	};
	__le16 tmp[2];
	struct spi_transfer trans[2] = {
		{
			.tx_buf = hdr,
			.len = 2,
		},
		{
			.tx_buf = tmp,
			.len = 2 * sizeof(*tmp),
		},
	};
	struct spi_message msg;

	tmp[0] = cpu_to_le16(and);
	tmp[1] = cpu_to_le16(or);
	spi_message_init_with_transfers(&msg, trans, 2);

	return spi_sync(dw->spi, &msg);
}

static int dw3000_reg_modify32(struct dw3000 *dw, unsigned int reg_fileid,
			       unsigned int reg_off, u32 and, u32 or)
{
	u8 hdr[2] = {
		DW3000_WNR(1) | DW3000_16B_ADDR | DW3000_ADDR_HI(reg_fileid, reg_off),
		DW3000_ADDR_LO_OFF(reg_off) | DW3000_ADDR_LO_ANDOR32
	};
	__le32 tmp[2];
	struct spi_transfer trans[2] = {
		{
			.tx_buf = hdr,
			.len = 2,
		},
		{
			.tx_buf = tmp,
			.len = 2 * sizeof(*tmp),
		},
	};
	struct spi_message msg;

	tmp[0] = cpu_to_le32(and);
	tmp[1] = cpu_to_le32(or);
	spi_message_init_with_transfers(&msg, trans, 2);

	return spi_sync(dw->spi, &msg);
}

#define dw3000_reg_or8(dw, reg_fileid, reg_off, or_val) \
	dw3000_reg_modify8(dw, reg_fileid, reg_off, (u8)-1, or_val)
#define dw3000_reg_and8(dw, reg_fileid, reg_off, and_val) \
	dw3000_reg_modify8(dw, reg_fileid, reg_off, (u8)and_val, 0)

#define dw3000_reg_or16(dw, reg_fileid, reg_off, or_val) \
	dw3000_reg_modify16(dw, reg_fileid, reg_off, (u16)-1, or_val)
#define dw3000_reg_and16(dw, reg_fileid, reg_off, and_val) \
	dw3000_reg_modify16(dw, reg_fileid, reg_off, (u16)and_val, 0)

#define dw3000_reg_or32(dw, reg_fileid, reg_off, or_val) \
	dw3000_reg_modify32(dw, reg_fileid, reg_off, (u32)-1, or_val)
#define dw3000_reg_and32(dw, reg_fileid, reg_off, and_val) \
	dw3000_reg_modify32(dw, reg_fileid, reg_off, (u32)and_val, 0)

static int dw3000_set_interrupt(struct dw3000 *dw, bool setnreset,
				u32 bitmask_low, u32 bitmask_high)
{
	int ret;

	if (bitmask_low) {
		if (setnreset)
			ret = dw3000_reg_or32(dw, DW3000_GENERAL_REG0_FID,
					      DW3000_SYS_ENABLE_LOW_OFF, bitmask_low);
		else
			ret = dw3000_reg_and32(dw, DW3000_GENERAL_REG0_FID,
					       DW3000_SYS_ENABLE_LOW_OFF, ~bitmask_low);
	}

	if (!ret && bitmask_high) {
		if (setnreset)
			ret = dw3000_reg_or32(dw, DW3000_GENERAL_REG0_FID,
					      DW3000_SYS_ENABLE_HIGH_OFF, bitmask_high);
		else
			ret = dw3000_reg_and32(dw, DW3000_GENERAL_REG0_FID,
					       DW3000_SYS_ENABLE_HIGH_OFF, ~bitmask_high);
	}

	return ret;
}

/**
 * dw3000_tx_frame() - prepare, execute or program TX
 * @dw: the DW device
 * @skb: socket buffer
 *
 * Fill the transmit buffer and orders transmission.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_tx_frame(struct dw3000 *dw, struct sk_buff *skb)
{
	int ret;

	/* FCS will be added by the hardware, it must fit in the hw buffer */
	if (WARN_ON_ONCE(skb->len + IEEE802154_FCS_LEN > IEEE802154_MTU))
		return -EINVAL;

	/* Keep a reference over the skb to inform about its completion */
	dw->tx_skb = skb;

	/* Write frame data to the DW IC buffer */
	ret = dw3000_spi_msg(dw, 1, DW3000_TX_BUFFER_FID, DW3000_TX_BUFFER_OFF,
			     skb->data, skb->len);
	if (ret) {
		dev_err(dw->dev, "Cannot write frame data to hw buffer (%d)\n", ret);
		return ret;
	}

//TODO	rc = dw3000_writetxfctrl(dw, len, 0, ranging);
//	if (unlikely(rc))
//		return rc;

//TODO	rc = dw3000_write_txctrl(dw);
//	if (unlikely(rc))
//		return rc;

	/* Order immediate transmission */
	ret = dw3000_spi_fast_cmd(dw, DW3000_CMD_TX_W4R);
	if (ret)
		return ret;

	return 0;
}

static int dw3000_xmit_async(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct dw3000 *dw = hw->priv;

	return dw3000_tx_frame(dw, skb);
}

//static int dw3000_ed(struct ieee802154_hw *hw, u8 *level)
//{
//	return -EOPNOTSUPP;
//}

/**
 * dw3000_enable_rx() - Enable reception on the PHY off
 * @dw: the DW device
 *
 * Return: 0 on success, a negative error code otherwise.
 */
static int dw3000_enable_rx(struct dw3000 *dw)
{
	return dw3000_spi_fast_cmd(dw, DW3000_CMD_RX);
}

/**
 * dw3000_disable_trx() - Force device in idle mode, Tx/Rx off
 * @dw: the DW device
 *
 * According to the DW3000 manual, this command must be send with IRQ disable
 * to avoid a race condition.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_disable_txrx(struct dw3000 *dw)
{
	u32 sys_state;
	int ret;

	/* Check if in TX or RX state before forcing device into IDLE state */
	ret = dw3000_reg_read32(dw, DW3000_DIAG_IFACE_FID,
				DW3000_SYS_STATE_OFF, &sys_state);
	if (ret) {
		dev_err(dw->dev, "Cannot retrieve FSM state (%d)\n", ret);
	} else {
		/* Device is already in IDLE or IDLE_RC */
		if (DW3000_SYS_STATE(sys_state) <= DW3000_SYS_STATE_IDLE)
			return 0;
	}

	ret = dw3000_spi_fast_cmd(dw, DW3000_CMD_TXRXOFF);

	return ret;
}

static int dw3000_configure_chan_ctrl(struct dw3000 *dw,
				      struct ieee802154_channel *chan)
{
	u16 chan_ctrl =
		(chan->channel == 5) ? DW3000_CHAN_CTRL_CHAN5 : DW3000_CHAN_CTRL_CHAN9 |
		DW3000_CHAN_CTRL_SFD_TYPE(dw->config.sfd_type) |
		DW3000_CHAN_CTRL_RX_PCODE(chan->preamble_code) |
		DW3000_CHAN_CTRL_TX_PCODE(chan->preamble_code);

	return dw3000_reg_write16(dw, DW3000_GENERAL_REG1_FID,
				  DW3000_CHAN_CTRL_OFF, chan_ctrl);
}

int dw3000_configure_plen_and_datarate(struct dw3000 *dw)
{
	const struct dw3000_plen_pac_info *lut = &dw3000_plen_pac_lut[dw->config.preamble_length];
	unsigned int sfd_len = (dw->config.sfd_type == DW3000_SFD_TYPE_DW_16) ? 16 : 8;
	u16 sfd_timeout;
	u8 fine_plen;
	u32 sfd_hldoff;
	int ret;

	/* Set Tx preamble size, and data rate */
	ret = dw3000_reg_modify32(dw, DW3000_GENERAL_REG0_FID, DW3000_TX_FCTRL_LOW_OFF,
				  ~(DW3000_TX_FCTRL_TXBR(0x1) | DW3000_TX_FCTRL_TXPSR(0xF)),
				  DW3000_TX_FCTRL_TXBR(dw->config.data_rate) |
				  DW3000_TX_FCTRL_TXPSR(lut->plen_reg_val));
	if (ret)
		return ret;

	/* Set SFD timeout = PLEN + 1 + SFD length - PAC size */
	sfd_timeout = lut->plen + 1 + sfd_len - lut->pac_sz;
	ret = dw3000_reg_write16(dw, DW3000_DRX_CONF_FID, DW3000_RX_SFDTOC_OFF, sfd_timeout);
	if (ret)
		return ret;

	/* Reconfigure PAC */
	ret = dw3000_reg_modify8(dw, DW3000_DRX_CONF_FID, DW3000_DTUNE0_OFF,
				 (u8)~DW3000_DTUNE0_PAC(0x3),
				 DW3000_DTUNE0_PAC(lut->pac_sz_reg_val));
	if (ret)
		return ret;

	/* With 72 symbols of preamble, we need to use fine preamble length of 9.
	 * Otherwise, clear the setting in the FINE_PLEN register to use default
	 * calculation based on TXPSR.
	 */
	fine_plen = (dw->config.preamble_length == DW3000_PLEN_72) ? 9 : 0;
	ret = dw3000_reg_modify16(dw, DW3000_GENERAL_REG0_FID, DW3000_TX_FCTRL_HIGH_OFF,
				  ~DW3000_TX_FINE_PLEN(0xff),
				  DW3000_TX_FINE_PLEN(fine_plen));
	if (ret)
		return ret;

	/* Number of symbols of accumulation to wait before checking for an SFD pattern */
	sfd_hldoff = (lut->plen <= 64) ? 0x14 : 0x20;
	return dw3000_reg_modify32(dw, DW3000_DRX_CONF_FID, DW3000_DTUNE4_OFF,
				   ~DW3000_SFD_HLDOFF(0xff),
				   DW3000_SFD_HLDOFF(sfd_hldoff));
}

int dw3000_configure_sys_cfg(struct dw3000 *dw)
{
	/* Set the PHR mode, PHR rate, STS protocol, SDC and PDOA mode */
	return dw3000_reg_modify32(dw, DW3000_GENERAL_REG0_FID, DW3000_SYS_CFG_OFF,
				   ~(DW3000_SYS_CFG_PHR_MODE(1) |
				     DW3000_SYS_CFG_PHR_RATE(1) |
				     DW3000_SYS_CFG_CP_SPC(1) |
				     DW3000_SYS_CFG_PDOA_MODE(1) |
				     DW3000_SYS_CFG_SDC(1)),
				   DW3000_SYS_CFG_PHR_MODE(dw->config.phr_mode) |
				   DW3000_SYS_CFG_PHR_RATE(dw->config.phr_rate) |
				   DW3000_SYS_CFG_CP_SPC(dw->config.sts_mode) |
				   DW3000_SYS_CFG_PDOA_MODE(dw->config.pdoa_mode));
}

static int dw3000_configure_rf(struct dw3000 *dw)
{
	u32 txctrl, rf_pll_cfg;
	int ret;

	/* Tx Control (factory values) */
	if (dw->hw->phy->current_chan.channel == 5) {
		txctrl = DW3000_RF_TX_CTRL_CH5;
		rf_pll_cfg = DW3000_PLL_CFG_CH5;
	} else {
		txctrl = DW3000_RF_TX_CTRL_CH9;
		rf_pll_cfg = DW3000_PLL_CFG_CH9;
	}

	ret = dw3000_reg_write32(dw, DW3000_RF_CONF_FID, DW3000_RF_TX_CTRL_HI_OFF, txctrl);
	if (ret)
		return ret;

	ret = dw3000_reg_write16(dw, DW3000_FS_CTRL_FID, DW3000_PLL_CFG_OFF, rf_pll_cfg);
	if (ret)
		return ret;

	ret = dw3000_reg_write8(dw, DW3000_RF_CONF_FID, DW3000_LDO_RLOAD_OFF,
				DW3000_LDO_RLOAD_DEF);
	if (ret)
		return ret;

	ret = dw3000_reg_write8(dw, DW3000_RF_CONF_FID, DW3000_RX_TX_CTRL1_OFF,
				DW3000_RX_TX_CTRL1_DEF);
	if (ret)
		return ret;

	/* Setup TX power */
	ret = dw3000_reg_write32(dw, DW3000_GENERAL_REG1_FID, DW3000_TX_POWER_OFF,
				 dw->config.power);
	if (ret)
		return ret;

	/* Extend the lock delay */
	ret = dw3000_reg_modify8(dw, DW3000_FS_CTRL_FID, DW3000_PLL_CAL_OFF,
				 DW3000_PLL_CFG_LD(0xf),
				 DW3000_PLL_CFG_LD(DW3000_PLL_CFG_LD_DEF));
	if (ret)
		return ret;

	/* Configure PLL coarse code */
#if defined E0
	ret = dw3000_reg_and8(dw, DW3000_FS_CTRL_FID, DW3000_PLL_CC_OFF + 3,
			      ~(DW3000_PLL_COARSE_CODE_CH9_RCAS |
				DW3000_PLL_COARSE_CODE_CH9_ICAS) >> 24);
	if (ret)
		return ret;
#elif defined D0
	if (dw->otp.pll_coarse_code) {
		ret = dw3000_reg_write8(dw, DW3000_FS_CTRL_FID, DW3000_PLL_CC_OFF,
					dw->otp.pll_coarse_code);
		if (ret)
			return ret;
	}
#endif

	return 0;
}

static int dw3000_set_default_mrxlut(struct dw3000 *dw)
{
	/* The manual gives magic LUT values which are copy/pasted here */
#if defined E0
	u32 dgc[] = { 0x10000240, 0x1a491248, 0x2db248db };
	u32 ch5_lut[] = { 0x3803e, 0x3876e, 0x397fe, 0x38efe, 0x39c7e, 0x39dfe, 0x39ff6 };
	u32 ch9_lut[] = { 0x5407e, 0x547be, 0x54d36, 0x55e36, 0x55f36, 0x55df6, 0x55ffe };
#elif defined D0
	u32 dgc[] = { 0x10000240, 0x1a491248 };
	u32 ch5_lut[] = { 0x1c0fd, 0x1c43e, 0x1c6be, 0x1c77e, 0x1cf36, 0x1cfb5, 0x1cff5 };
	u32 ch9_lut[] = { 0x2a8fe, 0x2ac36, 0x2a5fe, 0x2af3e, 0x2af7d, 0x2afb5, 0x2afb5 };
#endif
	u32 *lut = (dw->hw->phy->current_chan.channel == 5) ? ch5_lut : ch9_lut;
	int ret, i;

#if defined E0
	for (i = 0; i < DW3000_DGC_SZ; i++) {
		ret = dw3000_reg_write32(dw, DW3000_RX_TUNE_FID,
					 DW3000_DGC_BASE_OFF + (4 * i), dgc[i]);
		if (ret)
			return ret;
	}
#endif

	for (i = 0; i < DW3000_LUT_SZ; i++) {
		ret = dw3000_reg_write32(dw, DW3000_RX_TUNE_FID,
					 DW3000_LUT_BASE_OFF + (sizeof(u32) * i), lut[i]);
		if (ret)
			return ret;
	}

#if defined D0
	for (i = 0; i < DW3000_DGC_SZ; i++) {
		ret = dw3000_reg_write32(dw, DW3000_RX_TUNE_FID,
					 DW3000_DGC_BASE_OFF + (4 * i), dgc[i]);
		if (ret)
			return ret;
	}
#endif

	return 0;
}

int dw3000_configure_dgc(struct dw3000 *dw)
{
	int ret;

	/* If the OTP contains DGC parameters do a manual kick, otherwise
	 * fallback to a default LUT. This only applies to PRFs of 64.
	 */
	if (dw->hw->phy->current_chan.mean_prf == NL802154_MEAN_PRF_62890KHZ) {
		if (dw->otp.dgc_addr == DW3000_OTP_DGC_CFG0) {
			u16 dgc_sel = DW3000_OTP_CFG_DGC_KICK;
			if (dw->hw->phy->current_chan.channel == 9)
				dgc_sel |= DW3000_OTP_CFG_DGC_SEL;

			ret = dw3000_reg_modify16(dw, DW3000_OTP_FID, DW3000_OTP_CFG_OFF,
						  (u16)~DW3000_OTP_CFG_DGC_SEL, dgc_sel);
		} else {
			ret = dw3000_set_default_mrxlut(dw);
		}
		if (ret)
			return ret;

		//TODO: check RX_TUNE_EN bit should be set, that's my understanding but not what is in the dw3000_core.c driver
		ret = dw3000_reg_modify16(dw, DW3000_RX_TUNE_FID, DW3000_DGC_CFG_OFF,
					  ~(DW3000_DGC_CFG_THR_64(0x3f) /*| DW3000_RX_TUNE_EN*/),
					  DW3000_DGC_CFG_THR_64(DW3000_DGC_CFG_THR_64_DEF) /*| DW3000_RX_TUNE_EN */);
	} else {
		ret = dw3000_reg_and8(dw, DW3000_RX_TUNE_FID, DW3000_DGC_CFG_OFF,
				      ~DW3000_RX_TUNE_EN);
	}

	return ret;
}

// TODO: remove debug function
static void __maybe_unused dw3000_print_status_dbg(struct dw3000 *dw)
{
	u32 status_lo, status_hi;
	int ret;

	ret = dw3000_reg_read32(dw, DW3000_GENERAL_REG0_FID,
				DW3000_SYS_STATUS_OFF, &status_lo);
	ret = dw3000_reg_read32(dw, DW3000_GENERAL_REG0_FID,
				DW3000_SYS_STATUS_OFF + 4, &status_hi);
	printk("%s [%d] Status Lo:%08x Hi:%08x\n", __func__, __LINE__, status_lo, status_hi);
}

static int dw3000_calibrate_and_lock_pll(struct dw3000 *dw)
{
	int ret, i;
	u8 status;

	/* Force system clock to auto */
	ret = dw3000_reg_or8(dw, DW3000_PMSC_FID, DW3000_CLK_CTRL_OFF,
			     DW3000_SYS_CLK_FORCE_FASTRC);
	if (ret)
		return ret;

	/* Clear the auto INIT2IDLE bit to switch to IDLE_RC and set FORCE2INIT */
	ret = dw3000_reg_modify32(dw, DW3000_PMSC_FID, DW3000_SEQ_CTRL_OFF,
				  (u32)~DW3000_SEQ_CTRL_AINIT2IDLE,
				  DW3000_SEQ_CTRL_FORCE2INIT);
	if (ret)
		return ret;

	/* Then clear FORCE2INIT (device will stay in IDLE_RC) */
	ret = dw3000_reg_and32(dw, DW3000_PMSC_FID, DW3000_SEQ_CTRL_OFF,
			       (u32)~DW3000_SEQ_CTRL_FORCE2INIT);
	if (ret)
		return ret;

	/* Restore auto clock mode */
//	ret = dw3000_reg_write8(dw, DW3000_PMSC_FID, DW3000_CLK_CTRL_OFF,
//				DW3000_SYS_CLK_AUTO);
//TODO: the original driver does not only set the two first bytes to 0, it also writes bits supposed to be set by default in the register. let's try it:
	ret = dw3000_reg_write16(dw, DW3000_PMSC_FID, DW3000_CLK_CTRL_OFF,
				 (u16)(0x200 | 0x200000UL | 0x100000UL |DW3000_SYS_CLK_AUTO));
	if (ret)
		return ret;

	/* Ensure PLL lock bit is cleared */
	ret = dw3000_reg_write8(dw, DW3000_GENERAL_REG0_FID, DW3000_SYS_STATUS_OFF,
				DW3000_SYS_STATUS_CPLOCK);
	if (ret)
		return ret;

	//TODO: try: also clear the high bits giving idle_rc and init_rc
	ret = dw3000_reg_write32(dw, DW3000_GENERAL_REG0_FID, DW3000_SYS_STATUS_OFF,
				0x01800004);
	if (ret)
		return ret;

	// TODO: this is the force auto clock mode thing done again, see above
	ret = dw3000_reg_write16(dw, DW3000_PMSC_FID, DW3000_CLK_CTRL_OFF,
				 (u16)(0x200 | 0x200000UL | 0x100000UL |DW3000_SYS_CLK_AUTO));
	if (ret)
		return ret;

	/* Run the PLL calibration */
	ret = dw3000_reg_or32(dw, DW3000_FS_CTRL_FID, DW3000_PLL_CAL_OFF,
			      DW3000_PLL_CAL_EN);
	if (ret)
		return ret;

	/* Wait for the PLL calibration (needed before reading the calibration status register) */
	usleep_range(DW3000_D0_E0_PLL_CALIBRATION_DELAY_US * 2,
		     DW3000_D0_E0_PLL_CALIBRATION_DELAY_US * 2+ 50);

	/* Set auto INIT2IDLE back to cause the IC to enable the PLL */
	ret = dw3000_reg_or8(dw, DW3000_PMSC_FID, DW3000_SEQ_CTRL_OFF + 1,
			      DW3000_SEQ_CTRL_AINIT2IDLE >> 8);
	if (ret)
		return ret;

	/* Poll the PLL-locked bit in SYS_STATUS */
	for (i = 0; i < DW3000_MAX_RETRIES_FOR_PLL; i++) {
		usleep_range(10, 40);

		ret = dw3000_reg_read8(dw, DW3000_GENERAL_REG0_FID,
				       DW3000_SYS_STATUS_OFF, &status);
		//TODO: remove dbg
		printk("%s [%d] status %02x\n", __func__, __LINE__, status);
		if (ret)
			return ret;

		/* PLL is locked. */
		if (status & DW3000_SYS_STATUS_CPLOCK)
			return 0;
	}

	/* TODO: Re-sync SYS_TIME and DTU when chip is in IDLE_PLL */

	return -EAGAIN;
}

static int dw3000_calibrate_pgf(struct dw3000 *dw)
{
	u32 resiq;
	u16 ldo;
	u8 cal;
	int ret;

	/* Enable LDO before calibration */
	ret = dw3000_reg_read16(dw, DW3000_RF_CONF_FID, DW3000_LDO_CONTROL_OFF, &ldo);
	if (ret)
		return ret;

	ret = dw3000_reg_or16(dw, DW3000_RF_CONF_FID, DW3000_LDO_CONTROL_OFF,
			      DW3000_LDO_CONTROL_DIS);
	if (ret)
		goto turn_off_ldo;

	/* Turn on delay mode */
	ret = dw3000_reg_write16(dw, DW3000_EC_CTRL_FID, DW3000_RX_CAL_OFF_2,
				 DW3000_RC_CAL_COMP_DLY);
	if (ret)
		goto turn_off_ldo;

	/* Put into calibration mode */
	ret = dw3000_reg_modify8(dw, DW3000_EC_CTRL_FID, DW3000_RX_CAL_OFF,
				 (u8)~DW3000_RX_CAL_MODE(0x3),
				 DW3000_RX_CAL_MODE(DW3000_RX_CAL_MODE_EN));
	if (ret)
		goto turn_off_ldo;

	/* Trigger PGF calibration */
	ret = dw3000_reg_or8(dw, DW3000_EC_CTRL_FID, DW3000_RX_CAL_OFF,
			     DW3000_RX_CAL_EN);
	if (ret)
		goto turn_off_ldo;

	/* Calibration will be done within ~30 us (add some margin) */
	usleep_range(DW3000_PGFCAL_DELAY_US, DW3000_PGFCAL_DELAY_US + 100);

	/* Check if calibration was successful */
	ret = dw3000_reg_read8(dw, DW3000_EC_CTRL_FID, DW3000_RX_CAL_STS_OFF, &cal);
	if (ret)
		goto turn_off_ldo;

	if (cal != DW3000_RX_CAL_STS_DONE)
		goto try_again_cal;

	/* Put into normal mode again */
	ret = dw3000_reg_write8(dw, DW3000_EC_CTRL_FID, DW3000_RX_CAL_OFF, 0);
	if (ret)
		goto turn_off_ldo;

	/* Clear the status */
	ret = dw3000_reg_write8(dw, DW3000_EC_CTRL_FID, DW3000_RX_CAL_STS_OFF,
				DW3000_RX_CAL_STS_DONE);
	if (ret)
		goto turn_off_ldo;

	//TODO: "enable reading" step really strange, skipping

	/* Read PFG I and Q calibration */
	ret = dw3000_reg_read32(dw, DW3000_EC_CTRL_FID, DW3000_RX_CAL_RESI_OFF, &resiq);
	if (ret)
		goto turn_off_ldo;

	if (resiq == DW3000_RX_CAL_RES_INVALID)
		goto try_again_cal;

	ret = dw3000_reg_read32(dw, DW3000_EC_CTRL_FID, DW3000_RX_CAL_RESQ_OFF, &resiq);
	if (ret)
		goto turn_off_ldo;

	if (resiq == DW3000_RX_CAL_RES_INVALID)
		goto try_again_cal;

	/* Restore LDO values */
	return dw3000_reg_and16(dw, DW3000_RF_CONF_FID, DW3000_LDO_CONTROL_OFF, ldo);

try_again_cal:
	ret = -EAGAIN;
turn_off_ldo:
	dw3000_reg_and16(dw, DW3000_RF_CONF_FID, DW3000_LDO_CONTROL_OFF, ldo);
	return ret;
}

static int dw3000_e0_monitor_thresholds(struct dw3000 *dw, u32 *thresholds)
{
	u16 thresholds_avg[4] = {};
	u32 thresholds_val = 0;
	int ret, i;
	u8 status;

	for (i = 0; i < DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS; i++) {
		/* Unfreeze */
		ret = dw3000_reg_modify8(dw, DW3000_RX_TUNE_FID, DW3000_MRX_CFG_OFF,
					 (u8)~DW3000_MRX_CFG_EN, 0);
		if (ret)
			return ret;

		/* Freeze */
		ret = dw3000_reg_modify8(dw, DW3000_RX_TUNE_FID, DW3000_MRX_CFG_OFF,
					 (u8)~DW3000_MRX_CFG_EN, DW3000_MRX_CFG_EN);
		if (ret)
			return ret;

		/* Read the thresholds */
		ret = dw3000_reg_read32(dw, DW3000_RX_TUNE_FID, DW3000_ADC_THRESH_DBG_OFF,
					&thresholds_val);
		if (ret)
			return ret;

		thresholds_avg[0] += FIELD_GET(GENMASK(7, 0), thresholds_val);
		thresholds_avg[1] += FIELD_GET(GENMASK(15, 8), thresholds_val);
		thresholds_avg[2] += FIELD_GET(GENMASK(23, 16), thresholds_val);
		thresholds_avg[3] += FIELD_GET(GENMASK(31, 24), thresholds_val);
	}

	thresholds_avg[0] /= DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS;
	thresholds_avg[1] /= DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS;
	thresholds_avg[2] /= DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS;
	thresholds_avg[3] /= DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS;
	*thresholds = (thresholds_avg[3] << 24) + (thresholds_avg[2] << 16) +
		      (thresholds_avg[1] << 8) + thresholds_avg[0];

	/* Once thresholds are monitored check the system status to cancel the
	 * measure in case of any Rx event.
	 */
	ret = dw3000_reg_read8(dw, DW3000_GENERAL_REG0_FID, DW3000_SYS_STATUS_OFF,
			      &status);
	if (ret)
		return ret;

	if (status & (DW3000_SYS_STATUS_ALL_RX_GOOD_MASK |
		      DW3000_SYS_STATUS_ALL_RX_ERR_MASK |
		      DW3000_SYS_STATUS_ALL_RX_TO_MASK))
		return -EAGAIN;

	return 0;
}

static int dw3000_calibrate_adc_thresholds(struct dw3000 *dw)
{
	u32 switch_control_reg_backup, agc_reg_backup, dgc_reg_backup;
	u32 dgc_lut0_reg_backup, dgc_lut6_reg_backup;
	u32 sys_enable_low, sys_enable_high;
	u32 thresholds;
	int ret, i;
	u8 pgf_idx;

	/* Step 1: Save registers used or modified during the calibration */
	ret = dw3000_reg_read32(dw, DW3000_RF_CONF_FID, DW3000_RF_SWITCH_OFF,
			       &switch_control_reg_backup);
	if (ret)
		return ret;

	ret = dw3000_reg_read32(dw, DW3000_RX_TUNE_FID, DW3000_AGC_CFG_OFF,
				&agc_reg_backup);
	if (ret)
		return ret;

	ret = dw3000_reg_read32(dw, DW3000_RX_TUNE_FID, DW3000_DGC_CFG_OFF,
				&dgc_reg_backup);
	if (ret)
		return ret;

	ret = dw3000_reg_read32(dw, DW3000_RX_TUNE_FID, DW3000_LUT_BASE_OFF,
				&dgc_lut0_reg_backup);
	if (ret)
		return ret;

	ret = dw3000_reg_read32(dw, DW3000_RX_TUNE_FID, DW3000_LUT_BASE_OFF + (sizeof(u32) * 6),
				&dgc_lut6_reg_backup);
	if (ret)
		return ret;

	ret = dw3000_reg_read32(dw, DW3000_GENERAL_REG0_FID, DW3000_SYS_ENABLE_LOW_OFF,
				&sys_enable_low);
	if (ret)
		return ret;

	ret = dw3000_reg_read32(dw, DW3000_GENERAL_REG0_FID, DW3000_SYS_ENABLE_HIGH_OFF,
				&sys_enable_high);
	if (ret)
		return ret;

	/* Step 2a: De-sensitise RX path by shunting the TXRX switch */
	ret = dw3000_reg_modify32(dw, DW3000_RF_CONF_FID, DW3000_RF_SWITCH_OFF,
				  ~(DW3000_TRXSW_CTRL(0x3f) | DW3000_TRXSW_EN),
				  DW3000_TRXSW_CTRL(DW3000_TRXSW_CTRL_RX_CH5) | DW3000_TRXSW_EN);
	if (ret)
		return ret;

	/* Further de-sensitise the RX path by selecting a higher DGC setting */
	ret = dw3000_reg_write32(dw, DW3000_RX_TUNE_FID, DW3000_LUT_BASE_OFF,
				 dgc_lut6_reg_backup);
	if (ret)
		return ret;

	/* Step 2b: Disable AGC and set PGF gain manually */
	pgf_idx = dgc_lut0_reg_backup & 0x7;
	ret = dw3000_reg_modify8(dw, DW3000_RX_TUNE_FID, DW3000_AGC_CFG_OFF,
				 (u8)~(DW3000_PGF_SET_GAIN | DW3000_PGF_CTRL(0x7) | DW3000_AGC_EN),
				 DW3000_PGF_SET_GAIN | DW3000_PGF_CTRL(pgf_idx));
	if (ret)
		return ret;

	/* Step 2c: Reset ADC thresholds */
	ret = dw3000_reg_write8(dw, DW3000_RX_TUNE_FID, DW3000_AGC_CFG_OFF + 3, 0);
	if (ret)
		return ret;

	/* Step 2d: Disable DGC */
	ret = dw3000_reg_and8(dw, DW3000_RX_TUNE_FID, DW3000_DGC_CFG_OFF,
			      ~DW3000_RX_TUNE_EN);
	if (ret)
		return ret;

	/* Step 2e: Disable interrupt events */
	ret = dw3000_set_interrupt(dw, 0, 0xFFFFFFFF, 0xFFFFFFFF);
	if (ret)
		return ret;

	/* Monitor the thresholds */
	for (i = 0; i < DW3000_E0_ADC_THRESHOLD_TRIES; i++) {
		/* Step 3a: enable receiver */
		ret = dw3000_enable_rx(dw);
		if (ret)
			return ret;

		usleep_range(DW3000_E0_ADC_CALIBRATION_DELAY_US,
			     DW3000_E0_ADC_CALIBRATION_DELAY_US + 10);

		/* Step 3b: monitor thresholds */
		ret = dw3000_e0_monitor_thresholds(dw, &thresholds);
		if (ret) {
			if (ret == -EAGAIN)
				continue;

			return ret;
		}

		/* Step 3c: disable receiver */
		ret = dw3000_disable_txrx(dw);
		if (ret)
			return ret;

		break;
	}

	/* 3d: Restore interrupts */
	ret = dw3000_set_interrupt(dw, 1, sys_enable_low, sys_enable_high);
	if (ret)
		return ret;

	/* Step 3e: Set initial DAC indices to settled RMS values */
	ret = dw3000_reg_write32(dw, DW3000_RX_TUNE_FID, DW3000_ADC_THRESH_CFG_OFF,
				 thresholds);
	if (ret)
		return ret;

	/* Step 4: restore initial register values */
	ret = dw3000_reg_write32(dw, DW3000_RF_CONF_FID, DW3000_RF_SWITCH_OFF,
				 switch_control_reg_backup);
	if (ret)
		return ret;

	ret = dw3000_reg_write32(dw, DW3000_RX_TUNE_FID, DW3000_AGC_CFG_OFF,
				 agc_reg_backup);
	if (ret)
		return ret;

	ret = dw3000_reg_write32(dw, DW3000_RX_TUNE_FID, DW3000_DGC_CFG_OFF,
				 dgc_reg_backup);
	if (ret)
		return ret;

	ret = dw3000_reg_write32(dw, DW3000_RX_TUNE_FID, DW3000_LUT_BASE_OFF,
				 dgc_lut0_reg_backup);
	if (ret)
		return ret;

	return 0;
}

static int dw3000_configure_device(struct dw3000 *dw)
{
	int ret;

	ret = dw3000_configure_sys_cfg(dw);
	if (ret)
		return ret;

	/* Setup TX preamble size, data rate and SDF timeout count. There is
	 * currently no support for changing the preamble length so use 64 as a
	 * default for now.
	 */
	ret = dw3000_configure_plen_and_datarate(dw);
	if (ret)
		return ret;

	/* Set TX/RX analogs for given channel */
	ret = dw3000_configure_rf(dw);
	if (ret)
		return ret;

	/* Configure Digital Gain Config */
	ret = dw3000_configure_dgc(dw);
	if (ret)
		return ret;

	/* Auto calibrate the PLL and wait the IDLE_PLL state */
	ret = dw3000_calibrate_and_lock_pll(dw);
	if (ret) {
		if (ret == -EAGAIN)
			dev_err(dw->dev, "PLL calibration error, could be retried\n");
		return ret;
	}

	/* If the RX calibration routine fails the device receiver performance
	 * will be severely affected, the application should reset and try again.
	 */
	ret = dw3000_calibrate_pgf(dw);
	if (ret) {
		if (ret == -EAGAIN)
			dev_err(dw->dev, "Rx calibration error, could be retried\n");
		return ret;
	}

	/* Calibrate ADC offset after DGC configuration and after PLL lock
	 * (otherwise the PLL won't lock) by monitoring the thresholds.
	 */
	ret = dw3000_calibrate_adc_thresholds(dw);
	if (ret) {
		if (ret == -EAGAIN)
			dev_err(dw->dev, "ADC calibration error, could be retried\n");
		return ret;
	}

	return 0;
}

static int dw3000_set_channel(struct ieee802154_hw *hw, struct ieee802154_channel *chan)
{
	struct dw3000 *dw = hw->priv;
	int ret;

	/* Configure the CHAN_CTRL register with the new values */
	ret = dw3000_configure_chan_ctrl(dw, chan);
	if (ret)
		return ret;

	return dw3000_configure_device(dw);
}

static void dw3000_set_operational_state(struct dw3000 *dw,
					 enum dw3000_operational_state st)
{
	dw->operational_state = st;
	wake_up(&dw->operational_state_wq);
}

static int dw3000_wait_idle_state(struct dw3000 *dw)
{
	int timeout = msecs_to_jiffies(500);
	unsigned int max_speed_hz;
	int ret;

	/* The RCINIT interrupt never clears, the ISR will mask it */
	ret = dw3000_set_interrupt(dw, 1, DW3000_SYS_STATUS_RCINIT, 0);
	if (ret)
		return ret;

	/* Use slow spi clock speed right after resets and wake-ups */
	max_speed_hz = dw->spi->max_speed_hz;
	dw->spi->max_speed_hz = min_t(unsigned int, max_speed_hz, DW3000_SPI_SLOW_HZ);

	/* Enable interrupt so we can catch the SPI ready IRQ */
	enable_irq(dw->spi->irq);

	/* Now, wait for SPI ready interrupt */
	if (!wait_event_timeout(dw->operational_state_wq,
				dw->operational_state >= DW3000_OP_STATE_IDLE_RC,
				timeout)) {
		dev_err(dw->dev, "Timeout waiting power-on event\n");
		ret = -ETIMEDOUT;
	}

	/* No IRQs after this point until configuration is fully done */
	disable_irq(dw->spi->irq);

	/* Restore max SPI clock speed */
	dw->spi->max_speed_hz = max_speed_hz;

	return ret;
}

static int dw3000_supply_power(struct dw3000 *dw, bool enable)
{
	int ret;

	if (dw->regulator_1v8) {
		if (enable)
			ret = regulator_enable(dw->regulator_1v8);
		else
			ret = regulator_disable(dw->regulator_1v8);
		if (ret) {
			dev_err(dw->dev, "%s 1.8V regulator failed (%d)\n",
				enable ? "enabling" : "disabling", ret);
			return ret;
		}
	}

	if (dw->regulator_2v5) {
		if (enable)
			ret = regulator_enable(dw->regulator_2v5);
		else
			ret = regulator_disable(dw->regulator_2v5);
		if (ret) {
			dev_err(dw->dev, "%s 2.5V regulator failed (%d)\n",
				enable ? "enabling" : "disabling", ret);
			return ret;
		}
	}

	if (dw->regulator_vdd) {
		if (enable)
			ret = regulator_enable(dw->regulator_vdd);
		else
			ret = regulator_disable(dw->regulator_vdd);
		if (ret) {
			dev_err(dw->dev, "%s Vdd regulator failed (%d)\n",
				enable ? "enabling" : "disabling", ret);
			return ret;
		}
	}

	/* Add experimental delay to wait regulator stability propagation */
	usleep_range(USEC_PER_MSEC, USEC_PER_MSEC + 100);

	return 0;
}


static int dw3000_assert_reset(struct dw3000 *dw, bool reset)
{
	int ret;

	/* De-asserting the reset GPIO implies switching to open drain output or
	 * input. This pin should not be driven high.
	 */
	if (reset)
		ret = gpiod_direction_output(dw->reset_gpiod, reset);
	else
		ret = gpiod_direction_input(dw->reset_gpiod);

	if (ret)
		dev_err(dw->dev, "Could not %s reset gpio (%d)\n",
			reset ? "assert" : "deassert", ret);

	return ret;
}

/**
 * dw3000_get_devid() - Read and check the DEVID register
 * @dw: the DW device on which the SPI transfer will occurs
 *
 * Return: 0 on success, else -ENODEV error code.
 */
static int dw3000_get_devid(struct dw3000 *dw)
{
	u32 devid;
	int ret;
/*	int i;*/

	ret = dw3000_reg_read32(dw, DW3000_GENERAL_REG0_FID,
				DW3000_DEV_ID_OFF, &devid);
	if (ret)
		return ret;

	// TODO: possible action upon device identification, to be discussed
/*	for (i = 0; i < ARRAY_SIZE(dw3000_chip_versions); i++) {
		if (devid == dw3000_chip_versions[i].id) {
			if (!dw->chip_dev_id) {
				dev_info(dw->dev, "chip version found : %x\n",
					 devid);
				dw->chip_dev_id = devid;
				dw->chip_idx = i;
			}
			__dw3000_chip_version = dw3000_chip_versions[i].ver;
			dw->chip_ops = dw3000_chip_versions[i].ops;
			return 0;
		}
	}
	dev_warn(dw->dev, "unknown DEV_ID : %x\n", devid);
	return -ENODEV;
*/

	dev_info(dw->dev, "Found chip: 0x%08x\n", devid);

	return 0;
}

static int dw3000_set_antennas_delay(struct dw3000 *dw, u16 delay)
{
	int ret;

	ret = dw3000_reg_write16(dw, DW3000_CIA3_FID, DW3000_RX_ANTENNA_DELAY_OFF, delay);
	if (ret)
		return ret;

	return dw3000_reg_write16(dw, DW3000_GENERAL_REG0_FID, DW3000_TX_ANTD_OFF, delay);
}

static int dw3000_clear_aonconfig(struct dw3000 *dw)
{
	int ret;

	/* Clear any AON auto download bits (as reset will trigger AON download) */
	ret = dw3000_reg_write16(dw, DW3000_AON_FID, DW3000_AON_DIG_CFG, 0x0);
	if (ret)
		return ret;

	/* Clear the wake-up configuration */
	ret = dw3000_reg_write8(dw, DW3000_AON_FID, DW3000_AON_CFGAON_CFG, 0);
	if (ret)
		return ret;

	/* Upload the new configuration */
	ret = dw3000_reg_write8(dw, DW3000_AON_FID, DW3000_AON_CTRL, 0);
	if (ret)
		return ret;

	return dw3000_reg_write8(dw, DW3000_AON_FID, DW3000_AON_CTRL,
				 DW3000_AON_CTRL_SAVE);
}

static int dw3000_soft_reset(struct dw3000 *dw)
{
	unsigned int max_speed_hz;
	int ret;

	/* Ensure low speed during the reset */
	max_speed_hz = dw->spi->max_speed_hz;
	dw->spi->max_speed_hz = min_t(unsigned int, max_speed_hz, DW3000_SPI_SLOW_HZ);

	ret = dw3000_clear_aonconfig(dw);
	if (ret)
		return ret;

	usleep_range(DW3000_SOFT_RESET_DELAY_US,
		     DW3000_SOFT_RESET_DELAY_US + 100);

	/* Make sure PLL is not the system clock as the PLL will be switched off
	 * as part of the soft reset procedure.
	 */
	ret = dw3000_reg_or8(dw, DW3000_PMSC_FID, DW3000_CLK_CTRL_OFF,
			     DW3000_SYS_CLK_FORCE_FASTRC);
	if (ret)
		return ret;

	/* Send soft reset command */
	ret = dw3000_spi_fast_cmd(dw, DW3000_CMD_SEMA_RESET);
	if (ret)
		return ret;

	// TODO: comment
	/* DW3000 needs a 10us sleep to let clk PLL lock after reset
	 * - the PLL will automatically lock after the reset
	 * Could also have polled the PLL lock flag,
	 * but then the SPI needs to be <= 7MHz !! So a simple delay is easier.
	 */
	usleep_range(DW3000_SOFT_RESET_DELAY_US,
		     DW3000_SOFT_RESET_DELAY_US + 100);

	/* Restore max SPI clock speed */
	dw->spi->max_speed_hz = max_speed_hz;

	return 0;
}

int dw3000_otp_read32(struct dw3000 *dw, u16 addr, u32 *val)
{
	int ret;

	/* Set manual access mode */
	ret = dw3000_reg_write16(dw, DW3000_OTP_FID, DW3000_OTP_CFG_OFF, DW3000_OTP_CFG_MANUAL);
	if (ret)
		return ret;

	/* Provide the address */
	ret = dw3000_reg_write16(dw, DW3000_OTP_FID, DW3000_OTP_ADDR_OFF, DW3000_OTP_ADDR(addr));
	if (ret)
		return ret;

	/* Assert the read strobe */
	ret = dw3000_reg_write16(dw, DW3000_OTP_FID, DW3000_OTP_CFG_OFF, DW3000_OTP_CFG_READ);
	if (ret)
		return ret;

	/* Attempt a read from OTP at the configured address */
	return dw3000_reg_read32(dw, DW3000_OTP_FID, DW3000_OTP_RDATA_OFF, val);
}

int dw3000_read_otp(struct dw3000 *dw)
{
	int ret;

	ret = dw3000_otp_read32(dw, DW3000_OTP_ADDR_LDO_TUNE_LO, &dw->otp.ldo_tune_lo);
	if (ret)
		return ret;

	ret = dw3000_otp_read32(dw, DW3000_OTP_ADDR_LDO_TUNE_HI, &dw->otp.ldo_tune_hi);
	if (ret)
		return ret;

#if defined E0
	ret = dw3000_otp_read32(dw, DW3000_OTP_ADDR_BIAS_TUNE, &dw->otp.bias_tune);
	if (ret)
		return ret;

	dw->otp.bias_tune = DW3000_OTP_BIAS_TUNE(dw->otp.bias_tune);
#endif

	ret = dw3000_otp_read32(dw, DW3000_OTP_ADDR_XTAL_TRIM, &dw->otp.xtal_trim);
	if (ret)
		return ret;

	dw->otp.xtal_trim = DW3000_XTAL_TRIM(dw->otp.xtal_trim);
	if (!dw->otp.xtal_trim)
		dw->otp.xtal_trim = DW3000_XTAL_TRIM_DEFAULT;

	ret = dw3000_otp_read32(dw, DW3000_OTP_ADDR_DGC_TUNE, &dw->otp.dgc_addr);
	if (ret)
		return ret;

	ret = dw3000_otp_read32(dw, DW3000_OTP_ADDR_COARSE_CODE, &dw->otp.pll_coarse_code);
	if (ret)
		return ret;

	return 0;
}

static int dw3000_init(struct dw3000 *dw)
{
	u32 val = 0;
	int ret;

	// TODO: Ensure GPIO block clock is enabled

	ret = dw3000_soft_reset(dw);
	if (ret)
		return ret;

	/* Kick LDO and BIAS tuning based on OTP content */
	if (dw->otp.ldo_tune_lo || dw->otp.ldo_tune_hi)
		val |= DW3000_OTP_CFG_LDO_KICK;
#if defined E0
	if (dw->otp.bias_tune)
		val |= DW3000_OTP_CFG_BIAS_KICK;
#endif
	dw3000_reg_or16(dw, DW3000_OTP_FID, DW3000_OTP_CFG_OFF, val);

	/* Get XTRIM from OTP and write it */
	if (dw->otp.xtal_trim) {
		ret = dw3000_reg_write8(dw, DW3000_FS_CTRL_FID, DW3000_XTAL_OFF,
					dw->otp.xtal_trim);
		if (ret)
			return ret;
	}

	/* Configure PLL coarse code */
#if defined E0
	ret = dw3000_reg_and8(dw, DW3000_FS_CTRL_FID, DW3000_PLL_CC_OFF + 3,
			      ~(DW3000_PLL_COARSE_CODE_CH9_RCAS |
				DW3000_PLL_COARSE_CODE_CH9_ICAS) >> 24);
	if (ret)
		return ret;
#elif defined D0
	if (dw->otp.pll_coarse_code) {
		ret = dw3000_reg_write8(dw, DW3000_FS_CTRL_FID, DW3000_PLL_CC_OFF,
					dw->otp.pll_coarse_code);
		if (ret)
			return ret;
	}
#endif

	// TODO: extract and apply antenna calibration and delay?
	ret = dw3000_set_antennas_delay(dw, 0);
	if (ret) {
		dev_err(dw->dev, "Failed to set antennas delay (%d)\n", ret);
		return ret;
	}

//TODO: autoack seems disabled by default: to enable?
//	/* Set auto-ack delay. */
//	rc = dw3000_set_autoack_reply_delay(
//		dw, DW3000_NUMBER_OF_SYMBOL_DELAY_AUTO_ACK);
//	if (unlikely(rc))
//		return rc;
//	rc = dw3000_disable_autoack(dw, true);
//	if (unlikely(rc))
//		return rc;

//TODO: frame filtering handled in its own hook
//	/* Set the default frame filtering. */
//	rc = dw3000_framefilter_set(
//		dw, DW3000_FF_BEACON_EN | DW3000_FF_DATA_EN | DW3000_FF_ACK_EN |
//			    DW3000_FF_MAC_EN | DW3000_FF_MULTI_EN);
//	if (unlikely(rc))
//		return rc;

	return 0;
}

static int dw3000_start(struct ieee802154_hw *hw)
{
	struct dw3000 *dw = hw->priv;
	int ret;

	ret = dw3000_init(dw);
	if (ret) {
		dev_err(dw->dev, "Failed to initialization chip (%d)\n", ret);
		return ret;
	}

	ret = dw3000_enable_rx(dw);
	if (ret) {
		dev_err(dw->dev, "Failed to enable receiver (%d)\n", ret);
		return ret;
	}

	ret = dw3000_set_interrupt(dw, 1, DW3000_SYS_STATUS_RXFR |
//					  DW3000_SYS_STATUS_RXFCG |
					  DW3000_SYS_STATUS_TXFRS, 0);
	if (ret)
		goto disable_rx;

	enable_irq(dw->spi->irq);

	return 0;

disable_rx:
	dw3000_disable_txrx(dw);

	return ret;
}

static void dw3000_stop(struct ieee802154_hw *hw)
{
	struct dw3000 *dw = hw->priv;

	printk("%s [%d]\n", __func__, __LINE__);
	disable_irq(dw->spi->irq);
	dw3000_set_interrupt(dw, 0, 0xFFFFFFFF, 0xFFFFFFFF);
	dw3000_disable_txrx(dw);
//	dw3000_assert_reset(dw, true);
//	dw3000_supply_power(dw, false);
}

static const struct ieee802154_ops dw3000_ops = {
	.owner			= THIS_MODULE,
	.xmit_async		= dw3000_xmit_async,
//	.ed			= dw3000_ed,
	.set_channel		= dw3000_set_channel,
	.start			= dw3000_start,
	.stop			= dw3000_stop,
//	.set_hw_addr_filt	= dw3000_set_hw_addr_filt,
//	.set_txpower		= dw3000_txpower,
//	.set_lbt		= hulusb_set_lbt,
//	.set_cca_mode		= dw3000_set_cca_mode,
//	.set_cca_ed_level	= dw3000_set_cca_ed_level,
//	.set_csma_params	= dw3000_set_csma_params,
//	.set_frame_retries	= dw3000_set_frame_retries,
//	.set_promiscuous_mode	= dw3000_set_promiscuous_mode,
};

static int dw3000_read_sys_status(struct dw3000 *dw, u32 *sysstat)
{
	return dw3000_reg_read32(dw, DW3000_GENERAL_REG0_FID,
				 DW3000_SYS_STATUS_OFF, sysstat);
}

static int dw3000_clear_sys_status(struct dw3000 *dw, u32 mask)
{
	return dw3000_reg_write32(dw, DW3000_GENERAL_REG0_FID,
				  DW3000_SYS_STATUS_OFF, mask);
}

static int dw3000_isr_rx_event(struct dw3000 *dw)
{
	struct sk_buff *skb;
	u8 *buffer;
	u16 finfo;
	int flen;
	int ret;

	/* Read the first frame information register bytes, they contain the length */
	ret = dw3000_reg_read16(dw, DW3000_GENERAL_REG0_FID,
				DW3000_RX_FINFO_OFF, &finfo);
	if (ret)
		return ret;

	flen = DW3000_RX_FINFO_LEN(finfo);
	if (flen < IEEE802154_FCS_LEN || flen > IEEE802154_MTU) {
		dev_warn(dw->dev, "Empty frame received\n");
		return 0;
	}

	skb = dev_alloc_skb(flen);
	if (!skb)
		return -ENOMEM;

	/* Directly read data from the IC to the buffer */
	buffer = skb_put(skb, flen);
	ret = dw3000_spi_msg(dw, 0, DW3000_RX_BUFFER_A_FID, 0, buffer, flen);
	if (ret) {
		dev_kfree_skb_any(skb);
		return ret;
	}

	ieee802154_rx_irqsafe(dw->hw, skb, 0);

	return 0;
}

static irqreturn_t dw3000_isr(int irq, void *dev_id)
{
	struct dw3000 *dw = dev_id;
	u32 status;
	int ret;

	/* Spurious IRQ may sometimes happen during deep-sleep, do not access the registers */
	if (dw->operational_state == DW3000_OP_STATE_DEEP_SLEEP)
		return IRQ_NONE;

	/* Extract and clear status bits */
	ret = dw3000_read_sys_status(dw, &status);
	if (ret)
		return IRQ_NONE;

	ret = dw3000_clear_sys_status(dw, status);
	if (ret)
		return IRQ_HANDLED;

	/* Handle TX confirmation */
	if (status & DW3000_SYS_STATUS_TXFRS) {
		printk("%s [%d] TX done\n", __func__, __LINE__);
		ieee802154_xmit_complete(dw->hw, dw->tx_skb, false);
		dw->tx_skb = NULL;
	}

	/* Handle RX frame */
	if (status & DW3000_SYS_STATUS_RXFR) {
		printk("%s [%d] RX\n", __func__, __LINE__);
		ret = dw3000_isr_rx_event(dw);
		if (ret)
			dev_err(dw->dev, "Failure retrieving Rx frame (%d)\n", ret);
	}

	/* Device in idle state after wake-up/power-up */
	if (status & DW3000_SYS_STATUS_RCINIT) {
		printk("%s [%d] SPI Rdy and Idle RC\n", __func__, __LINE__);
		dw3000_set_interrupt(dw, 0, DW3000_SYS_STATUS_RCINIT, 0);
		dw3000_set_operational_state(dw, DW3000_OP_STATE_IDLE_RC);
	}

	return IRQ_HANDLED;
}

static int dw3000_get_regulators(struct dw3000 *dw)
{
	int ret;

	dw->regulator_1v8 = devm_regulator_get_optional(dw->dev, "power_reg_1p8");
	if (IS_ERR(dw->regulator_1v8) && PTR_ERR(dw->regulator_1v8) == -ENODEV)
		dw->regulator_1v8 = NULL;
	if (IS_ERR(dw->regulator_1v8)) {
		ret = PTR_ERR(dw->regulator_1v8);
		dev_err(dw->dev, "Failed to get 1.8V regulator (%d)\n", ret);
		return ret;
	}

	dw->regulator_2v5 = devm_regulator_get_optional(dw->dev, "power_reg_2p5");
	if (IS_ERR(dw->regulator_2v5) && PTR_ERR(dw->regulator_2v5) == -ENODEV)
		dw->regulator_2v5 = NULL;
	if (IS_ERR(dw->regulator_2v5)) {
		ret = PTR_ERR(dw->regulator_2v5);
		dev_err(dw->dev, "Failed to get 2.5V regulator (%d)\n", ret);
		return ret;
	}

	dw->regulator_vdd = devm_regulator_get_optional(dw->dev, "power_reg");
	if (IS_ERR(dw->regulator_vdd) && PTR_ERR(dw->regulator_vdd) == -ENODEV)
		dw->regulator_vdd = NULL;
	if (IS_ERR(dw->regulator_vdd)) {
		ret = PTR_ERR(dw->regulator_vdd);
		dev_err(dw->dev, "Failed to get Vdd regulator (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int dw3000_get_reset(struct dw3000 *dw)
{
	dw->reset_gpiod = devm_gpiod_get(dw->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(dw->reset_gpiod))
		return PTR_ERR(dw->reset_gpiod);

	gpiod_set_consumer_name(dw->reset_gpiod, "reset");

	return 0;
}

static int dw3000_get_irq(struct dw3000 *dw)
{
	int ret;

	/* Keep the interrupt masked on the irqchip side */
	ret = devm_request_irq(dw->dev, dw->spi->irq, dw3000_isr,
			       IRQF_NO_AUTOEN, dev_name(dw->dev), dw);
	if (ret) {
		dev_err(dw->dev, "Failed to request IRQ %d (%d)\n",
			dw->spi->irq, ret);
		return ret;
	}

	return 0;
}

static int dw3000_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct dw3000 *dw;
	int ret;

	printk("%s [%d] NEW\n", __func__, __LINE__);
	hw = ieee802154_alloc_hw(sizeof(*dw), &dw3000_ops);
	printk("%s [%d]\n", __func__, __LINE__);
	if (!hw)
		return -ENOMEM;

	dw = hw->priv;
	dw->hw = hw;
	dw->spi = spi;
	dw->dev = &spi->dev;
	init_waitqueue_head(&dw->operational_state_wq);
	dw3000_set_operational_state(dw, DW3000_OP_STATE_OFF);

	spi_set_drvdata(spi, dw);

	/* Initialize default configuration */
	dw->config.preamble_length = DW3000_PLEN_64;
	dw->config.sfd_type = DW3000_SFD_TYPE_4Z;
	dw->config.data_rate = DW3000_TXBR_6M8;
	dw->config.phr_mode = DW3000_PHRMODE_STD;
	dw->config.phr_rate = DW3000_PHRRATE_STD;
	dw->config.sts_mode = DW3000_STS_MODE_OFF;
	dw->config.pdoa_mode = DW3000_PDOA_OFF;
	dw->config.power = 0xfefefefe;

	/* Configure IEEE802154 HW capabilities and default configuration */
	dw->hw->flags =
		(IEEE802154_HW_TX_OMIT_CKSUM /* | TODO: IEEE802154_HW_AFILT |
						  IEEE802154_HW_PROMISCUOUS */);
	dw->hw->phy->supported.channels[4] = 0x0220;
	dw->hw->phy->supported.pcodes[5] = 0x1e18;
	dw->hw->phy->supported.pcodes[9] = 0x1e18;
	dw->hw->phy->supported.prfs = NL802154_MEAN_PRF_16100KHZ |
				      NL802154_MEAN_PRF_62890KHZ;
	dw->hw->phy->current_chan.page = 4;
	dw->hw->phy->current_chan.channel = 5;
	dw->hw->phy->current_chan.preamble_code = 9;
	dw->hw->phy->current_chan.mean_prf = NL802154_MEAN_PRF_62890KHZ;
	//TODO: ensure default pcode/prf config

	//TODO: extract uid from OTP?
	ieee802154_random_extended_addr(&dw->hw->phy->perm_extended_addr);

	ret = dw3000_get_regulators(dw);
	if (ret)
		goto free_hw;

	/* Setup spi and put CS line in high state */
	spi->bits_per_word = 8;
	spi->rt = 1;
	spi->master->last_cs = spi->chip_select;
	ret = spi_setup(spi);
	if (ret)
		goto free_hw;

	ret = dw3000_get_reset(dw);
	if (ret)
		goto free_hw;

	ret = dw3000_get_irq(dw);
	if (ret)
		goto free_hw;

	/* Turn on power */
	ret = dw3000_supply_power(dw, true);
	if (ret) {
		dev_err_probe(dw->dev, ret, "Failed to power device\n");
		goto free_hw;
	}

	/* Assert reset GPIO for enough time */
	ret = dw3000_assert_reset(dw, true);
	if (ret) {
		dev_err(dw->dev, "Failed to assert reset (%d)\n", ret);
		goto power_off;
	}

	usleep_range(DW3000_HARD_RESET_DELAY_US,
		     DW3000_HARD_RESET_DELAY_US + USEC_PER_MSEC);

	ret = dw3000_assert_reset(dw, false);
	if (ret) {
		dev_err(dw->dev, "Failed to de-assert reset (%d)\n", ret);
		goto power_off;
	}

	/* Wait spi ready IRQ */
	ret = dw3000_set_interrupt(dw, 0, 0xFFFFFFFF, 0xFFFFFFFF);
	if (ret)
		goto reset;

	dw3000_set_operational_state(dw, DW3000_OP_STATE_OFF);
	ret = dw3000_wait_idle_state(dw);
	if (ret) {
		dev_err(dw->dev, "Device not ready\n");
		goto reset;
	}

	/* Disable SPI CRC */
	// TODO: does not seem to work
	ret = dw3000_reg_and8(dw, DW3000_GENERAL_REG0_FID, DW3000_SYS_CFG_OFF,
			      ~DW3000_SYS_CFG_SPI_CRCEN);
	if (ret)
		goto reset;

	ret = dw3000_clear_sys_status(dw, DW3000_SYS_STATUS_SPICRCE);
	if (ret)
		goto reset;

	// TODO: Handle D0 and E0 at the same time. Remove the conditional
	if (IS_ENABLED(CONFIG_DEBUG) || IS_ENABLED(CONFIG_DEBUG_FS)) {
		ret = dw3000_get_devid(dw);
		if (ret)
			goto reset;
	}

	ret = dw3000_read_otp(dw);
	if (ret)
		goto reset;

	ret = ieee802154_register_hw(hw);
	if (ret)
		goto reset;

	dw3000_print_status_dbg(dw);

	return 0;

reset:
	dw3000_assert_reset(dw, true);
power_off:
	dw3000_supply_power(dw, false);
free_hw:
	ieee802154_free_hw(hw);

	return ret;
}

static void dw3000_remove(struct spi_device *spi)
{
	struct dw3000 *dw = spi_get_drvdata(spi);

	dw3000_assert_reset(dw, true);
	dw3000_supply_power(dw, false);
	ieee802154_unregister_hw(dw->hw);
	ieee802154_free_hw(dw->hw);
}

static const struct of_device_id dw3000_of_ids[] = {
	{ .compatible = "decawave,dw3000" },
	{},
};
MODULE_DEVICE_TABLE(of, dw3000_of_ids);

static struct spi_driver dw3000_driver = {
	.driver = {
		.name = "dw3000",
		.of_match_table = dw3000_of_ids,
	},
	.probe = dw3000_probe,
	.remove = dw3000_remove,
};
module_spi_driver(dw3000_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Girault <david.girault@qorvo.com>");
MODULE_AUTHOR("Romual Despres <romuald.despres@qorvo.com>");
MODULE_AUTHOR("Miquel Raynal <miquel.raynal@bootlin.com>");
MODULE_DESCRIPTION("DecaWave DW3000 IEEE 802.15.4 driver");
