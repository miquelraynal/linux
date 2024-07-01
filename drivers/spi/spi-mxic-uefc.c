// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2024 Macronix International Co., Ltd.

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand-ecc-mxic.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

/* Host Controller Register */
#define HC_CTRL 0x00
#define HC_CTRL_RQE_EN BIT(31)
#define HC_CTRL_SDMA_BD(x) (((x) & 0x7) << 28)
#define HC_CTRL_PARALLEL_1 BIT(27)
#define HC_CTRL_PARALLEL_0 BIT(26)
#define HC_CTRL_DATA_ORDER BIT(25) //OctaFlash, OctaRAM
#define HC_CTRL_SIO_SHIFTER(x) (((x) & 0x3) << 23)X
#define HC_CTRL_EX_SER_B BIT(22)
#define HC_CTRL_EX_SER_A BIT(21)
#define HC_CTRL_ASSIMI_BYTE_B(x) (((x) & 0x3) << 19)
#define HC_CTRL_ASSIMI_BYTE_A(x) (((x) & 0x3) << 17)
#define HC_CTRL_EX_PHY_ITE_B BIT(16)
#define HC_CTRL_EX_PHY_ITE_A BIT(15)
#define HC_CTRL_EX_PHY_DQS_B BIT(14)
#define HC_CTRL_EX_PHY_DQS_A BIT(13)
#define HC_CTRL_LED BIT(12)
#define HC_CTRL_CH_SEL BIT(11)
#define HC_CTRL_CH_SEL_B BIT(11)
#define HC_CTRL_CH_SEL_A 0
#define HC_CTRL_LUN_SEL(x) (((x) & 0x7) << 8) //NAND
#define HC_CTRL_PORT_SEL(x) (((x) & 0xFF) << 0)
#define HC_CTRL_PORT_SEL_0 0
#define HC_CTRL_PORT_CH_MASK (HC_CTRL_CH_SEL | HC_CTRL_PORT_SEL(0xff))
#define HC_CTRL_PORT_0_CH_B (HC_CTRL_PORT_SEL_0 | HC_CTRL_CH_SEL_B)
#define HC_CTRL_PORT_0_CH_A (HC_CTRL_PORT_SEL_0 | HC_CTRL_CH_SEL_A)

/*
 * INT_STS            0x04, Normal Interrupt Status Register
 * ERR_INT_STS        0x08, Error  Interrupt Status Register
 * INT_STS_EN         0x0C, Normal Interrupt Status Enable Register
 * ERR_INI_STS_EN     0x10, Error  Interrupt Status Enable Register
 * INT_STS_SIG_EN     0x14, Normal Interrupt status Signal Enable Register
 * ERR_INT_STS_SIG_EN 0x18, Error  Interrupt statis Signal Enable Register
 */
/* Normal Interrupt Status Register */
#define INT_STS 0x04
#define INT_STS_CA_REQ BIT(30)
#define INT_STS_CACHE_RDY BIT(29)
#define INT_STS_AC_RDY BIT(28) // For Auto Calibration
#define INT_STS_ERR_INT BIT(15)
#define INT_STS_CQE_INT BIT(14) //Command Queue Engine Interrupt
#define INT_STS_DMA_TFR_CMPLT BIT(7)
#define INT_STS_DMA_INT BIT(6)
#define INT_STS_BUF_RD_RDY BIT(5) //eMMC
#define INT_STS_BUF_WR_RDY BIT(4) //eMMC
#define INT_STS_BLK_GAP BIT(2)   //eMMC
#define INT_STS_DAT_CMPLT BIT(1) //eMMC
#define INT_STS_CMD_CMPLT BIT(0) //eMMC
#define INT_STS_ALL_CLR (INT_STS_AC_RDY | \
                         INT_STS_ERR_INT | \
                         INT_STS_DMA_TFR_CMPLT | \
                         INT_STS_DMA_INT)

/* Error Interrupt Status Register */
#define ERR_INT_STS 0x08
#define ERR_INT_STS_ECC BIT(19)
#define ERR_INT_STS_PREAM BIT(18)
#define ERR_INT_STS_CRC BIT(17)
#define ERR_INT_STS_AC BIT(16)
#define ERR_INT_STS_ADMA BIT(9)
#define ERR_INT_STS_AUTO_CMD BIT(8) //eMMC
#define ERR_INT_STS_DATA_END BIT(6) //eMMC
#define ERR_INT_STS_DATA_CRC BIT(5) //eMMC
#define ERR_INT_STS_DATA_TIMEOUT BIT(4)
#define ERR_INT_STS_CMD_IDX BIT(3) //eMMC
#define ERR_INT_STS_CMD_END BIT(2) //eMMC
#define ERR_INT_STS_CMD_CRC BIT(1) //eMMC
#define ERR_INT_STS_CMD_TIMEOUT BIT(0) //eMMC
#define ERR_INT_STS_ALL_CLR (ERR_INT_STS_ECC | \
                             ERR_INT_STS_PREAM | \
                             ERR_INT_STS_CRC | \
                             ERR_INT_STS_AC | \
                             ERR_INT_STS_ADMA)

/* Normal Interrupt Status Enable Register */
#define INT_STS_EN 0x0C
#define INT_STS_EN_CA_REQ BIT(30)
#define INT_STS_EN_CACHE_RDY BIT(29)
#define INT_STS_EN_AC_RDY BIT(28) //Calibration
#define INT_STS_EN_ERR_INT BIT(15)
#define INT_STS_EN_DMA_TFR_CMPLT BIT(7) //???
#define INT_STS_EN_BUF_RD_RDY BIT(5) //Must be set 1 for eMMC
#define INT_STS_EN_BUF_WR_RDY BIT(4) //Must be set 1 for eMMC
#define INT_STS_EN_DMA_INT BIT(3)
#define INT_STS_EN_BLK_GAP BIT(2)
#define INT_STS_EN_DAT_CMPLT BIT(1)
#define INT_STS_EN_CMD_CMPLT BIT(0)
#define INT_STS_EN_ALL_EN (INT_STS_EN_AC_RDY | \
                           INT_STS_EN_ERR_INT | \
                           INT_STS_EN_DMA_TFR_CMPLT | \
                           INT_STS_EN_DMA_INT)

/* Error Interrupt Status Enable Register */
#define ERR_INT_STS_EN 0x10
#define ERR_INT_STS_EN_ECC BIT(19)
#define ERR_INT_STS_EN_PREAM BIT(18)
#define ERR_INT_STS_EN_CRC BIT(17)
#define ERR_INT_STS_EN_AC BIT(16)
#define ERR_INT_STS_EN_ADMA BIT(9)
#define ERR_INT_STS_EN_AUTO_CMD BIT(8) //eMMC
#define ERR_INT_STS_EN_DATA_END BIT(6) //eMMC
#define ERR_INT_STS_EN_DATA_CRC BIT(5) //eMMC
#define ERR_INT_STS_EN_DATA_TIMEOUT BIT(4)
#define ERR_INT_STS_EN_CMD_IDX BIT(3) //eMMC
#define ERR_INT_STS_EN_CMD_END BIT(2) //eMMC
#define ERR_INT_STS_EN_CMD_CRC BIT(1) //eMMC
#define ERR_INT_STS_EN_CMD_TIMEOUT BIT(0) //eMMC
#define ERR_INT_STS_EN_ALL_EN (ERR_INT_STS_EN_ECC | \
                               ERR_INT_STS_EN_PREAM | \
                               ERR_INT_STS_EN_CRC | \
                               ERR_INT_STS_EN_AC | \
                               ERR_INT_STS_EN_ADMA)

/* Normal Interrupt Signal Enable Register */
#define INT_STS_SIG_EN 0x14
#define INT_STS_SIG_EN_CA_REQ BIT(30)
#define INT_STS_SIG_EN_CACHE_RDY BIT(29)
#define INT_STS_SIG_EN_AC_RDY BIT(28) //Calibration
#define INT_STS_SIG_EN_ERR_INT BIT(15)
#define INT_STS_SIG_EN_DMA_TFR_CMPLT BIT(7) //???
#define INT_STS_SIG_EN_BUF_RD_RDY BIT(5) //eMMC
#define INT_STS_SIG_EN_BUF_WR_RDY BIT(4) //eMMC
#define INT_STS_SIG_EN_DMA_INT BIT(3)
#define INT_STS_SIG_EN_BLK_GAP BIT(2)
#define INT_STS_SIG_EN_DAT_CMPLT BIT(1)
#define INT_STS_SIG_EN_CMD_CMPLT BIT(0)
#define INT_STS_SIG_EN_ALL_EN (INT_STS_SIG_EN_AC_RDY | \
                               INT_STS_SIG_EN_ERR_INT | \
                               INT_STS_SIG_EN_DMA_TFR_CMPLT | \
                               INT_STS_SIG_EN_DMA_INT)

/* Error Interrupt Signal Enable Register */
#define ERR_INT_STS_SIG_EN 0x18
#define ERR_INT_STS_SIG_EN_ECC BIT(19)
#define ERR_INT_STS_SIG_EN_PREAM BIT(18)
#define ERR_INT_STS_SIG_EN_CRC BIT(17)
#define ERR_INT_STS_SIG_EN_AC BIT(16)
#define ERR_INT_STS_SIG_EN_ADMA BIT(9)
#define ERR_INT_STS_SIG_EN_AUTO_CMD BIT(8) //eMMC
#define ERR_INT_STS_SIG_EN_DATA_END BIT(6) //eMMC
#define ERR_INT_STS_SIG_EN_DATA_CRC BIT(5) //eMMC
#define ERR_INT_STS_SIG_EN_DATA_TIMEOUT BIT(4)
#define ERR_INT_STS_SIG_EN_CMD_IDX BIT(3) //eMMC
#define ERR_INT_STS_SIG_EN_CMD_END BIT(2) //eMMC
#define ERR_INT_STS_SIG_EN_CMD_CRC BIT(1) //eMMC
#define ERR_INT_STS_SIG_EN_CMD_TIMEOUT BIT(0) //eMMC
#define ERR_INT_STS_SIG_EN_ALL_EN (ERR_INT_STS_SIG_EN_ECC | \
                                   ERR_INT_STS_SIG_EN_PREAM | \
                                   ERR_INT_STS_SIG_EN_CRC | \
                                   ERR_INT_STS_SIG_EN_AC | \
                                   ERR_INT_STS_SIG_EN_ADMA)

/* Transfer Mode register */
#define TFR_MODE 0x1C
#define TFR_MODE_BUSW_1 0
#define TFR_MODE_BUSW_2 1
#define TFR_MODE_BUSW_4 2
#define TFR_MODE_BUSW_8 3
#define TFR_MODE_DMA_TYPE BIT(31)
#define TFR_MODE_DMA_KEEP_CSB BIT(30)
#define TFR_MODE_TO_ENHC BIT(29)
#define TFR_MODE_PREAM_WITH BIT(28)
#define TFR_MODE_CSB_DONT_CARE BIT(27) //ONFI Nand
    /* share with MAPRD, MAPWR */
    #define OP_DMY_CNT(x) (((x) & 0x3F) << 21)
    #define OP_ADDR_CNT(x) (((x) & 0x7) << 18)
    #define OP_CMD_CNT(x) (((x) - 1) << 17)
    #define OP_DATA_BUSW(x) (((x) & 0x3) << 14)
    #define OP_DATA_DTR(x) (((x) & 0x1) << 16)
    #define OP_ADDR_BUSW(x) (((x) & 0x3) << 11)
    #define OP_ADDR_DTR(x) (((x) & 0x1) << 13)
    #define OP_CMD_BUSW(x) (((x) & 0x3) << 8)
    #define OP_CMD_DTR(x) (((x) & 1) << 10)
    #define OP_DD_RD BIT(4)
#define TFR_MODE_SIO_1X_RD_BUS(x) (((x) & 0x3) << 6)
#define TFR_MODE_MULT_BLK BIT(5)
#define TFR_MODE_AUTO_CMD(x) (((x) & 0x3) << 2)
#define TFR_MODE_CNT_EN BIT(1)
#define TFR_MODE_DMA_EN BIT(0)

/* Transfer Control Register */
#define TFR_CTRL 0x20
#define TFR_CTRL_DEV_DIS BIT(18)
#define TFR_CTRL_IO_END BIT(16)
#define TFR_CTRL_DEV_ACT BIT(2)
#define TFR_CTRL_HC_ACT BIT(1)
#define TFR_CTRL_IO_START BIT(0)

/* Present State Register */
#define PRES_STS 0x24
#define PRES_STS_ADMA(x) (((x) & 0x7) << 29)
#define PRES_STS_XSPI_TX(x) (((x) & 0xF) << 25)
#define PRES_STS_ONFI_TX(x) (((x) & 0x1F) << 20)
#define PRES_STS_RX_NFULL BIT(19)
#define PRES_STS_RX_NEMPT BIT(18)
#define PRES_STS_TX_NFULL BIT(17)
#define PRES_STS_TX_EMPT BIT(16)
#define PRES_STS_EMMC_TX(x) (((x) & 0xF) << 12)
#define PRES_STS_BUF_RD_EN BIT(11)
#define PRES_STS_BUF_WR_EN BIT(10)
#define PRES_STS_RD_TFR BIT(9)
#define PRES_STS_WR_TFR BIT(8)
#define PRES_STS_DAT_ACT BIT(2)
#define PRES_STS_CMD_INH_DAT BIT(1)
#define PRES_STS_CMD_INH_CMD BIT(0)

/* SDMA Transfer Count Register */
#define SDMA_CNT 0x28
#define SDMA_CNT_TFR_BYTE(x) (((x) & 0xFFFFFFFF) << 0)

/* SDMA System Address Register */
#define Sraw_addr 0x2C
#define Sraw_addr_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* ADMA_System Address Register */
#define ADMA2_ADDR 0x30
#define ADMA2_ADDR_VALUE (((x) & 0xFFFFFFFF) << 0)

/* ADMA3 Ubtegrated Descruotir Address Register */
#define ADMA3_ADDR 0x34
#define ADMA3_ADDR_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* Mapping Base Address Register */
#define BASE_MAP_ADDR 0x38
#define BASE_MAP_ADDR_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* Software Reset Register */
#define SW_RST 0x44
#define SW_RST_DAT BIT(2)
#define SW_RST_CMD BIT(1)
#define SW_RST_ALL BIT(0)

/* Timeout Control register */
#define TO_CTRL 0x48
#define TO_CTRL_CA(x) (((x) & 0xF) << 16)
#define TO_CTRL_DAT(x) (((x) & 0xF) << 16)

/* Clock Control Register */
#define CLK_CTRL 0x4C
#define CLK_CTRL_SLOW_CLOCK BIT(31)
#define CLK_CTRL_RX_SS_B(x) (((x) & 0x1F) << 21)
#define CLK_CTRL_RX_SS_A(x) (((x) & 0x1F) << 16)
#define CLK_CTRL_PLL_SELECT(x) (((x) & 0xFFFF) << 0)

/* Cache Control Register */
#define CACHE_CTRL 0x54
#define CACHE_CTRL_DIRTY_LEVEL(x) (((x) & 0x3) << 30)
#define CACHE_CTRL_LEN_TH(x) (((x) & 0xFF) << 22)
#define CACHE_CTRL_CONT_ADDR BIT(21)
#define CACHE_CTRL_FETCH_CNT(x) (((x) & 0x7) << 18)
#define CACHE_CTRL_MST(x) (((x) & 0xFFFF) << 2)
#define CACHE_CTRL_CLEAN BIT(1)
#define CACHE_CTRL_INVALID BIT(0)

/* Capabilities Register */
#define CAP_1 0x58
#define CAP_1_DUAL_CH BIT(31)
#define CAP_1_XSPI_ITF BIT(30)
#define CAP_1_ONFI_ITF BIT(29)
#define CAP_1_EMMC_ITF BIT(28)
#define CAP_1_MAPPING_MODE BIT(27)
#define CAP_1_CACHE BIT(26)
#define CAP_1_ATOMIC BIT(25)
#define CAP_1_DMA_SLAVE_MODE BIT(24)
#define CAP_1_DMA_MASTER_MODE BIT(23)
#define CAP_1_CQE BIT(22)
#define CAP_1_FIFO_DEPTH(x) (((x) & 0x3) << 15)
#define CAP_1_SYS_DW(x) (((x) & 0x3) << 13)
#define CAP_1_LUN_NUM(x) (((x) & 0xF) << 9)
#define CAP_1_CSB_NUM(x) (((x) & 0x1FF) << 0)

/* Host Controller Version Register */
#define HC_VER 0x5C
#define HC_VER_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/*  RTL Version Register */
#define RTL_VER 0x60
#define RTL_VER_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* Transmit Data 0~3 Register */
#define TXD_REG 0x70
#define TXD(x) (TXD_REG + ((x) * 4))

/* Receive Data Register */
#define RXD_REG 0x80
#define RXD_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* Send CRC Cycle Register */
#define SEND_CRC_CYC 0x84
#define SEND_CRC_CYC_EN BIT(0)

/* Block Count Register */
#define BLK_CNT 0x90
#define BLK_CNT_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* Argument Register */
#define ARG_REG 0x94
#define ARG_REG_CMD(x) (((x) & 0xFFFFFFFF) << 0)

/* Command Register */
#define CMD_REG 0x98
#define CMD_REG_BOOT_BUS(x) (((x) & 0x7) << 19)
#define CMD_REG_BOOT_TYPE BIT(18)
#define CMD_REG_BOOT_ACK_EN BIT(17)
#define CMD_REG_BOOT_EN BIT(16)
#define CMD_REG_CMD_IDX(x) (((x) & 3F) << 8)
#define CMD_REG_WR_CRC_STS_EN BIT(6)
#define CMD_REG_DAT_EN BIT(5)
#define CMD_REG_CMD_IDX_CHK_EN BIT(4)
#define CMD_REG_CMD_CRC_CHK_EN BIT(3)
#define CMD_REG_RSP_SEL(x) (((x) & 0x3) << 0)

/* Response 1 Register */
#define RSP_1 0x9C
#define RSP_1_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* Response 2 Register */
#define RSP_2 0xA0
#define RSP_2_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* Response 3 Register */
#define RSP_3 0xA4
#define RSP_3_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* Response 4 Register */
#define RSP_4 0xA8
#define RSP_4_1_VALUE(x) (((x) & 0xFF) << 0)
#define RSP_4_0_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* Buffer Data Port register */
#define DATA_REG 0xAC
#define DATA_REG_BUF(x) (((x) & 0xFFFFFFFF) << 0)

/* Auto CMD Argument Register */
#define AUTO_CMD 0xB0
#define AUTO_CMD_ARGU(x) (((x) & 0xFFFFFFFF) << 0)

/* Auto CMD Error Status Register */
#define AUTO_CMD_ERR_STS 0xB4
#define AUTO_CMD_ERR_STS_IDX BIT(4)
#define AUTO_CMD_ERR_STS_END BIT(3)
#define AUTO_CMD_ERR_STS_CRC BIT(2)
#define AUTO_CMD_ERR_STS_TIMEOUT BIT(1)

/* Boot System Address Register */
#define BOOT_SYS_ADDR 0xB8
#define BOOT_SYS_ADDR_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* Block Gap Control Register */
#define BLK_GAP_CTRL 0xBC
#define BLK_GAP_CTRL_CONT_REQ BIT(1)
#define BLK_GAP_CTRL_STOP_GAP BIT(0)

/* Device Present Status Register */
#define DEV_CTRL 0xC0
#define DEV_CTRL_TYPE(x) (((x) & 0x7) << 29)
#define DEV_CTRL_TYPE_MASK DEV_CTRL_TYPE(0x7)
#define DEV_CTRL_TYPE_SPI (0 << 29) /* spi-nor, qspi-nor, octa-nor, spi-nand */
#define DEV_CTRL_TYPE_LYBRA (1 << 29)
#define DEV_CTRL_TYPE_OCTARAM (2 << 29)
#define DEV_CTRL_TYPE_RAWNAND_ONFI (4 << 29)
#define DEV_CTRL_TYPE_RAWNAND_JEDEC (5 << 29)
#define DEV_CTRL_TYPE_RAWNAND_EMMC (6 << 29)
#define DEV_CTRL_SCLK_SEL(x) (((x) & 0xF) << 25)
#define DEV_CTRL_SCLK_SEL_MASK DEV_CTRL_SCLK_SEL(0xF)
#define DEV_CTRL_SCLK_SEL_DIV(x) (((x >> 1) - 1) << 25)
#define DEV_CTRL_CACHEABLE BIT(24)
#define DEV_CTRL_WR_PLCY(x) (((x) & 0x3) << 22)
#define DEV_CTRL_PAGE_SIZE(x) (((x) & 0x7) << 19)
#define DEV_CTRL_BLK_SIZE(x) (((x) & 0xFFF) << 7)
#define DEV_CTRL_PRE_DQS_EN BIT(6)
#define DEV_CTRL_DQS_EN BIT(5)
#define DEV_CTRL_CRC_EN BIT(4)
#define DEV_CTRL_CRCB_IN_EN BIT(3)
#define DEV_CTRL_CRC_CHUNK_SIZE(x) (((x) & 0x3) << 1)
#define DEV_CTRL_CRCB_OUT_EN BIT(0)

/* Mapping Read Control Register */
#define MAP_RD_CTRL 0xC4
#define MAP_RD_CTRL_PREAM_EN BIT(28)
#define MAP_RD_CTRL_SIO_1X_RD(x) (((x) & 0x3) << 6)

/* Linear/Mapping Write Control Register */
#define MAP_WR_CTRL 0xC8

/* Mapping Command Register */
#define MAP_CMD_RD 0xCC
#define MAP_CMD_WR 0xCE

/* Top Mapping Address Register */
#define TOP_MAP_ADDR 0xD0
#define TOP_MAP_ADDR_VALUE(x) (((x) & 0xFFFFFFFF) << 0)

/* General Purpose Inputs and Outputs Register */
#define GPIO_REG 0xD4
#define GPIO_REG_DATA_LEVEL(x) (((x) & 0xFF) << 24)
#define GPIO_REG_RYBYB_LEVEL BIT(23) /* Get RYBY# Level*/
#define GPIO_REG_CMD_LEVEL BIT(22) /* Get CMD Level*/
#define GPIO_REG_SIO3_EN BIT(13)
#define GPIO_REG_SIO2_EN BIT(12)
#define GPIO_REG_SIO3_DRIV_HIGH BIT(5)
#define GPIO_REG_SIO2_DRIV_HIGH BIT(4)
#define GPIO_REG_HP_DRIV_HIGH BIT(3) /* Drive Hardware Protect Level, RAW NAND */
#define GPIO_REG_RESTB_DRIV_HIGH BIT(2) /* Drive RESET# Level*/
#define GPIO_REG_HOLDB_DRIV_HIGH BIT(1) /* Drive HOLD# Level*/
#define GPIO_REG_WPB_DRIV_HIGH BIT(0) /* Drive WRITE PROTECT# Level*/

/* Auto Chlibration Control Register */
#define AC_CTRL 0xD8
#define AC_CTRL_CMD_2(x) (((x) & 0xFF) << 24)
#define AC_CTRL_CMD_1(x) (((x) & 0xFF) << 16)
#define AC_CTRL_WINDOW(x) (((x) & 0x3) << 14)
#define AC_CTRL_LAZY_DQS_EN BIT(9)
#define AC_CTRL_LEN_32B_SEL BIT(8)
#define AC_CTRL_PHY_EN BIT(6)
#define AC_CTRL_DQS_TEST_EN BIT(5)
#define AC_CTRL_SIO_ALIG_EN BIT(4)
#define AC_CTRL_NVDDR_EN BIT(3) /* Calibration for ONFI v5.0 DDR */
#define AC_CTRL_SAMPLE_DQS_EN BIT(2) /* Calibration for Sampling with DQS */
#define AC_CTRL_SAMPLE_EN BIT(1) /* Calibration for Sampling w/o DQS */
#define AC_CTRL_START BIT(0)

/* Preamble Bit 1 Register */
#define PREAM_1_REG 0xDC
#define PREAM_1_REG_SIO_1(x) (((x) & 0xFFFF) << 16)
#define PREAM_1_REG_SIO_0(x) (((x) & 0xFFFF) << 0)

/* Preamble Bit 2 Register */
#define PREAM_2_REG 0xE0
#define PREAM_2_REG_SIO_3(x) (((x) & 0xFFFF) << 16)
#define PREAM_2_REG_SIO_2(x) (((x) & 0xFFFF) << 0)

/* Preamble Bit 3 Register */
#define PREAM_3_REG 0xE4
#define PREAM_3_REG_SIO_5(x) (((x) & 0xFFFF) << 16)
#define PREAM_3_REG_SIO_4(x) (((x) & 0xFFFF) << 0)

/* Preamble Bit 4 Register */
#define PREAM_4_REG 0xE8
#define PREAM_4_REG_SIO_7(x) (((x) & 0xFFFF) << 16)
#define PREAM_4_REG_SIO_6(x) (((x) & 0xFFFF) << 0)

/* Sample Point Adjust Register */
#define SAMPLE_ADJ 0xEC
#define SAMPLE_ADJ_DQS_IDLY_DOPI(x) (((x) & 0xFF) << 24)
#define SAMPLE_ADJ_DQS_IDLY_SOPI(x) (((x) & 0xFF) << 16)
#define SAMPLE_ADJ_DQS_ODLY(x) (((x) & 0xFF) << 8)
#define SAMPLE_ADJ_POINT_SEL_DDR(x) (((x) & 0x7) << 3)
#define SAMPLE_ADJ_POINT_SEL_SDR(x) (((x) & 0x7) << 0)

/* SIO Input Delay 1 Register */
#define SIO_IDLY_1 0xF0
#define SIO_IDLY_1_SIO3(x) (((x) & 0xFF) << 24)
#define SIO_IDLY_1_SIO2(x) (((x) & 0xFF) << 16)
#define SIO_IDLY_1_SIO1(x) (((x) & 0xFF) << 8)
#define SIO_IDLY_1_SIO0(x) (((x) & 0xFF) << 0)

/* SIO Input Delay 2 Register */
#define SIO_IDLY_2 0xF4
#define SIO_IDLY_2_SIO4(x) (((x) & 0xFF) << 24)
#define SIO_IDLY_2_SIO5(x) (((x) & 0xFF) << 16)
#define SIO_IDLY_2_SIO6(x) (((x) & 0xFF) << 8)
#define SIO_IDLY_2_SIO7(x) (((x) & 0xFF) << 0)
#define IDLY_CODE_VAL(x, v)    ((v) << (((x) % 4) * 8))

/* SIO Output Delay 1 Register */
#define SIO_ODLY_1 0xF8
#define SIO_ODLY_1_SIO3(x) (((x) & 0xFF) << 24)
#define SIO_ODLY_1_SIO2(x) (((x) & 0xFF) << 16)
#define SIO_ODLY_1_SIO1(x) (((x) & 0xFF) << 8)
#define SIO_ODLY_1_SIO0(x) (((x) & 0xFF) << 0)

/* SIO Output Delay 2 Register */
#define SIO_ODLY_2 0xFC
#define SIO_ODLY_2_SIO4(x) (((x) & 0xFF) << 24)
#define SIO_ODLY_2_SIO5(x) (((x) & 0xFF) << 16)
#define SIO_ODLY_2_SIO6(x) (((x) & 0xFF) << 8)
#define SIO_ODLY_2_SIO7(x) (((x) & 0xFF) << 0)

#define UPDATE_WRITE(reg, mask, value) \
    do {\
        writel((value | (readl(mxic->regs + (reg)) & ~(mask))), mxic->regs + (reg));\
    } while(0)

struct mxic_uefc_spi {
    struct device *dev;
    struct clk *ps_clk;
    struct clk *send_clk;
    struct clk *send_dly_clk;
    void __iomem *regs;
    u32 cur_speed_hz;
    struct {
        void __iomem *addr;
        dma_addr_t raw_addr;
        size_t size;
    } dirmap;

    struct {
        bool use_pipelined_conf;
        struct nand_ecc_engine *pipelined_engine;
        void *ctx;
    } ecc;
};

static int mxic_uefc_spi_clk_enable(struct mxic_uefc_spi *mxic)
{
    int ret;

    ret = clk_prepare_enable(mxic->send_clk);
    if (ret)
        return ret;

    ret = clk_prepare_enable(mxic->send_dly_clk);
    if (ret)
        goto err_send_dly_clk;

    return ret;

err_send_dly_clk:
    clk_disable_unprepare(mxic->send_clk);

    return ret;
}

static void mxic_uefc_spi_clk_disable(struct mxic_uefc_spi *mxic)
{
    clk_disable_unprepare(mxic->send_clk);
    clk_disable_unprepare(mxic->send_dly_clk);
}

static void mxic_uefc_device_type(struct mxic_uefc_spi *mxic, uint8_t type, uint32_t port_ch)
{
    UPDATE_WRITE(HC_CTRL, HC_CTRL_PORT_CH_MASK, port_ch);
    UPDATE_WRITE(DEV_CTRL, DEV_CTRL_TYPE_MASK | DEV_CTRL_SCLK_SEL_MASK, type | DEV_CTRL_SCLK_SEL_DIV(4));
}

static void mxic_uefc_hw_init(struct mxic_uefc_spi *mxic, uint8_t type)
{
    writel(CLK_CTRL_RX_SS_A(1) | CLK_CTRL_RX_SS_B(1), mxic->regs + CLK_CTRL);

    writel(INT_STS_ALL_CLR, mxic->regs + INT_STS);
    writel(INT_STS_EN_ALL_EN, mxic->regs + INT_STS_EN);
    writel(INT_STS_SIG_EN_ALL_EN, mxic->regs + INT_STS_SIG_EN);

    writel(ERR_INT_STS_ALL_CLR, mxic->regs + ERR_INT_STS);
    writel(ERR_INT_STS_EN_ALL_EN, mxic->regs + ERR_INT_STS_EN);
    writel(ERR_INT_STS_SIG_EN_ALL_EN, mxic->regs + ERR_INT_STS_SIG_EN);

    writel(SAMPLE_ADJ_POINT_SEL_DDR(1) | SAMPLE_ADJ_POINT_SEL_SDR(1), mxic->regs + SAMPLE_ADJ);
    writel(0, mxic->regs + SIO_IDLY_1);
    writel(0, mxic->regs + SIO_IDLY_2);
    writel(0, mxic->regs + SIO_ODLY_1);
    writel(0, mxic->regs + SIO_ODLY_2);

    /* for spinand and spinor, selct port 0 and channel A */
    mxic_uefc_device_type(mxic, type, HC_CTRL_PORT_0_CH_A);
}

static u32 mxic_uefc_spi_mem_prep_op_cfg(struct mxic_uefc_spi *mxic,
            const struct spi_mem_op *op, unsigned int data_len)
{
    u32 cfg = OP_CMD_CNT(op->cmd.nbytes) |
          OP_CMD_BUSW(fls(op->cmd.buswidth) - 1) |
          OP_CMD_DTR(op->cmd.dtr);

    if (op->addr.nbytes)
        cfg |= OP_ADDR_CNT(op->addr.nbytes) |
        OP_ADDR_BUSW(fls(op->addr.buswidth) - 1) |
        OP_ADDR_DTR(op->addr.dtr);

    if (op->dummy.nbytes) {
        u8 dummy_nbytes = op->dummy.nbytes;
        if (op->dummy.buswidth < op->data.buswidth)
            dummy_nbytes *= (op->data.buswidth / op->dummy.buswidth);
        else if (op->dummy.buswidth > op->data.buswidth)
            dummy_nbytes /= (op->dummy.buswidth / op->data.buswidth);

        cfg |= OP_DMY_CNT(dummy_nbytes);
    }

    /* Direct mapping data.nbytes field is not populated */
    if (data_len) {
        cfg |= OP_DATA_BUSW(fls(op->data.buswidth) - 1) |
            OP_DATA_DTR(op->data.dtr);
        if (op->data.dir == SPI_MEM_DATA_IN) {
            cfg |= OP_DD_RD;
            if (op->data.dtr)
                UPDATE_WRITE(DEV_CTRL, DEV_CTRL_DQS_EN, DEV_CTRL_DQS_EN);
            else
                UPDATE_WRITE(DEV_CTRL, DEV_CTRL_DQS_EN, 0);
        }
    }

    return cfg;
}

static int mxic_uefc_spi_io_mode_xfer(struct mxic_uefc_spi *mxic, const void *tx, void *rx, 
            unsigned int len)
{
    u32 pos = 0;

    while (pos < len) {
        u32 nbytes = len - pos;
        u32 data = 0xffffffff;
        u32 sts = 0;
        int ret = 0;

        if (nbytes > 4)
            nbytes = 4;

        if (tx)
            memcpy(&data, tx + pos, nbytes);
        ret = readl_poll_timeout(mxic->regs + PRES_STS, sts,
                     sts & PRES_STS_TX_NFULL, 0, USEC_PER_SEC);
        if (ret)
            return ret;
        writel(data, mxic->regs + TXD(nbytes % 4));

        ret = readl_poll_timeout(mxic->regs + PRES_STS, sts,
                     sts & PRES_STS_RX_NEMPT, 0, USEC_PER_SEC);
        if (ret)
            return ret;
        
        data = readl(mxic->regs + RXD_REG);
        if (rx)
            memcpy(rx + pos, &data, nbytes);
        
        pos += nbytes;
    }

    return 0;
}

static ssize_t mxic_uefc_spi_mem_dirmap_read(struct spi_mem_dirmap_desc *desc,
                    u64 offs, size_t len, void *buf)
{
    struct mxic_uefc_spi *mxic = spi_controller_get_devdata(desc->mem->spi->controller);
    int ret = 0;

    writel(mxic_uefc_spi_mem_prep_op_cfg(mxic, &desc->info.op_tmpl, len), mxic->regs + MAP_RD_CTRL);
    writel(desc->info.op_tmpl.cmd.opcode, mxic->regs + MAP_CMD_RD);

    if (mxic->ecc.use_pipelined_conf && desc->info.op_tmpl.data.ecc) {
        ret = mxic_ecc_process_data_pipelined(mxic->ecc.pipelined_engine,
                              NAND_PAGE_READ,
                              mxic->dirmap.raw_addr + (u32)offs);
        if (ret)
            return ret;
    } else {
        memcpy_fromio(buf, mxic->dirmap.addr + offs, len);
    }

    return len;
}

static ssize_t mxic_uefc_spi_mem_dirmap_write(struct spi_mem_dirmap_desc *desc,
                     u64 offs, size_t len, const void *buf)
{
    struct mxic_uefc_spi *mxic = spi_controller_get_devdata(desc->mem->spi->controller);
    int ret = 0;    

    writel(mxic_uefc_spi_mem_prep_op_cfg(mxic, &desc->info.op_tmpl, len), mxic->regs + MAP_WR_CTRL);
    writel(desc->info.op_tmpl.cmd.opcode, mxic->regs + MAP_CMD_WR);

    if (mxic->ecc.use_pipelined_conf && desc->info.op_tmpl.data.ecc) {
        ret = mxic_ecc_process_data_pipelined(mxic->ecc.pipelined_engine,
                              NAND_PAGE_WRITE,
                              mxic->dirmap.raw_addr + offs);
        if (ret)
            return ret;
    } else {
        memcpy_toio(mxic->dirmap.addr + offs, buf, len);
    }

    return len;
}


static bool mxic_uefc_spi_mem_supports_op(struct spi_mem *mem,
                     const struct spi_mem_op *op)
{
    if (op->data.buswidth > 8 || op->addr.buswidth > 8 ||
        op->dummy.buswidth > 8 || op->cmd.buswidth > 8)
        return false;

    if (op->addr.nbytes > 7)
        return false;
    return spi_mem_default_supports_op(mem, op);
}

static int mxic_uefc_spi_mem_dirmap_create(struct spi_mem_dirmap_desc *desc)
{
    struct mxic_uefc_spi *mxic = spi_controller_get_devdata(desc->mem->spi->controller);

    if (!mxic->dirmap.addr)
        return -EOPNOTSUPP;
    if ((desc->info.offset + desc->info.length) > mxic->dirmap.size)
        return -EINVAL;
    if (!mxic_uefc_spi_mem_supports_op(desc->mem, &desc->info.op_tmpl))
        return -EOPNOTSUPP;
    return 0;
}

static void mxic_uefc_spi_set_cs(struct spi_device *spi, bool assert_cs)
{
    struct mxic_uefc_spi *mxic = spi_controller_get_devdata(spi->controller);

    if (assert_cs) {
        writel(TFR_CTRL_IO_START, mxic->regs + TFR_CTRL);
        writel(TFR_CTRL_HC_ACT, mxic->regs + TFR_CTRL);
        writel(TFR_CTRL_DEV_ACT, mxic->regs + TFR_CTRL);
    } else {
        writel(TFR_CTRL_DEV_DIS, mxic->regs + TFR_CTRL);
        writel(TFR_CTRL_IO_END, mxic->regs + TFR_CTRL);
    }
}

static int mxic_uefc_spi_mem_exec_op(struct spi_mem *mem,
                const struct spi_mem_op *op)
{
    struct mxic_uefc_spi *mxic = spi_controller_get_devdata(mem->spi->controller);
    int i, ret = 0;
    u8 addr[8], cmd[2];
    
    if (ret)
        return ret;
    
    writel(mxic_uefc_spi_mem_prep_op_cfg(mxic, op, op->data.nbytes), mxic->regs + TFR_MODE);
    
    mxic_uefc_spi_set_cs(mem->spi, true);

    for (i = 0; i < op->cmd.nbytes; i++)
        cmd[i] = op->cmd.opcode >> (8 * (op->cmd.nbytes - i - 1));    

    ret = mxic_uefc_spi_io_mode_xfer(mxic, cmd, NULL, op->cmd.nbytes);
    if (ret)
        goto out;

    for (i = 0; i < op->addr.nbytes; i++)
        addr[i] = op->addr.val >> (8 * (op->addr.nbytes - i - 1));

    ret = mxic_uefc_spi_io_mode_xfer(mxic, addr, NULL, op->addr.nbytes);
    if (ret)
        goto out;

    ret = mxic_uefc_spi_io_mode_xfer(mxic, NULL, NULL, op->dummy.nbytes);
    if (ret)
        goto out;

    ret = mxic_uefc_spi_io_mode_xfer(mxic,
                 op->data.dir == SPI_MEM_DATA_OUT ?
                 op->data.buf.out : NULL,
                 op->data.dir == SPI_MEM_DATA_IN ?
                 op->data.buf.in : NULL,
                 op->data.nbytes);

out:
    mxic_uefc_spi_set_cs(mem->spi, false);
    return ret;
}

static const struct spi_controller_mem_ops mxic_uefc_spi_mem_ops = {
    .supports_op = mxic_uefc_spi_mem_supports_op,
    .exec_op = mxic_uefc_spi_mem_exec_op,
    .dirmap_create = mxic_uefc_spi_mem_dirmap_create,
    .dirmap_read = mxic_uefc_spi_mem_dirmap_read,
    .dirmap_write = mxic_uefc_spi_mem_dirmap_write,
};

static const struct spi_controller_mem_caps mxic_uefc_spi_mem_caps = {
    .dtr = true,
    .ecc = true,
};

/* ECC wrapper */
static int mxic_spi_mem_ecc_init_ctx(struct nand_device *nand)
{
    struct nand_ecc_engine_ops *ops = mxic_ecc_get_pipelined_ops();
    struct mxic_uefc_spi *mxic = nand->ecc.engine->priv;

    mxic->ecc.use_pipelined_conf = true;

    return ops->init_ctx(nand);
}

static void mxic_spi_mem_ecc_cleanup_ctx(struct nand_device *nand)
{
    struct nand_ecc_engine_ops *ops = mxic_ecc_get_pipelined_ops();
    struct mxic_uefc_spi *mxic = nand->ecc.engine->priv;

    mxic->ecc.use_pipelined_conf = false;

    ops->cleanup_ctx(nand);
}

static int mxic_spi_mem_ecc_prepare_io_req(struct nand_device *nand,
                       struct nand_page_io_req *req)
{
    struct nand_ecc_engine_ops *ops = mxic_ecc_get_pipelined_ops();

    return ops->prepare_io_req(nand, req);
}

static int mxic_spi_mem_ecc_finish_io_req(struct nand_device *nand,
                      struct nand_page_io_req *req)
{
    struct nand_ecc_engine_ops *ops = mxic_ecc_get_pipelined_ops();

    return ops->finish_io_req(nand, req);
}

static struct nand_ecc_engine_ops mxic_spi_mem_ecc_engine_pipelined_ops = {
    .init_ctx = mxic_spi_mem_ecc_init_ctx,
    .cleanup_ctx = mxic_spi_mem_ecc_cleanup_ctx,
    .prepare_io_req = mxic_spi_mem_ecc_prepare_io_req,
    .finish_io_req = mxic_spi_mem_ecc_finish_io_req,
};

static void mxic_uefc_spi_mem_ecc_remove(struct mxic_uefc_spi *mxic)
{
    if (mxic->ecc.pipelined_engine) {
        mxic_ecc_put_pipelined_engine(mxic->ecc.pipelined_engine);
        nand_ecc_unregister_on_host_hw_engine(mxic->ecc.pipelined_engine);
    }
}

static int mxic_uefc_spi_mem_ecc_probe(struct platform_device *pdev, struct mxic_uefc_spi *mxic)
{
    struct nand_ecc_engine *eng;

    if (!mxic_ecc_get_pipelined_ops())
        return -EOPNOTSUPP;
    eng = mxic_ecc_get_pipelined_engine(pdev);
    if (IS_ERR(eng))
        return PTR_ERR(eng);
    eng->dev = &pdev->dev;
    eng->integration = NAND_ECC_ENGINE_INTEGRATION_PIPELINED;
    eng->ops = &mxic_spi_mem_ecc_engine_pipelined_ops;
    eng->priv = mxic;
    mxic->ecc.pipelined_engine = eng;
    nand_ecc_register_on_host_hw_engine(eng);

    return 0;
}

static int __maybe_unused mxic_uefc_spi_runtime_suspend(struct device *dev)
{
    struct spi_controller *host = dev_get_drvdata(dev);
    struct mxic_uefc_spi *mxic = spi_controller_get_devdata(host);

    mxic_uefc_spi_clk_disable(mxic);
    clk_disable_unprepare(mxic->ps_clk);

    return 0;
}

static int __maybe_unused mxic_uefc_spi_runtime_resume(struct device *dev)
{
    struct spi_controller *host = dev_get_drvdata(dev);
    struct mxic_uefc_spi *mxic = spi_controller_get_devdata(host);
    int ret;

    ret = clk_prepare_enable(mxic->ps_clk);
    if (ret) {
        dev_err(dev, "Cannot enable ps_clock.\n");
        return ret;
    }

    return mxic_uefc_spi_clk_enable(mxic);
}

static const struct dev_pm_ops mxic_uefc_spi_dev_pm_ops = {
    SET_RUNTIME_PM_OPS(mxic_uefc_spi_runtime_suspend,
               mxic_uefc_spi_runtime_resume, NULL)
};

static int mxic_uefc_spi_probe(struct platform_device *pdev)
{
    struct spi_controller *host;
    struct resource *res;
    struct mxic_uefc_spi *mxic;
    int ret;

    host = devm_spi_alloc_host(&pdev->dev, sizeof(struct mxic_uefc_spi));
    if (!host)
        return -ENOMEM;

    platform_set_drvdata(pdev, host);

    mxic = spi_controller_get_devdata(host);
    mxic->dev = &pdev->dev;

    host->dev.of_node = pdev->dev.of_node;

    mxic->ps_clk = devm_clk_get(&pdev->dev, "ps_clk");
    if (IS_ERR(mxic->ps_clk))
        return PTR_ERR(mxic->ps_clk);

    mxic->send_clk = devm_clk_get(&pdev->dev, "send_clk");
    if (IS_ERR(mxic->send_clk))
        return PTR_ERR(mxic->send_clk);

    mxic->send_dly_clk = devm_clk_get(&pdev->dev, "send_dly_clk");
    if (IS_ERR(mxic->send_dly_clk))
        return PTR_ERR(mxic->send_dly_clk);

    mxic->regs = devm_platform_ioremap_resource_byname(pdev, "regs");
    if (IS_ERR(mxic->regs))
        return PTR_ERR(mxic->regs);

    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dirmap");
    mxic->dirmap.addr = devm_ioremap_resource(&pdev->dev, res);
    if (!IS_ERR(mxic->dirmap.addr)) {
        mxic->dirmap.raw_addr = res->start;
        mxic->dirmap.size = resource_size(res);
        writel(mxic->dirmap.raw_addr, mxic->regs + BASE_MAP_ADDR);
        writel(mxic->dirmap.raw_addr + mxic->dirmap.size, mxic->regs + TOP_MAP_ADDR);
    } else {
        mxic->dirmap.addr = NULL;
    }

    pm_runtime_enable(&pdev->dev);
     host->auto_runtime_pm = true;

    host->num_chipselect = 1;
    host->mem_ops = &mxic_uefc_spi_mem_ops;
    host->mem_caps = &mxic_uefc_spi_mem_caps;

    host->bits_per_word_mask = SPI_BPW_MASK(8);
    host->mode_bits = SPI_CPOL | SPI_CPHA |
              SPI_RX_DUAL | SPI_TX_DUAL |
              SPI_RX_QUAD | SPI_TX_QUAD |
              SPI_RX_OCTAL | SPI_TX_OCTAL;

    mxic_uefc_hw_init(mxic, DEV_CTRL_TYPE_SPI);

    ret = mxic_uefc_spi_mem_ecc_probe(pdev, mxic);
    if (ret == -EPROBE_DEFER) {
        pm_runtime_disable(&pdev->dev);
        return ret;
    }

    ret = spi_register_controller(host);
    if (ret) {
        dev_err(&pdev->dev, "spi_register_controller failed\n");
        pm_runtime_disable(&pdev->dev);
        mxic_uefc_spi_mem_ecc_remove(mxic);
    }

    return ret;
}

static void mxic_uefc_spi_remove(struct platform_device *pdev)
{
    struct spi_controller *host = platform_get_drvdata(pdev);
    struct mxic_uefc_spi *mxic = spi_controller_get_devdata(host);

    pm_runtime_disable(&pdev->dev);
    mxic_uefc_spi_mem_ecc_remove(mxic);
    spi_unregister_controller(host);
}

static const struct of_device_id mxic_uefc_spi_of_ids[] = {
    { .compatible = "mxic-uefc,uefc-spi", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxic_uefc_of_ids);

static struct platform_driver mxic_uefc_spi_driver = {
    .probe = mxic_uefc_spi_probe,
    .remove_new = mxic_uefc_spi_remove,
    .driver = {
        .name = "mxic-uefc-spi",
        .of_match_table = mxic_uefc_spi_of_ids,
        .pm = &mxic_uefc_spi_dev_pm_ops,
    },
};
module_platform_driver(mxic_uefc_spi_driver);

MODULE_AUTHOR("Alvin Zhou <alvinzhou@mxic.com.tw>");
MODULE_DESCRIPTION("UEFC controller driver");
MODULE_LICENSE("GPL v2");
