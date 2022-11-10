// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2018 Macronix International Co., Ltd.
//
// Authors:
//	Jaime Liao <jaimeliao@mxic.com.tw>
//

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
//include "mxic_UEFC.h"

//#define BIT(x) (1U<<(x))

// 0x000: Host Controller Register (HC_CTRL)
#define HC_RQE_EN_OFS						31
#define HC_SDMA_BD_OFS						28
#define HC_PARALLEL_1_OFS					27
#define HC_PARALLEL_0_OFS					26
#define HC_DATA_ORDER_OFS					25
#define HC_SIO_SHIFTER_OFS					23
#define HC_EX_SER_B						BIT(22)
#define HC_EX_SER_A						BIT(21)
#define HC_DES_INPUT_B						BIT(18)
#define HC_DES_INPUT_A						BIT(17)
#define HC_LED_OFS							16
#define HC_CH_SEL_OFS						15
	#define HC_CH_SEL_MASK						(1 << HC_CH_SEL_OFS)
	#define HC_CH_SEL_B							(1 << HC_CH_SEL_OFS)
	#define HC_CH_SEL_A							(0 << HC_CH_SEL_OFS)
#define HC_LUN_SEL_OFS						8
	#define HC_LUN_SEL_MASK						(127 << HC_PORT_SEL_OFS)
#define HC_PORT_SEL_OFS						0
	#define HC_PORT_SEL_MASK					(255 << HC_PORT_SEL_OFS)

// 0x004: Normal Interrupt Status Register (INT_STS)
// 0x00C: Normal Interrupt Status Enable Register (INT_STS_EN)
// 0x014: Normal Interrupt Signal Enable Register (INT_STS_SIG)
#define INT_ITF_DONE						BIT(31)
#define INT_CA_REQ							BIT(30)
#define INT_CACHE_RDY						BIT(29)
#define INT_AC_RDY							BIT(28)
#define INT_ERR								BIT(15)
#define INT_TFR_CMPLT						BIT(7)
#define INT_DMA								BIT(6)
#define INT_BUF_RD_RDY						BIT(5)
#define INT_BUF_WR_RDY						BIT(4)
#define INT_BLK_GAP							BIT(2)
#define INT_DAT_CMPLT						BIT(1)
#define INT_CMD_CMPLT						BIT(0)

#define INT_STS_ALL_EN_MASK					(INT_AC_RDY | INT_ERR | INT_TFR_CMPLT | INT_DMA)

// 0x008: Error Interrupt Status Register (ERR_INT_STS)
// 0x010: Error Interrupt Status Enable Register (ERR_INT_STS_EN)
// 0x018: Error Interrupt Signal Enable Register (ERR_INT_SIG)
#define INT_ECC_ERR							BIT(19)
#define INT_PREAM_ERR						BIT(18)
#define INT_CRC_ERR							BIT(17)
#define INT_AC_ERR							BIT(16)
#define INT_ADMA_ERR						BIT(9)
#define INT_AUTO_CMD_ERR					BIT(8)
#define INT_DATA_END_ERR					BIT(6)
#define INT_DATA_CRC_ERR					BIT(5)
#define INT_DATA_TIMEOUT					BIT(4)
#define INT_CMD_IDX_ERR						BIT(3)
#define INT_CMD_END_ERR						BIT(2)
#define INT_CMD_CRC_ERR						BIT(1)
#define INT_CMD_TIMEOUT						BIT(0)

#define INT_ERR_STS_ALL_EN_MASK				(INT_ECC_ERR | INT_PREAM_ERR | INT_CRC_ERR | INT_AC_ERR | INT_ADMA_ERR)

// 0x01C: Transfer Mode Register (TFR_MODE)
#define TFR_MODE_ENHC_OFS					29
#define TFR_MODE_PREAM_EN_OFS				28
#define TFR_MODE_DMY_CNT_OFS				21
#define TFR_MODE_ADR_CNT_OFS				18
#define TFR_MODE_CMD_CNT_OFS				17
#define TFR_MODE_DAT_BUS_OFS				14
#define TFR_MODE_ADR_BUS_OFS				11
#define TFR_MODE_CMD_BUS_OFS				8
#define TFR_MODE_MULT_BLK_OFS				5
#define TFR_MODE_DD_OFS						4
#define TFR_MODE_ACMD_OFS					2
#define TFR_MODE_BLK_CNT_EN_OFS				1
#define TFR_MODE_DMA_EN_OFS					0

#define CMD_BUS_1S							(0 << TFR_MODE_CMD_BUS_OFS)
#define CMD_BUS_2S							(1 << TFR_MODE_CMD_BUS_OFS)
#define CMD_BUS_4S							(2 << TFR_MODE_CMD_BUS_OFS)
#define CMD_BUS_8S							(3 << TFR_MODE_CMD_BUS_OFS)
#define CMD_BUS_1D							(4 << TFR_MODE_CMD_BUS_OFS)
#define CMD_BUS_2D							(5 << TFR_MODE_CMD_BUS_OFS)
#define CMD_BUS_4D							(6 << TFR_MODE_CMD_BUS_OFS)
#define CMD_BUS_8D							(7 << TFR_MODE_CMD_BUS_OFS)

#define ADR_BUS_1S							(0 << TFR_MODE_ADR_BUS_OFS)
#define ADR_BUS_2S							(1 << TFR_MODE_ADR_BUS_OFS)
#define ADR_BUS_4S							(2 << TFR_MODE_ADR_BUS_OFS)
#define ADR_BUS_8S							(3 << TFR_MODE_ADR_BUS_OFS)
#define ADR_BUS_1D							(4 << TFR_MODE_ADR_BUS_OFS)
#define ADR_BUS_2D							(5 << TFR_MODE_ADR_BUS_OFS)
#define ADR_BUS_4D							(6 << TFR_MODE_ADR_BUS_OFS)
#define ADR_BUS_8D							(7 << TFR_MODE_ADR_BUS_OFS)

#define DAT_BUS_1S							(0 << TFR_MODE_DAT_BUS_OFS)
#define DAT_BUS_2S							(1 << TFR_MODE_DAT_BUS_OFS)
#define DAT_BUS_4S							(2 << TFR_MODE_DAT_BUS_OFS)
#define DAT_BUS_8S							(3 << TFR_MODE_DAT_BUS_OFS)
#define DAT_BUS_1D							(4 << TFR_MODE_DAT_BUS_OFS)
#define DAT_BUS_2D							(5 << TFR_MODE_DAT_BUS_OFS)
#define DAT_BUS_4D							(6 << TFR_MODE_DAT_BUS_OFS)
#define DAT_BUS_8D							(7 << TFR_MODE_DAT_BUS_OFS)

// 0x020: Transfer Control Register (TFR_CTRL)
#define TFR_CTRL_DEV_DIS					BIT(18)
#define TFR_CTRL_IO_END						BIT(16)
#define TFR_CTRL_DEV_ACT					BIT(2)
#define TFR_CTRL_HC_ACT						BIT(1)
#define TFR_CTRL_IO_STRT					BIT(0)


// 0x024: Present State Register (PRES_STS)
#define PRES_STS_AC_RDY						BIT(28)
#define PRES_STS_RX_NFULL					BIT(19)
#define PRES_STS_RX_NEMPT					BIT(18)
#define PRES_STS_TX_NFULL					BIT(17)
#define PRES_STS_TX_EMPT					BIT(16)

// 0x0C0: Device Control Register (DEV_CTRL)
#define DEV_CTRL_TYPE_OFS					29
	#define DEV_CTRL_TYPE_MASK					(7 << DEV_CTRL_TYPE_OFS)
	#define DEV_CTRL_TYPE_SPI					(0 << DEV_CTRL_TYPE_OFS)
	#define DEV_CTRL_TYPE_LYBRA					(1 << DEV_CTRL_TYPE_OFS)
	#define DEV_CTRL_TYPE_OCTARAM				(2 << DEV_CTRL_TYPE_OFS)
	#define DEV_CTRL_TYPE_ONFI					(4 << DEV_CTRL_TYPE_OFS)
	#define DEV_CTRL_TYPE_EMMC					(6 << DEV_CTRL_TYPE_OFS)
#define DEV_CTRL_SCLK_SEL_OFS				25
	#define DEV_CTRL_SCLK_SEL_MASK				(15 << DEV_CTRL_SCLK_SEL_OFS)
	#define DEV_CTRL_SCLK_DIV_2					(0 << DEV_CTRL_SCLK_SEL_OFS)
	#define DEV_CTRL_SCLK_DIV_4					(1 << DEV_CTRL_SCLK_SEL_OFS)
	#define DEV_CTRL_SCLK_DIV_6					(2 << DEV_CTRL_SCLK_SEL_OFS)
	#define DEV_CTRL_SCLK_DIV_8					(3 << DEV_CTRL_SCLK_SEL_OFS)//(4 << DEV_CTRL_SCLK_SEL_OFS) 10MHz
#define DEV_CTRL_CACHEABLE_OFS				24
#define DEV_CTRL_WR_PLCY_OFS				22
#define DEV_CTRL_PAGE_SZ_OFS				19
#define DEV_CTRL_BLK_SZ_OFS					7
#define DEV_CTRL_PRE_DQS_OFS				6
#define DEV_CTRL_DQS_EN_OFS				 	5
#define DEV_CTRL_CRC_EN_OFS					4
#define DEV_CTRL_CRCB_IN_EN_OFS				3
#define DEV_CTRL_CRC_CHUNK_OFS				1
#define DEV_CTRL_CRCB_OUT_EN_OFS			0

// 0x0D8: Auto Calibration Control Register (AC_CTRL)
#define AC_WINDOWN_8						0
#define AC_WINDOWN_16						1
#define AC_WINDOWN_32						2
#define AC_WINDOWN_64						3

#define AC_CTRL_CMD_2_OFS					24
#define AC_CTRL_CMD_1_OFS					16
#define AC_CTRL_WINDOW_OFS					14
	#define AC_CTRL_WINDOW_8					(AC_WINDOWN_8 << AC_CTRL_WINDOW_OFS)
	#define AC_CTRL_WINDOW_16					(AC_WINDOWN_16 << AC_CTRL_WINDOW_OFS)
	#define AC_CTRL_WINDOW_32					(AC_WINDOWN_32 << AC_CTRL_WINDOW_OFS)
	#define AC_CTRL_WINDOW_64					(AC_WINDOWN_64 << AC_CTRL_WINDOW_OFS)
#define AC_CTRL_NVDDR_OFS					3
#define AC_CTRL_DQS_OFS						2
#define AC_CTRL_SAMPLE_OFS					1

// 0x0EC: Sample Point Adjust Register
#define DQS_IDLY_DOPI						24
#define DQS_IDLY_SOPI						16
#define DQS_ODLY						8
#define DQS_INV							BIT(7)
#define SAMP_POINT_DDR						3
#define SAMP_POINT_SDR						0

#define HC_CTRL 		0x000
#define INT_STS			0x004
#define ERR_INT_STS		0x008
#define INT_STS_EN		0x00c

#define ERR_INT_STS_EN		0x010
#define INT_STS_SIG		0x014
#define ERR_INT_SIG		0x018
#define TFR_MODE		0x01c

#define TFR_CTRL		0x020
#define PRES_STS		0x024
#define SDMA_CNT		0x028
#define SDMA_ADDR		0x02c

#define ADMA_ADDR		0x030
#define ADMA2_ADDR		0x034
#define BASE_ADDR			0x038
#define RESERVED_0		0x03c

#define DATA_BUF			0x040
#define SW_RST			0x044
#define TIMEOUT_CTRL		0x048
#define CLOCK_CTRL		0x04c

#define RESERVED_01		0x050
#define CACHE_CTRL		0x054
#define CAP_1				0x058
#define HC_VER			0x05c

#define RESERVED_02		0x060
#define RESERVED_03		0x064
#define RESERVED_04		0x068
#define RESERVED_05		0x06c

#define TXD_0				0x070
#define TXD_1				0x074
#define TXD_2				0x078
#define TXD_3				0x07c

#define RXD				0x080
#define RESERVED_06		0x084
#define RESERVED_07		0x088
#define RESERVED_08		0x08c

#define BLK_CNT			0x090
#define ARGU				0x094
#define TRF_MODE			0x098
#define RSP_1				0x09c

#define RSP_2				0x0a0
#define RSP_3				0x0a4
#define RSP_4				0x0a8
#define BUF_DATA_PORT		0x0ac

#define AUTO_ARGU 		0x0b0
#define ERR_ACMD_STS		0x0b4
#define RESERVED_09		0x0b8
#define RESERVED_10		0x0bc

#define DEV_CTRL			0x0c0
#define LRD_CONFIG		0x0c4
#define LWR_CONFIG		0x0c8
#define MAP_CMD			0x0cc

#define TOP_MAP_ADDR		0x0d0
#define GPIO				0x0d4
#define AC_CTRL			0x0d8
#define PREAM_1			0x0dc

#define PREAM_2			0x0e0
#define PREAM_3			0x0e4
#define PREAM_4			0x0e8
#define SAMPLE			0x0ec

#define SIO_IDLY_1		0x0f0
#define SIO_IDLY_2		0x0f4
#define SIO_ODLY_1		0x0f8
#define SIO_ODLY_2		0x0fc


struct mxic_spi {
	struct clk *ps_clk;
	struct clk *send_clk;
	struct clk *send_dly_clk;
	void __iomem *regs;
	u32 cur_speed_hz;
};

static int mxic_spi_clk_enable(struct mxic_spi *mxic)
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

static void mxic_spi_clk_disable(struct mxic_spi *mxic)
{
	clk_disable_unprepare(mxic->send_clk);
	clk_disable_unprepare(mxic->send_dly_clk);
}

//static void mxic_spi_set_input_delay_dqs(struct mxic_spi *mxic, u8 idly_code)
//{
//	writel(0, mxic->regs + SIO_IDLY_1);
//	writel(0, mxic->regs + SIO_IDLY_2);	
//}

static int mxic_spi_data_xfer(struct mxic_spi *mxic, const void *txbuf,
			      void *rxbuf, unsigned int len)
{
	unsigned int pos = 0;
//	uint32_t rd_data;
	while (pos < len) {
		unsigned int nbytes = len - pos;
		u32 data = 0xffffffff;
//		u32 sts;
//		int ret;

		if (nbytes > 4)
			nbytes = 4;

		if (txbuf){
			memcpy(&data, txbuf + pos, nbytes);

		}

		if(nbytes == 4){
			while (!(readl(mxic->regs + PRES_STS) & PRES_STS_TX_NFULL));
			writel(data, mxic->regs + TXD_0);
		}
		else if(nbytes == 3){
			while (!(readl(mxic->regs + PRES_STS) & PRES_STS_TX_NFULL));
			writel(data, mxic->regs + TXD_3);
		}
		else if(nbytes == 2){
			while (!(readl(mxic->regs + PRES_STS) & PRES_STS_TX_NFULL));
			writel(data, mxic->regs + TXD_2);
		}
		else if(nbytes == 1){
			while (!(readl(mxic->regs + PRES_STS) & PRES_STS_TX_NFULL));
			writel(data, mxic->regs + TXD_1);
			
		}

		if (rxbuf) {
			while(!(readl(mxic->regs + PRES_STS) & PRES_STS_RX_NEMPT));	
			data = readl(mxic->regs + RXD);
			memcpy(rxbuf + pos, &data, nbytes);
		} else {
			readl(mxic->regs + RXD);
		}

		pos += nbytes;
	}

	return 0;
}

static int calibration_manually(struct mxic_spi *mxic)
{
#define SFDP_SIGNATURE          0x50444653U
#define RDSFDP			0x5a
#define SFDP_HEADER_ADDR	0x0
	uint32_t reg_tfr_mode = 0, reg_hc_ctrl = 0, reg_dev_ctrl = 0;
	bool dqs_en = false;
	uint32_t cali_buf;
	int i, boundary_l = 0, boundary_h = 0, count = 0;

	//set up buswidth
	reg_tfr_mode = CMD_BUS_1S | ADR_BUS_1S | DAT_BUS_1S;


	//sl dqs
	if (dqs_en) {
		//printk("Jaime_trace: Setting DQS\n");
		reg_hc_ctrl = readl(mxic->regs + HC_CTRL);
		reg_hc_ctrl &= ~(HC_CH_SEL_MASK | HC_LUN_SEL_MASK | HC_PORT_SEL_MASK);
		reg_hc_ctrl |= ((0 << HC_CH_SEL_OFS) | (0 << HC_PORT_SEL_OFS));

		reg_dev_ctrl = readl(mxic->regs + DEV_CTRL);
		reg_dev_ctrl |= (1 << DEV_CTRL_DQS_EN_OFS);
		//Set sample point register for DTR mode
		writel(180 << DQS_IDLY_DOPI, mxic->regs + SAMPLE);

	} else {	
		reg_hc_ctrl = readl(mxic->regs + HC_CTRL);
		reg_hc_ctrl &= ~(HC_CH_SEL_MASK | HC_LUN_SEL_MASK | HC_PORT_SEL_MASK);
		reg_hc_ctrl |= ((0 << HC_CH_SEL_OFS) | (0 << HC_PORT_SEL_OFS));

		reg_dev_ctrl = readl(mxic->regs + DEV_CTRL);
		reg_dev_ctrl &= ~(1 << DEV_CTRL_DQS_EN_OFS);
	}

	writel(reg_hc_ctrl, mxic->regs + HC_CTRL);
	writel(reg_dev_ctrl, mxic->regs + DEV_CTRL);

	/* Set up the number of address cycles for 1S-1S-1S READ SFDP */
	reg_tfr_mode |= (0 << TFR_MODE_CMD_CNT_OFS);
	reg_tfr_mode |= (3 << TFR_MODE_ADR_CNT_OFS);
	reg_tfr_mode |= (1 << TFR_MODE_DMY_CNT_OFS);

	//disable preamable bit
	reg_tfr_mode |= (0 << TFR_MODE_PREAM_EN_OFS);
	//fixed at READ DATA mode
	reg_tfr_mode |= (1 << TFR_MODE_DD_OFS);

	/* Write Reg.TFR_MODE */
	writel(reg_tfr_mode, mxic->regs + TFR_MODE);

	for(i = 0;i < 8;i++) {
		writel(i << SAMP_POINT_SDR, mxic->regs + SAMPLE);
		//Enable IO mode
		writel(TFR_CTRL_IO_STRT, mxic->regs + TFR_CTRL);
		//setup the configuration of TFR_MODE
		writel(TFR_CTRL_HC_ACT, mxic->regs + TFR_CTRL);
		//assert CS
		writel(TFR_CTRL_DEV_ACT, mxic->regs + TFR_CTRL);

		while (!(readl(mxic->regs + PRES_STS) & PRES_STS_TX_NFULL));
		writel(RDSFDP, mxic->regs + TXD_1);
		readl(mxic->regs + RXD);

		while (!(readl(mxic->regs + PRES_STS) & PRES_STS_TX_NFULL));
		writel(SFDP_HEADER_ADDR, mxic->regs + TXD_3);
		readl(mxic->regs + RXD);


		while (!(readl(mxic->regs + PRES_STS) & PRES_STS_TX_NFULL));
		writel(0xFF, mxic->regs + TXD_1);
		readl(mxic->regs + RXD);

		while (!(readl(mxic->regs + PRES_STS) & PRES_STS_TX_NFULL));
		writel(0xFFFFFFFF, mxic->regs + TXD_0);
		
		while(!(readl(mxic->regs + PRES_STS) & PRES_STS_RX_NEMPT));
		cali_buf = readl(mxic->regs + RXD);
		if(cali_buf == SFDP_SIGNATURE) {
			printk("data = %x\n", cali_buf);
			count++;
			boundary_h = i;
			if(count == 1)
				boundary_l = i;
		}else {
			printk("data = %x\n", cali_buf);
		}
		//De-assert CS
		writel(TFR_CTRL_DEV_DIS, mxic->regs + TFR_CTRL);
		//Disable IO mode
		writel(TFR_CTRL_IO_END, mxic->regs + TFR_CTRL);
	}
	printk("Jaime_trace: cali result boundary_l = %d and boundary_h = %d\n", boundary_l, boundary_h);
	printk("(boundary_l + boundary_h)/2 = %d", (boundary_h + boundary_l)/2);
	
	writel(((boundary_l + boundary_h)/2) << SAMP_POINT_SDR, mxic->regs + SAMPLE);

	return 0;
}

static void mxic_spi_hw_init(struct mxic_spi *mxic)
{
	uint32_t reg_hc_ctrl;
	uint32_t reg_dev_ctrl;

	//Set host controller register
	//writel(0 << HC_DATA_ORDER_OFS | HC_EX_SER_B | HC_EX_SER_A | HC_DES_INPUT_B | HC_DES_INPUT_A, mxic->regs + HC_CTRL);
	//writel(0 << HC_DATA_ORDER_OFS | HC_DES_INPUT_B | HC_DES_INPUT_A, mxic->regs + HC_CTRL);
	//writel(0 << HC_DATA_ORDER_OFS | HC_EX_SER_B | HC_EX_SER_A, mxic->regs + HC_CTRL);
	writel(0 << HC_DATA_ORDER_OFS, mxic->regs + HC_CTRL);

	//Add SIO Line Shifter
	writel(2 << HC_SIO_SHIFTER_OFS, mxic->regs + HC_CTRL);
	
	//Set clock controller register
	//Set 0x07 means clock speed is 8*SS CTRL
	//writel(0x07 << 21 | 0x07 << 16 | 1, mxic->regs + CLOCK_CTRL);
	//writel(0x07 << 21 | 0x07 << 16 | 0, mxic->regs + CLOCK_CTRL);
	/* UEFC should know the ratio which is following FSBL setting */
	writel(0x01<< 21 | 0x01 << 16 | 1, mxic->regs + CLOCK_CTRL);
	
	//Set interrupt stats register
	writel(INT_STS_ALL_EN_MASK, mxic->regs + INT_STS);
	writel(INT_STS_ALL_EN_MASK, mxic->regs + INT_STS_EN);
	writel(INT_STS_ALL_EN_MASK, mxic->regs + INT_STS_SIG);


	writel(INT_ERR_STS_ALL_EN_MASK, mxic->regs + ERR_INT_STS);
	writel(INT_ERR_STS_ALL_EN_MASK, mxic->regs + ERR_INT_STS_EN);
	writel(INT_ERR_STS_ALL_EN_MASK, mxic->regs + ERR_INT_SIG);

	writel(0x00 << 7 | 0x00 << 6, mxic->regs + SAMPLE);
	//writel(4<<3|4, mxic->regs + SAMPLE);
	writel(0x00 << SAMP_POINT_SDR, mxic->regs + SAMPLE);

	writel(0, mxic->regs + SIO_IDLY_1);
	writel(0, mxic->regs + SIO_IDLY_2);
	writel(0, mxic->regs + SIO_ODLY_1);
	writel(0, mxic->regs + SIO_ODLY_2);

	//device present
	reg_hc_ctrl = readl(mxic->regs + HC_CTRL);
	reg_hc_ctrl &= ~(HC_CH_SEL_MASK | HC_PORT_SEL_MASK);
	//reg_hc_ctrl |= ((0 << HC_CH_SEL_OFS) | (0 << HC_PORT_SEL_OFS));
	writel(reg_hc_ctrl, mxic->regs + HC_CTRL);

	reg_dev_ctrl = readl(mxic->regs + DEV_CTRL);
	reg_dev_ctrl &= ~DEV_CTRL_TYPE_MASK;

	//Device type
	reg_dev_ctrl |= DEV_CTRL_TYPE_SPI;
	//Frequency
	reg_dev_ctrl &= ~DEV_CTRL_SCLK_SEL_MASK;
	reg_dev_ctrl |= DEV_CTRL_SCLK_DIV_2;
	writel(reg_dev_ctrl, mxic->regs + DEV_CTRL);

	calibration_manually(mxic);
}

static bool mxic_spi_mem_supports_op(struct spi_mem *mem,
				     const struct spi_mem_op *op)
{
//	bool all_false;

	if (op->data.buswidth > 8 || op->addr.buswidth > 8 ||
	    op->dummy.buswidth > 8 || op->cmd.buswidth > 8)
		return false;

	//if (op->data.nbytes && op->dummy.nbytes &&
	//    op->data.buswidth != op->dummy.buswidth)
	//	return false;

	if (op->addr.nbytes > 7)
		return false;

//	all_false = !op->cmd.dtr && !op->addr.dtr && !op->dummy.dtr &&
//		    !op->data.dtr;
//	printk("TODO: refuse all DTR ops\n");
//	if (!all_false)
//		return false;

//	if (all_false)
		return spi_mem_default_supports_op(mem, op);
//	else
//		return spi_mem_dtr_supports_op(mem, op);
}

static void mx_UEFC_config_io_mode(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct mxic_spi *mxic = spi_master_get_devdata(mem->spi->master);
	uint32_t reg_hc_ctrl = 0, reg_dev_ctrl = 0, reg_tfr_mode = 0;

	if (op->cmd.buswidth == 8){
		reg_tfr_mode = CMD_BUS_8D | ADR_BUS_8D | DAT_BUS_8D;
		//printk("xxxxxxxxxxxJaime_Trace: setup bus = 8-8-8 DTR in tfr mode\n");
	}
	else{
		reg_tfr_mode = CMD_BUS_1S | ADR_BUS_1S | DAT_BUS_1S;
		//printk("xxxxxxxxxxxJaime_Trace: setup bus = 1-1-1 STR in tfr mode\n");
	}
	//sl dqs
	if (op->data.dtr) {
		//printk("Jaime_trace: Setting DQS\n");
		reg_hc_ctrl = readl(mxic->regs + HC_CTRL);
		reg_hc_ctrl &= ~(HC_CH_SEL_MASK | HC_LUN_SEL_MASK | HC_PORT_SEL_MASK);
		reg_hc_ctrl |= ((0 << HC_CH_SEL_OFS) | (0 << HC_PORT_SEL_OFS));

		reg_dev_ctrl = readl(mxic->regs + DEV_CTRL);
		reg_dev_ctrl |= (1 << DEV_CTRL_DQS_EN_OFS);
		//Set sample point register
		writel(180 << DQS_IDLY_DOPI, mxic->regs + SAMPLE);

	} else {	
		reg_hc_ctrl = readl(mxic->regs + HC_CTRL);
		reg_hc_ctrl &= ~(HC_CH_SEL_MASK | HC_LUN_SEL_MASK | HC_PORT_SEL_MASK);
		reg_hc_ctrl |= ((0 << HC_CH_SEL_OFS) | (0 << HC_PORT_SEL_OFS));

		reg_dev_ctrl = readl(mxic->regs + DEV_CTRL);
		reg_dev_ctrl &= ~(1 << DEV_CTRL_DQS_EN_OFS);
	}

	writel(reg_hc_ctrl, mxic->regs + HC_CTRL);
	writel(reg_dev_ctrl, mxic->regs + DEV_CTRL);

//printk("cmd.nbytes = %d\n", op->cmd.nbytes);
//printk("addr.nbytes = %d\n", op->addr.nbytes);
//printk("dummy.nbytes = %d\n", op->dummy.nbytes);
	/* Set up the number of address cycles. */
	reg_tfr_mode |= ((op->cmd.nbytes == 1) ?  (0 << TFR_MODE_CMD_CNT_OFS) : (1 << TFR_MODE_CMD_CNT_OFS)) ;
	reg_tfr_mode |= (op->addr.nbytes << TFR_MODE_ADR_CNT_OFS);
	reg_tfr_mode |= (op->dummy.nbytes << TFR_MODE_DMY_CNT_OFS);

	//disable preamable bit
	reg_tfr_mode |= (0 << TFR_MODE_PREAM_EN_OFS);

	if (op->data.dir == SPI_MEM_DATA_IN) {
		reg_tfr_mode |= (1 << TFR_MODE_DD_OFS);
		//printk("Jaime_Trace: Data In , Set DD_OFS in TFR reg\n");
	}

	/* Write Reg.TFR_MODE */
	writel(reg_tfr_mode, mxic->regs + TFR_MODE);
}



static int mxic_spi_mem_exec_op(struct spi_mem *mem,
				const struct spi_mem_op *op)
{
	struct mxic_spi *mxic = spi_master_get_devdata(mem->spi->master);
	int /*nio = 1,*/ i, ret;
	u8 addr[8], cmd[2];
//printk("Jaime_Trace: opcode = %x\n", op->cmd.opcode);
	//setup the IO mode configurations
	mx_UEFC_config_io_mode(mem, op);

	//Enable IO mode
	writel(TFR_CTRL_IO_STRT, mxic->regs + TFR_CTRL);
	//setup the configuration of TFR_MODE
	writel(TFR_CTRL_HC_ACT, mxic->regs + TFR_CTRL);
	//assert CS
	writel(TFR_CTRL_DEV_ACT, mxic->regs + TFR_CTRL);

	for (i = 0; i < op->cmd.nbytes; i++)
		cmd[i] = op->cmd.opcode >> (8 * (op->cmd.nbytes - i - 1));

	ret = mxic_spi_data_xfer(mxic, cmd, NULL, op->cmd.nbytes);
	if (ret)
		goto out;

	for (i = 0; i < op->addr.nbytes; i++)
		addr[i] = op->addr.val >> (8 * (op->addr.nbytes - i - 1));

	ret = mxic_spi_data_xfer(mxic, addr, NULL, op->addr.nbytes);
	if (ret)
		goto out;

	ret = mxic_spi_data_xfer(mxic, NULL, NULL, op->dummy.nbytes);
	if (ret)
		goto out;

	ret = mxic_spi_data_xfer(mxic,
				 op->data.dir == SPI_MEM_DATA_OUT ?
				 op->data.buf.out : NULL,
				 op->data.dir == SPI_MEM_DATA_IN ?
				 op->data.buf.in : NULL,
				 op->data.nbytes);
	//De-assert CS
	writel(TFR_CTRL_DEV_DIS, mxic->regs + TFR_CTRL);
	//Disable IO mode
	writel(TFR_CTRL_IO_END, mxic->regs + TFR_CTRL);

	return ret;

out:
	//De-assert CS
	writel(TFR_CTRL_DEV_DIS, mxic->regs + TFR_CTRL);
	//Disable IO mode
	writel(TFR_CTRL_IO_END, mxic->regs + TFR_CTRL);

	return ret;
}

static const struct spi_controller_mem_ops mxic_spi_mem_ops = {
	.supports_op = mxic_spi_mem_supports_op,
	.exec_op = mxic_spi_mem_exec_op,
};

static void mxic_spi_set_cs(struct spi_device *spi, bool lvl)
{
//	struct mxic_spi *mxic = spi_master_get_devdata(spi->master);
/*
	if (!lvl) {
		writel(readl(mxic->regs + HC_CFG) | HC_CFG_MAN_CS_EN,
		       mxic->regs + HC_CFG);
		writel(HC_EN_BIT, mxic->regs + HC_EN);
		writel(readl(mxic->regs + HC_CFG) | HC_CFG_MAN_CS_ASSERT,
		       mxic->regs + HC_CFG);
	} else {
		writel(readl(mxic->regs + HC_CFG) & ~HC_CFG_MAN_CS_ASSERT,
		       mxic->regs + HC_CFG);
		writel(0, mxic->regs + HC_EN);
	}
*/
}

static int mxic_spi_transfer_one(struct spi_master *master,
				 struct spi_device *spi,
				 struct spi_transfer *t)
{
/*1
	struct mxic_spi *mxic = spi_master_get_devdata(master);
	int ret;
	int nio = 1;
	uint32_t reg_tfr_mode = 0;

	if (t->rx_buf && t->tx_buf) {
		if (((spi->mode & SPI_TX_QUAD) &&
		     !(spi->mode & SPI_RX_QUAD)) ||
		    ((spi->mode & SPI_TX_DUAL) &&
		     !(spi->mode & SPI_RX_DUAL)))
			return -ENOTSUPP;
	}
		
	if (ret)
		return ret;

	if (t->tx_buf) {
		if (spi->mode & SPI_TX_QUAD)
			nio = 4;
		else if (spi->mode & SPI_TX_DUAL)
			nio = 2;
	} else if (t->rx_buf) {
		if (spi->mode & SPI_RX_QUAD)
			nio = 4;
		else if (spi->mode & SPI_RX_DUAL)
			nio = 2;
	}

	if (nio == 8)
		reg_tfr_mode = CMD_BUS_8D | ADR_BUS_8D | DAT_BUS_8D;
	else
		reg_tfr_mode = CMD_BUS_1S | ADR_BUS_1S | DAT_BUS_1S;

	if (t->rx_buf)
		reg_tfr_mode |= (1 << TFR_MODE_DD_OFS);
*/
	/* Write Reg.TFR_MODE */
/*1	writel(reg_tfr_mode, mxic->regs + TFR_MODE);

	//Enable IO mode
	writel(TFR_CTRL_IO_STRT, mxic->regs + TFR_CTRL);
	//setup the configuration of TRF_MODE
	writel(TFR_CTRL_HC_ACT, mxic->regs + TFR_CTRL);
	//assert CS
	writel(TFR_CTRL_DEV_ACT, mxic->regs + TFR_CTRL);

	ret = mxic_spi_data_xfer(mxic, t->tx_buf, t->rx_buf, t->len);
	if (ret)
		return ret;

	spi_finalize_current_transfer(master);
*/
	return 0;
}

static int __maybe_unused mxic_spi_runtime_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct mxic_spi *mxic = spi_master_get_devdata(master);

	mxic_spi_clk_disable(mxic);
	clk_disable_unprepare(mxic->ps_clk);

	return 0;
}

static int __maybe_unused mxic_spi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct mxic_spi *mxic = spi_master_get_devdata(master);
	int ret;

	ret = clk_prepare_enable(mxic->ps_clk);
	if (ret) {
		dev_err(dev, "Cannot enable ps_clock.\n");
		return ret;
	}

	return mxic_spi_clk_enable(mxic);
}

static const struct dev_pm_ops mxic_spi_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(mxic_spi_runtime_suspend,
			   mxic_spi_runtime_resume, NULL)
};

static int mxic_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct resource *res;
	struct mxic_spi *mxic;
	int ret;
	printk("xxxxxxxxxxxxxxxxxxxxJaime_trace: %s\n", __func__);

	master = devm_spi_alloc_master(&pdev->dev, sizeof(struct mxic_spi));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	mxic = spi_master_get_devdata(master);

	master->dev.of_node = pdev->dev.of_node;

	mxic->ps_clk = devm_clk_get(&pdev->dev, "ps_clk");
	if (IS_ERR(mxic->ps_clk))
		return PTR_ERR(mxic->ps_clk);

	mxic->send_clk = devm_clk_get(&pdev->dev, "send_clk");
	if (IS_ERR(mxic->send_clk))
		return PTR_ERR(mxic->send_clk);

	mxic->send_dly_clk = devm_clk_get(&pdev->dev, "send_dly_clk");
	if (IS_ERR(mxic->send_dly_clk))
		return PTR_ERR(mxic->send_dly_clk);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	mxic->regs = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(mxic->regs))
		return PTR_ERR(mxic->regs);

	pm_runtime_enable(&pdev->dev);
	master->auto_runtime_pm = true;

	master->num_chipselect = 1;
	master->mem_ops = &mxic_spi_mem_ops;

	master->set_cs = mxic_spi_set_cs;
	master->transfer_one = mxic_spi_transfer_one;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->mode_bits = SPI_CPOL | SPI_CPHA |
			SPI_RX_DUAL | SPI_TX_DUAL |
			SPI_RX_QUAD | SPI_TX_QUAD |
			SPI_RX_OCTAL | SPI_TX_OCTAL;

	mxic_spi_hw_init(mxic);

	ret = spi_register_master(master);
	if (ret) {
		dev_err(&pdev->dev, "spi_register_master failed\n");
		pm_runtime_disable(&pdev->dev);
	}

	return ret;
}

static int mxic_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	spi_unregister_master(master);

	return 0;
}

static const struct of_device_id mxic_spi_of_ids[] = {
	{ .compatible = "mxicy,mx25f0a-spi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxic_spi_of_ids);

static struct platform_driver mxic_spi_driver = {
	.probe = mxic_spi_probe,
	.remove = mxic_spi_remove,
	.driver = {
		.name = "mxic-spi",
		.of_match_table = mxic_spi_of_ids,
		.pm = &mxic_spi_dev_pm_ops,
	},
};
module_platform_driver(mxic_spi_driver);

MODULE_AUTHOR("Jaime Liao <jaimeliao@mxic.com.tw>");
MODULE_DESCRIPTION("UEFC SPI controller driver");
MODULE_LICENSE("GPL v1");
