/*
 * Marvell Armada-385-GP-AP NAND controller driver
 *
 * Heavily based on oxnas_nand.c and sunxi_nand.c.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>

/* todo Arbitrary value from old driver pxa3xx_nand.c */
#define	CHIP_DELAY_TIMEOUT (msecs_to_jiffies(200)*10)
/* Data FIFO size stated in the Armada 38x family functional specificattion */
#define FIFO_SIZE 8
/* Greatest id for a nand chip select pin (range is [0; NFC_MAX_CS]) */
#define NFC_MAX_CS 3
/* Greatest id for a nand ready/busy pin (range is [0; NFC_MAX_RB]) */
#define NFC_MAX_RB 1

#define NDCR (0x00) /* Control register */
#define NDTR0CS0 (0x04) /* Timing Parameter 0 for CS0 */
#define NDTR1CS0 (0x0C) /* Timing Parameter 1 for CS0 */
#define NDSR (0x14) /* Status Register */
#define NDECCCTRL (0x28) /* ECC control */
#define NDDB (0x40) /* Data Buffer */
#define NDCB0 (0x48) /* Command Buffer0 */
#define NDCB1 (0x4C) /* Command Buffer1 */
#define NDCB2 (0x50) /* Command Buffer2 */
#define NDCB3 (0x54) /* Command Buffer3 */

#define NDCR_ND_RUN (0x1 << 28)
#define NDCR_ECC_EN (0x1 << 30)
#define NDCR_SPARE_EN (0x1 << 31)

#define NDSR_WRCMDREQ (0x1 << 0)
#define NDSR_RDDREQ (0x1 << 1)
#define NDSR_WRDREQ (0x1 << 2)

/* NDCR and NDSR common fields */
#define ALL_INT (0xFFF)
//#define CMD_REQ_INT (0x1 << 0)
//#define RDD_REQ_INT (0x1 << 1)
//#define WRD_REQ_INT (0x1 << 2)

#define NDCR_CS1_CMDDM (0x1 << 7)
#define NDCR_CS0_CMDDM (0x1 << 8)
#define NDCR_RDYM (0x1 << 11)
#define NDCR_RD_ID_CNT(x) (((x) & 0x7) << 16)

#define NDSR_WRCMDREQ (0x1 << 0)
#define NDSR_CS1_CMDD (0x1 << 7)
#define NDSR_CS0_CMDD (0x1 << 8)
#define NDSR_RDY0 (0x1 << 11)
#define NDSR_RDY1 (0x1 << 12)

#define NDCB0_ADDR_CYC(x) (((x) & 0x7) << 16)
#define NDCB0_CMD_TYPE(x) (((x) & 0x7) << 21)
#define NDCB0_CSEL (0x1 << 24)
#define NDCB0_RDY_BYP (0x1 << 27)
#define NDCB0_LEN_OVRD (0x1 << 28)
#define NDCB0_CMD_XTYPE(x) (((x) & 0x7) << 29)

#define TYPE_READ (0)
#define TYPE_WRITE (1)
#define TYPE_READ_ID (3)
#define TYPE_RESET (5)
#define TYPE_NAKED_CMD (6)
#define TYPE_NAKED_ADDR (7)
#define XTYPE_MONOLITHIC_READ (0)
#define XTYPE_LAST_NAKED_READ (1)
#define XTYPE_NAKED_READ (5)
#define XTYPE_NAKED_WRITE (5)
#define XTYPE_COMMAND_DISPATCH (5)

#define show(x) printk(#x ": 0x%08x [%s:%i]\n", readl(nfc->regs + x), __FUNCTION__, __LINE__)

struct marvell_nand_chip_sel {
	u8 cs;
	u8 rb;
};

struct marvell_nand_chip {
	struct nand_chip nand;
	struct list_head node;
	int selected;
	int nsels;
	struct marvell_nand_chip_sel sels[0];
};

static inline struct marvell_nand_chip *to_marvell_nand(struct nand_chip *nand)
{
	return container_of(nand, struct marvell_nand_chip, nand);
}

struct marvell_nfc {
	struct nand_hw_control controller;
	struct device *dev;
	void __iomem *regs;
	struct clk *nd_clk;
	struct completion cmdd, rdy;
	unsigned long assigned_cs;
	struct list_head chips;
	int new_cmd;
	u8 buf[FIFO_SIZE] __attribute__((aligned(4)));
	int buf_pos;
	int boundary;
};

static inline struct marvell_nfc *to_marvell_nfc(struct nand_hw_control *ctrl)
{
	return container_of(ctrl, struct marvell_nfc, controller);
}

static void marvell_nfc_disable_int(struct marvell_nfc *nfc, u32 int_mask)
{
	u32 reg;

	/* Writing 1 disables the interrupt */
	reg = readl(nfc->regs + NDCR);
	writel(reg | int_mask, nfc->regs + NDCR);

	/* Clear pending interrupts if any */
	writel(readl(nfc->regs + NDSR), nfc->regs + NDSR);
}

static void marvell_nfc_enable_int(struct marvell_nfc *nfc, u32 int_mask)
{
	u32 reg;

	/* Clear pending interrupts if any */
	writel(readl(nfc->regs + NDSR), nfc->regs + NDSR);

	/* Writing 0 enables the interrupt */
	reg = readl(nfc->regs + NDCR);
	writel(reg & ~int_mask, nfc->regs + NDCR);
}

static irqreturn_t marvell_nfc_irq(int irq, void *dev_id)
{
	struct marvell_nfc *nfc = dev_id;
	u32 reg, ndsr = 0;
	irqreturn_t status = IRQ_NONE;

	reg = readl(nfc->regs + NDSR);
	ndsr = reg;
	printk("irq (reg %x)\n", reg);

	/* Check command status */
	if (reg & (NDSR_CS0_CMDD | NDSR_CS1_CMDD)) {
		printk("irq cmd\n");
		complete(&nfc->cmdd);
		ndsr |= NDSR_CS0_CMDD | NDSR_CS1_CMDD;
		status = IRQ_HANDLED;
	}

	/* Check read/write data status */
	if (reg & (NDSR_RDY0 | NDSR_RDY1)) {
		printk("irq rdy\n");
		complete(&nfc->rdy);
		ndsr |= NDSR_RDY0 | NDSR_RDY1;
		status = IRQ_HANDLED;
	}

	/* Clear interrupts */
	writel(ndsr, nfc->regs + NDSR);
	printk("IRQ handled %d (NDSR %x)\n", status == IRQ_HANDLED, reg);

	return status;
}

/* Send controls to the nand chip */
static void marvell_nfc_cmd_ctrl(struct mtd_info *mtd, int data,
				unsigned int ctrl)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
	u32 ndcr, val;
	int ret;

	if (ctrl & (NAND_CLE | NAND_ALE)) {
		/*
		 * Flag used in the ->read/write{byte,word,buf}() helpers to
		 * trigger a naked read/write
		 */
		nfc->new_cmd = 1;

		/* Deassert ND_RUN and clear NDSR before issuing any command */
		ndcr = readl(nfc->regs + NDCR);
		writel(ndcr & ~NDCR_ND_RUN, nfc->regs + NDCR);
		writel(readl(nfc->regs + NDSR), nfc->regs + NDSR);

		/* Assert ND_RUN bit and wait the NFC to be ready */
		writel(ndcr | NDCR_ND_RUN, nfc->regs + NDCR);
		ret = readl_poll_timeout(nfc->regs + NDSR, val,
					 val & NDSR_WRCMDREQ, 0, 1000);
		if (ret) {
			dev_err(nfc->dev, "Timeout on WRCMDRE\n");
			return;
		}

		/* Command may be written, clear WRCMDREQ status bit */
		writel(NDSR_WRCMDREQ, nfc->regs + NDSR);

		/*
		 * Marvell NFC uses naked commands and naked addresses.
		 *
		 * NDCB{1-3} registers are RO while NDCB0 is RW.
		 * NFC waits for all these three registers to be written
		 * by the use of NDCB0 only (it acts as a FIFO)
		 */
		if (ctrl & NAND_CLE) {
			writel(NDCB0_CMD_TYPE(TYPE_NAKED_CMD) | data,
			       nfc->regs + NDCB0);
			writel(0, nfc->regs + NDCB0);
			writel(0, nfc->regs + NDCB0);
		} else if (ctrl & NAND_ALE) {
			writel(NDCB0_CMD_TYPE(TYPE_NAKED_ADDR) |
			       NDCB0_ADDR_CYC(1), nfc->regs + NDCB0);
			writel(data, nfc->regs + NDCB0);
			writel(0, nfc->regs + NDCB0);
		}

		/*
		 * The command is being processed, wait for the ND_RUN bit to be
		 * cleared by the NFC. If not, we must clear it by hand.
		 */
		ret = readl_poll_timeout(nfc->regs + NDCR, val,
					 (val & NDCR_ND_RUN) == 0, 0, 1000);
		if (ret) {
			dev_err(nfc->dev, "Timeout on ND_RUN\n");
			writel(readl(nfc->regs + NDCR) & ~NDCR_ND_RUN,
			       nfc->regs + NDCR);
			return;
		}
	}
}

static void marvell_nfc_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(nand);
	struct marvell_nfc *nfc = to_marvell_nfc(marvell_nand->nand.controller);

	
}

static int marvell_nfc_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);

	return !!(readl(nfc->regs + NDSR) & (NDSR_RDY0 | NDSR_RDY1));
}

//todo
static int marvell_nfc_waitfunc(struct mtd_info *mtd, struct nand_chip *nand)
{
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
//	printk("init completion rdy waitfunc\n");
//	init_completion(&nfc->rdy);

//	marvell_nfc_enable_int(nfc, NDCR_RDYM);
//	printk("wait for completion rdy (ndcr %x)\n", readl(nfc->regs+NDCR));
//	if (!wait_for_completion_timeout(&nfc->rdy, CHIP_DELAY_TIMEOUT)) {
//		dev_err(nfc->dev, "Timeout on read/write data request\n");
//		return -ETIMEDOUT;
//	}

//	marvell_nfc_disable_int(nfc, NDCR_RDYM);

	return 0;
}

static void marvell_nfc_do_naked_read(struct mtd_info *mtd, int len)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
	u32 ndcr, val;
	int ret;

	/* Deassert ND_RUN and clear NDSR before issuing any command */
	ndcr = readl(nfc->regs + NDCR);
	writel(ndcr & ~NDCR_ND_RUN, nfc->regs + NDCR);
	writel(readl(nfc->regs + NDSR), nfc->regs + NDSR);

	/* Assert ND_RUN bit and wait the NFC to be ready */
	writel(ndcr | NDCR_ND_RUN, nfc->regs + NDCR);
	ret = readl_poll_timeout(nfc->regs + NDSR, val,
				 val & NDSR_WRCMDREQ, 0, 1000);
	if (ret) {
		dev_err(nfc->dev, "Naked read operation: timeout on WRCMDRE\n");
		return;
	}

	/* Command may be written, clear WRCMDREQ status bit */
	writel(NDSR_WRCMDREQ, nfc->regs + NDSR);

	/* Trigger the naked read operation */
	writel(NDCB0_CMD_TYPE(TYPE_READ) |
	       NDCB0_CMD_XTYPE(XTYPE_LAST_NAKED_READ) |
	       NDCB0_LEN_OVRD,
	       nfc->regs + NDCB0);
	writel(0, nfc->regs + NDCB0);
	writel(0, nfc->regs + NDCB0);
	writel(len, nfc->regs + NDCB0);

	ret = readl_poll_timeout(nfc->regs + NDSR, val,
				 val & NDSR_RDDREQ, 0, 1000);
	if (ret) {
		dev_err(nfc->dev, "Naked read operation: timeout on RDDREQ\n");
		show(NDCR);
		show(NDSR);
		show(NDDB);
		show(NDECCCTRL);
		show(NDTR1CS0);
		return;
	}

	/* Clear RDDREQ flag */
	writel(NDSR_RDDREQ, nfc->regs + NDSR);
}

static void marvell_nfc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
	u32 *buf32;
	int rounded_len, last_step, i = 0;

	/*
	 * ->new_cmd flag indicates ->cmd_ctrl() has been triggered and a naked
	 * read operation must be done.
	 */
	if (nfc->new_cmd) {
		nfc->new_cmd = false;
		nfc->buf_pos = 0;
		nfc->boundary = 0;
	}

	/* If there are valid bytes in the local buffer, copy them */
	if (nfc->buf_pos) {
		i = min(len, FIFO_SIZE - nfc->buf_pos);
		memcpy(buf, &nfc->buf[nfc->buf_pos], i);
		nfc->buf_pos = (nfc->buf_pos + i) % FIFO_SIZE;
		/* If there is no more data to read, operation is finished */
		if (i == len) {
			return;
		}
	}

	/* Realize as much FIFO_SIZE aligned reads as possible */
	while (i < len - FIFO_SIZE) {
		/* Do a naked read any time boundary is reached */
		if (i >= nfc->boundary) {
			rounded_len = round_down(len, FIFO_SIZE);
			marvell_nfc_do_naked_read(mtd, rounded_len);
			nfc->boundary += rounded_len;
		}
		buf32 = (u32 *)&buf[i];
		buf32[0] = readl(nfc->regs + NDDB);
		buf32[1] = readl(nfc->regs + NDDB);
		i += FIFO_SIZE;
	}

	/* Remaining bytes (read operation is less than FIFO_SIZE bytes) */
	last_step = len - i;
	if (last_step) {
		marvell_nfc_do_naked_read(mtd, FIFO_SIZE);
		nfc->boundary += FIFO_SIZE;
		buf32 = (u32 *)nfc->buf;
		buf32[0] = readl(nfc->regs + NDDB);
		buf32[1] = readl(nfc->regs + NDDB);
		memcpy(buf + i, nfc->buf, last_step);
	}
	nfc->buf_pos = last_step;
}

static u8 marvell_nfc_read_byte(struct mtd_info *mtd)
{
	u8 data[1];

	marvell_nfc_read_buf(mtd, data, 1);
//	printk("Read byte: %c\n", data[0]);
	return data[0];
}

static u16 marvell_nfc_read_word(struct mtd_info *mtd)
{
	u8 data[2];

	marvell_nfc_read_buf(mtd, data, 2);

	return (u16)data[0];
}

//static int marvell_nfc_init(struct marvell_nfc *nfc)
//{
//	int ret;
//	u32 ndcr, ndsr;
//
//	/* Activate the NFCv2 */
//	ndcr = readl(nfc->regs + NDCR);
//	writel(ndcr | NDCR_ND_RUN, nfc->regs + NDCR);
//	/* Wait for WRCMDREQ */
//	ret = readl_poll_timeout(nfc->regs + NDSR, ndsr,
//				ndsr & NDSR_WRCMDREQ, 1000, 5000);
//	if (ret) {
//		dev_err(nfc->dev,
//			"Timeout on WRCMDREQ while initializating NFC\n");
//		return -EIO;
//	}
//
//	/* Reset the command request bit */
//	writel(ndsr & NDSR_WRCMDREQ, nfc->regs + NDSR);
//	//todo remove sleep
////	msleep(10);
//
//	/*todo : write 3 commands */
//	writel(0, nfc->regs + NDCB0);
//	writel(0, nfc->regs + NDCB1);
//	writel(0, nfc->regs + NDCB2);
//	writel(0, nfc->regs + NDCB3);
////	printk("end of init\n");
//
//	return 0;
//}

/////////////////: todo adapt OUT OF REWORK START
//#define NDTR0_tCH(c)	(min((c), 7) << 19)
//#define NDTR0_tCS(c)	(min((c), 7) << 16)
//#define NDTR0_tWH(c)	(min((c), 7) << 11)
//#define NDTR0_tWP(c)	(min((c), 7) << 8)
//#define NDTR0_tRH(c)	(min((c), 7) << 3)
//#define NDTR0_tRP(c)	(min((c), 7) << 0)
//
//#define NDTR1_tR(c)	(min((c), 65535) << 16)
//#define NDTR1_tWHR(c)	(min((c), 15) << 4)
//#define NDTR1_tAR(c)	(min((c), 15) << 0)
//
///* convert nano-seconds to nand flash controller clock cycles */
//#define ns2cycle(ns, clk)	(int)((ns) * (clk / 1000000) / 1000)
//
//struct pxa3xx_nand_timing {
//	unsigned int	tCH;  /* Enable signal hold time */
//	unsigned int	tCS;  /* Enable signal setup time */
//	unsigned int	tWH;  /* ND_nWE high duration */
//	unsigned int	tWP;  /* ND_nWE pulse time */
//	unsigned int	tRH;  /* ND_nRE high duration */
//	unsigned int	tRP;  /* ND_nRE pulse width */
//	unsigned int	tR;   /* ND_nWE high to ND_nRE low for read */
//	unsigned int	tWHR; /* ND_nWE high to ND_nRE low for status read */
//	unsigned int	tAR;  /* ND_ALE low to ND_nRE low delay */
//};
//
//static int marvell_set_timings(struct mtd_info *mtd)
//{
//	const struct nand_sdr_timings *timings;
//
//	/* use the common timing to make a try */
//	timings = onfi_async_timing_mode_to_sdr_timings(0);
//	if (IS_ERR(timings))
//		return PTR_ERR(timings);
//
//	{
//		struct nand_chip *nand = mtd_to_nand(mtd);
//		struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
//		unsigned long nand_clk = clk_get_rate(nfc->nd_clk);
//		uint32_t ndtr0, ndtr1;
//
//		u32 tCH_min = DIV_ROUND_UP(timings->tCH_min, 1000);
//		u32 tCS_min = DIV_ROUND_UP(timings->tCS_min, 1000);
//		u32 tWH_min = DIV_ROUND_UP(timings->tWH_min, 1000);
//		u32 tWP_min = DIV_ROUND_UP(timings->tWC_min - timings->tWH_min, 1000);
//		u32 tREH_min = DIV_ROUND_UP(timings->tREH_min, 1000);
//		u32 tRP_min = DIV_ROUND_UP(timings->tRC_min - timings->tREH_min, 1000);
//		u32 tR = nand->chip_delay * 1000;
//		u32 tWHR_min = DIV_ROUND_UP(timings->tWHR_min, 1000);
//		u32 tAR_min = DIV_ROUND_UP(timings->tAR_min, 1000);
//
//		/* fallback to a default value if tR = 0 */
//		if (!tR)
//			tR = 20000;
//
//		ndtr0 = NDTR0_tCH(ns2cycle(tCH_min, nand_clk)) |
//			NDTR0_tCS(ns2cycle(tCS_min, nand_clk)) |
//			NDTR0_tWH(ns2cycle(tWH_min, nand_clk)) |
//			NDTR0_tWP(ns2cycle(tWP_min, nand_clk)) |
//			NDTR0_tRH(ns2cycle(tREH_min, nand_clk)) |
//			NDTR0_tRP(ns2cycle(tRP_min, nand_clk));
//		//todo
//		ndtr0 = 0x65081212;
//
//		ndtr1 = NDTR1_tR(ns2cycle(tR, nand_clk)) |
//			NDTR1_tWHR(ns2cycle(tWHR_min, nand_clk)) |
//			NDTR1_tAR(ns2cycle(tAR_min, nand_clk));
//		//todo
//		ndtr1 = 0x00ff8155;
//
//		writel(ndtr0, nfc->regs + NDTR0CS0);
//		writel(ndtr1, nfc->regs + NDTR1CS0);
//	}
//
//	return 0;
//}
/////////////////: todo OUT OF REWORK END

static int marvell_nand_chip_init(struct device *dev, struct marvell_nfc *nfc,
				struct device_node *np)
{
	struct marvell_nand_chip *chip;
	struct mtd_info *mtd;
	struct nand_chip *nand;
	int nsels, ret, i;
	u32 tmp;

	/* Check there is only one line of CS pins */
	if (!of_get_property(np, "reg", &nsels))
		return -EINVAL;

	nsels /= sizeof(u32);
	if (!nsels) {
		dev_err(dev, "invalid reg property size\n");
		return -EINVAL;
	}

	/* Alloc the nand chip structure */
	chip = devm_kzalloc(dev,
			    sizeof(*chip) +
			    (nsels * sizeof(struct marvell_nand_chip_sel)),
			    GFP_KERNEL);
	if (!chip) {
		dev_err(dev, "could not allocate chip\n");
		return -ENOMEM;
	}

	chip->nsels = nsels;
	chip->selected = -1;

	for (i = 0; i < nsels; i++) {
		/* Retrieve CS id */
		ret = of_property_read_u32_index(np, "reg", i, &tmp);
		if (ret) {
			dev_err(dev, "could not retrieve reg property: %d\n",
				ret);
			return ret;
		}

		if (tmp > NFC_MAX_CS) {
			dev_err(dev, "invalid reg value: %u (max CS = %d)\n",
				tmp, NFC_MAX_CS);
			return -EINVAL;
		}

		if (test_and_set_bit(tmp, &nfc->assigned_cs)) {
			dev_err(dev, "CS %d already assigned\n", tmp);
			return -EINVAL;
		}

		chip->sels[i].cs = tmp;

		/* Retrieve RB id */
		ret = of_property_read_u32_index(np, "marvell,rb", i, &tmp);
		if (ret) {
			dev_err(dev, "could not retrieve RB property: %d\n",
				ret);
			return ret;
		}

		if (tmp > NFC_MAX_RB) {
			dev_err(dev, "invalid reg value: %u (max RB = %d)\n",
				tmp, NFC_MAX_RB);
			return -EINVAL;
		}

		chip->sels[i].rb = tmp;
	}

	nand = &chip->nand;
	/* todo: Default tR value specified in the ONFI spec (chapter 4.15.1) */
	nand->chip_delay = 200;
	nand->controller = &nfc->controller;

	// todo: delete, this is done in nand_scan_ident > nand_dt_init
	/* Set the ECC mode according to the DT */
	/* nand->ecc.mode = of_get_nand_ecc_mode(np); */
	/* if (nand->ecc.mode < 0) */
	/* 	nand->ecc.mode = NAND_ECC_NONE; */

	/* if (nand->ecc.mode == NAND_ECC_SOFT) { */
	/* 	nand->ecc.algo = of_get_nand_ecc_algo(np); */
	/* 	if (nand->ecc.algo < 0) { */
	/* 		dev_warn("default to soft BCH ecc mode\n"); */
	/* 		nand->ecc.algo = NAND_ECC_BCH; */
	/* 	} */
	/* } */

	nand_set_flash_node(nand, np);
	nand->select_chip = marvell_nfc_select_chip;
	nand->cmd_ctrl = marvell_nfc_cmd_ctrl;
	nand->dev_ready = marvell_nfc_dev_ready;
//todo	nand->waitfunc = marvell_nfc_waitfunc;
	nand->read_byte = marvell_nfc_read_byte;
	nand->read_word = marvell_nfc_read_word;
	nand->read_buf = marvell_nfc_read_buf;
//todo	nand->write_buf = marvell_nfc_write_buf;
//todo	nand->setup_data_interface = sunxi_nfc_setup_data_interface;

	mtd = nand_to_mtd(nand);
	mtd->dev.parent = dev;

//todo	marvell_set_timings(mtd);

	ret = nand_scan_ident(mtd, 1 /*chip->nsels*/, NULL);
	if (ret)
		return ret;

	if (nand->bbt_options & NAND_BBT_USE_FLASH) {
		/*
		 * We'll use a bad block table stored in-flash and don't
		 * allow writing the bad block marker to the flash.
		 */
		nand->bbt_options |= NAND_BBT_NO_OOB_BBM;
//todo		nand->bbt_td = &bbt_main_descr;
//todo		nand->bbt_md = &bbt_mirror_descr;
	}
	if (nand->options & NAND_NEED_SCRAMBLING)
		nand->options |= NAND_NO_SUBPAGE_WRITE;

//todo	nand->options |= NAND_SUBPAGE_READ;

/*todo	ret = marvell_nand_ecc_init(mtd, &nand->ecc, np);
	if (ret) {
		dev_err(dev, "ECC init failed: %d\n", ret);
		return ret;
	}
*/

	ret = nand_scan_tail(mtd);
	if (ret) {
		dev_err(dev, "nand_scan_tail failed: %d\n", ret);
		return ret;
	}

	ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		dev_err(dev, "failed to register mtd device: %d\n", ret);
		nand_release(mtd);
		return ret;
	}

	list_add_tail(&chip->node, &nfc->chips);

	return 0;
}

static int marvell_nand_chips_init(struct device *dev, struct marvell_nfc *nfc)
{
	struct device_node *np = dev->of_node;
	struct device_node *nand_np;
	int nchips = of_get_child_count(np);
	int ret;

	if (nchips > 8) {
		dev_err(dev, "too many NAND chips: %d (max = 8)\n", nchips);
		return -EINVAL;
	}

	for_each_child_of_node(np, nand_np) {
		ret = marvell_nand_chip_init(dev, nfc, nand_np);
		if (ret) {
			of_node_put(nand_np);
			return ret;
		}
	}

	return 0;
}

static void marvell_nand_chips_cleanup(struct marvell_nfc *nfc)
{
	struct marvell_nand_chip *entry, *temp;

	list_for_each_entry_safe(entry, temp, &nfc->chips, node) {
		nand_release(nand_to_mtd(&entry->nand));
//todo		marvell_nand_ecc_cleanup(&chip->nand.ecc);
		list_del(&entry->node);
	}
}

/*
 * Probe for the NAND device.
 */
static int marvell_nfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *r;
	struct marvell_nfc *nfc;
	int ret;
	int irq;

	printk("*** MARVELL NAND REWORK ***\n");
	/* Allocate memory for the device structure (and zero it) */
	nfc = devm_kzalloc(&pdev->dev, sizeof(struct marvell_nfc),
			     GFP_KERNEL);
	if (!nfc)
		return -ENOMEM;

	nfc->dev = dev;
	nand_hw_control_init(&nfc->controller);
	INIT_LIST_HEAD(&nfc->chips);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nfc->regs = devm_ioremap_resource(dev, r);
	if (IS_ERR(nfc->regs))
		return PTR_ERR(nfc->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "failed to retrieve irq\n");
		return irq;
	}

	nfc->nd_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(nfc->nd_clk))
		return PTR_ERR(nfc->nd_clk);

	ret = clk_prepare_enable(nfc->nd_clk);
	if (ret)
		return ret;

	marvell_nfc_disable_int(nfc, ALL_INT);
	ret = devm_request_irq(dev, irq, marvell_nfc_irq,
			0, "marvell-nand", nfc);
	if (ret)
		goto out_clk_unprepare;

//toto: this looks useless and may be deleted
//	ret = marvell_nfc_init(nfc);
//	if (ret)
//		return ret;
	//todo rendre ça plus joli
	writel(readl(nfc->regs + NDCR) & ~(NDCR_SPARE_EN | NDCR_ECC_EN),
	       nfc->regs + NDCR);
	writel(readl(nfc->regs + NDTR1CS0) & ~(1<<15), nfc->regs + NDTR1CS0);
	//todo et ça aussi
	writel(0, nfc->regs + NDECCCTRL);

	platform_set_drvdata(pdev, nfc);

	ret = marvell_nand_chips_init(dev, nfc);
	if (ret)
		goto out_clk_unprepare;

	return 0;

out_clk_unprepare:
	clk_disable_unprepare(nfc->nd_clk);

	return ret;
}

static int marvell_nfc_remove(struct platform_device *pdev)
{
	struct marvell_nfc *nfc = platform_get_drvdata(pdev);

	marvell_nand_chips_cleanup(nfc);

	clk_disable_unprepare(nfc->nd_clk);

	return 0;
}

static const struct of_device_id marvell_nfc_ids[] = {
	{ .compatible = "marvell,armada370-nfc" }, //todo s/nand/nfc/
	{},
};
MODULE_DEVICE_TABLE(of, marvell_nand_match);

static struct platform_driver marvell_nfc_driver = {
	.driver	= {
		.name		= "marvell-nfc",
		.of_match_table = marvell_nfc_ids,
	},
	.probe	= marvell_nfc_probe,
	.remove	= marvell_nfc_remove,
};

module_platform_driver(marvell_nfc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Marvell NAND controller driver");
