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

#define	CHIP_DELAY_TIMEOUT (msecs_to_jiffies(200)*10) /* Arbitrary value from old driver pxa3xx_nand.c */

#define NDCR (0x00) /* Control register */
#define NDTR0CS0 (0x04) /* Timing Parameter 0 for CS0 */
#define NDTR1CS0 (0x0C) /* Timing Parameter 1 for CS0 */
#define NDSR (0x14) /* Status Register */
#define NDDB (0x40) /* Data Buffer */
#define NDCB0 (0x48) /* Command Buffer0 */
#define NDCB1 (0x4C) /* Command Buffer1 */
#define NDCB2 (0x50) /* Command Buffer2 */
#define NDCB3 (0x54) /* Command Buffer3 */

#define NDCR_ND_RUN (0x1 << 28)

/*todo : supprimer ces trois lignes, utiliser celles de dessous */
#define NDSR_WRCMDREQ (0x1 << 0)
#define NDSR_RDDREQ (0x1 << 1)
#define NDSR_WRDREQ (0x1 << 2)
//todo : implement wait func et dev ready

/* NDCR and NDSR common fields */
#define ALL_INT (0xFFF)
//#define CMD_REQ_INT (0x1 << 0)
//#define RDD_REQ_INT (0x1 << 1)
//#define WRD_REQ_INT (0x1 << 2)

#define NDCR_CS1_CMDDM (0x1 << 7)
#define NDCR_CS0_CMDDM (0x1 << 8)
#define NDCR_RDYM (0x1 << 11)

#define NDSR_WRCMDREQ (0x1 << 0)
#define NDSR_CS1_CMDD (0x1 << 7)
#define NDSR_CS0_CMDD (0x1 << 8)
#define NDSR_RDY0 (0x1 << 11)
#define NDSR_RDY1 (0x1 << 12)

#define NDCB0_ADDR_CYC(x) ((x) << 16)
#define NDCB0_CMD_TYPE(x) ((x) << 21)
#define NDCB0_CSEL (0x1 << 24)
#define NDCB0_CMD_XTYPE(x) ((x) << 29)

#define TYPE_READ (0)
#define TYPE_WRITE (1)
#define TYPE_RESET (5)
#define TYPE_NAKED_CMD (6)
#define TYPE_NAKED_ADDR (7)
#define XTYPE_NAKED_READ (5)
#define xTYPE_NAKED_WRITE (5)

struct marvell_nfc {
	struct nand_hw_control controller;
	struct device *dev;
	void __iomem *regs;
	struct clk *nd_clk;
	struct completion cmdd, rdy;
	struct list_head chips;
};

static inline struct marvell_nfc *to_marvell_nfc(struct nand_hw_control *ctrl)
{
	return container_of(ctrl, struct marvell_nfc, controller);
}

//struct marvell_nand_chip_sel {
//	u8 cs;
//};

struct marvell_nand_chip {
	struct nand_chip nand;
	int selected;
	struct list_head node;
//	int addr_cycles;
//	int cmd_cycles;
//	int nsels;
//	struct marvell_nand_chip_sel sels[0];
};

static void marvell_nfc_disable_int(struct marvell_nfc *nfc, u32 int_mask)
{
	u32 reg;

	/* Writing 1 disables the interrupt */
	reg = readl(nfc->regs + NDCR);
	writel(reg | int_mask, nfc->regs + NDCR);
	printk("interrupt disabled %x\n", int_mask);
	/* Clear pending interrupts if any */
	reg = readl(nfc->regs + NDSR);
	writel(reg & ~int_mask, nfc->regs + NDSR);
}

static void marvell_nfc_enable_int(struct marvell_nfc *nfc, u32 int_mask)
{
	u32 reg;

	/* Clear pending interrupts if any */
	reg = readl(nfc->regs + NDSR);
	printk("enable int : clear pending int (ndsr %x)\n", reg);
	writel(reg & ~int_mask, nfc->regs + NDSR);

	/* Writing 0 enables the interrupt */
	reg = readl(nfc->regs + NDCR);
	writel(reg & ~int_mask, nfc->regs + NDCR);
	printk("end of enable int (ndcr %x)\n", readl(nfc->regs + NDCR));
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
	printk("irq handled (reg %x, rw %d, cmd %d)\n", reg, ndsr & (NDSR_RDY0|NDSR_RDY1), ndsr == (NDSR_CS0_CMDD|NDSR_CS1_CMDD));

	return status;
}

// juste pour le test, attrapé depuis pxa3xx_nand.c:629
#define NAND_STOP_DELAY		msecs_to_jiffies(40)
static int wait_run(struct marvell_nfc *nfc)
{
	int timeout = NAND_STOP_DELAY;
	u32 ndcr;

	/* wait RUN bit in NDCR become 0 */
	ndcr = readl(nfc->regs + NDCR);
	while ((ndcr & NDCR_ND_RUN) && (timeout-- > 0)) {
		ndcr = readl(nfc->regs + NDCR);
		udelay(1);
	}

	if (timeout <= 0) {
		ndcr &= ~NDCR_ND_RUN;
		writel(ndcr, nfc->regs + NDCR);
		printk("timeout attendant le wait run\n");
	}

	return 0;
}

/* Single CS command control */
/* Send controls to the nand chip */
static void marvell_nfc_cmd_ctrl(struct mtd_info *mtd, int cmd,
				unsigned int ctrl)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
	int ret, val;

	if (ctrl & (NAND_CLE | NAND_ALE)) {

/*		ret = readl_poll_timeout(nfc->regs + NDSR, val,
					 val & NDSR_WRCMDREQ, 1000, 5000);
		if (ret) {
			dev_err(nfc->dev,
				"Timeout on WRCMDRE (val %x)\n", val);
			return;
		}
*/

//		init_completion(&nfc->cmdd);
//		marvell_nfc_enable_int(nfc, (NDCR_CS0_CMDDM | NDCR_CS1_CMDDM));

		/* Deassert ND_RUN bit and clear the status register before issuing any command */
		writel(readl(nfc->regs + NDCR) & ~NDCR_ND_RUN, nfc->regs + NDCR);
		writel(readl(nfc->regs + NDSR), nfc->regs + NDSR);
		printk("NDSR 0x%x\n", readl(nfc->regs + NDSR));

		/* Assert ND_RUN bit and wait the NFC to be ready by issuing the WRCMDREQ status bit */
		writel(readl(nfc->regs + NDCR) | NDCR_ND_RUN, nfc->regs + NDCR);
		ret = readl_poll_timeout(nfc->regs + NDSR, val,
					 val & NDSR_WRCMDREQ, 1000, 5000);
		if (ret) {
			dev_err(nfc->dev,
				"Timeout on WRCMDRE (val %x)\n", val);
			return;
		}

		printk("NDSR after poll 0x%x\n", readl(nfc->regs + NDSR));

		/* NFC uses naked commands and naked addresses */
		if (ctrl & NAND_CLE) {
			printk("CLE... ");
			writel(NDCB0_CMD_TYPE(TYPE_NAKED_CMD) | cmd,
			       nfc->regs + NDCB0);
			printk("writing %x in NDCB0\n", NDCB0_CMD_TYPE(TYPE_NAKED_CMD) | cmd);
			writel(0, nfc->regs + NDCB0);
			writel(0, nfc->regs + NDCB0);
			writel(0, nfc->regs + NDCB0);

		} else if (ctrl & NAND_ALE) {
			printk("ALE... ");
			writel(NDCB0_CMD_TYPE(TYPE_NAKED_ADDR) | NDCB0_ADDR_CYC(1),
			       nfc->regs + NDCB0);
			printk("writing %x in NDCB0\n", NDCB0_CMD_TYPE(TYPE_NAKED_ADDR) | NDCB0_ADDR_CYC(1));
			writel(cmd, nfc->regs + NDCB0 /* C'EST LE 1 */);
			writel(0, nfc->regs + NDCB0);
			writel(0, nfc->regs + NDCB0);
		}

		/* Print ndcb* regs */
		printk("NDCB0 0x%x\n", readl(nfc->regs + NDCB0));
		printk("NDCB1 0x%x\n", readl(nfc->regs + NDCB1));
		printk("NDCB2 0x%x\n", readl(nfc->regs + NDCB2));
		printk("NDCB3 0x%x\n", readl(nfc->regs + NDCB3));
		printk("NDCR 0x%x\n", readl(nfc->regs + NDCR));
		printk("NDSR 0x%x\n", readl(nfc->regs + NDSR));


		//todo remove sleep
//		msleep(10);
		printk("end of ->cmd_ctrl() (ndcr %x)\n", readl(nfc->regs+NDCR));

//		wait_run(nfc);
		/* The command is being processed, wait for the ND_RUN bit to be cleared by the NFC. If not, we must clear it by hand after a timeout */
		ret = readl_poll_timeout(nfc->regs + NDCR, val,
					 (val & NDCR_ND_RUN) == 0, 1000, 5000);
		if (ret) {
			dev_err(nfc->dev,
				"Timeout on ND_RUN (val %x)\n", val);
			writel(readl(nfc->regs + NDCR) & ~NDCR_ND_RUN, nfc->regs + NDCR);
			return;
		}

		printk("NDSR after wait run 0x%x\n", readl(nfc->regs + NDSR));

//		if (!wait_for_completion_timeout(&nfc->cmdd, CHIP_DELAY_TIMEOUT)) {
//			dev_err(nfc->dev, "Timeout on command request\n");
//			return;
//		}
//		marvell_nfc_disable_int(nfc, (NDCR_CS0_CMDDM | NDCR_CS1_CMDDM));
	} else printk("ni ALE ni CLE\n");
}


static int marvell_nfc_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);

//	msleep(100);
//	printk("%s return always true after a 100ms delay !!!\n", __FUNCTION__);
	return !!(readl(nfc->regs + NDSR) & (NDSR_RDY0 | NDSR_RDY1));
}

static int marvell_nfc_waitfunc(struct mtd_info *mtd, struct nand_chip *nand)
{
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
	printk("DO NOT USE WAITFUNC YET\n");
	printk("init completion rdy waitfunc\n");
	init_completion(&nfc->rdy);

	marvell_nfc_enable_int(nfc, NDCR_RDYM);
	printk("wait for completion rdy (ndcr %x)\n", readl(nfc->regs+NDCR));
	if (!wait_for_completion_timeout(&nfc->rdy, CHIP_DELAY_TIMEOUT)) {
		dev_err(nfc->dev, "Timeout on read/write data request\n");
		return -ETIMEDOUT;
	}

	marvell_nfc_disable_int(nfc, NDCR_RDYM);

	return 0;
}

static void marvell_nfc_trigger_read_op(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);

	u32 val;
	int ret;

	writel(readl(nfc->regs + NDSR) & ~NDSR_RDDREQ, nfc->regs + NDSR);
//	init_completion(&nfc->rdd_req);

	/* Trigger the naked read operation */
	writel(NDCB0_CMD_TYPE(TYPE_READ) | NDCB0_CMD_XTYPE(XTYPE_NAKED_READ),
		nfc->regs + NDCB0);

	/* Wait the data to be in the NFC SRAM buffer */
//	marvell_nfc_waitfunc(mtd, mtd_to_nand(mtd));

	ret = readl_poll_timeout(nfc->regs + NDSR, val,
				val & NDSR_RDDREQ, 1000, 5000);
	if (ret) {
		dev_err(nfc->dev,
			"Timeout on RDDREQ while draining the FIFO\n");
		return;
	} else
		writel(val & ~NDSR_RDDREQ, nfc->regs + NDSR);

//	if (!wait_for_completion_timeout(&nfc->rdd_req, CHIP_DELAY_TIMEOUT)) {
//		dev_err(nfc->dev, "Timeout on read data request\n");
		/* Stop State Machine for next command cycle */
//todo		pxa3xx_nand_stop(info);
//	}
}

static u8 marvell_nfc_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
	int ret;
	marvell_nfc_trigger_read_op(mtd);
	ret = readl(nfc->regs + NDDB) /*& 0xFF*/;
	printk("read_byte->value %x\n", ret);

	return ret & 0xFF;
}

static u16 marvell_nfc_read_word(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
	int ret;
	printk("read_word :\n");
	marvell_nfc_trigger_read_op(mtd);
	ret = readl(nfc->regs + NDDB) & 0xFFFF;
	printk("value %x\n", ret);

	return ret;
}

static void marvell_nfc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
	int remaining_bytes = len % 4, i;
	u32 reg;
	printk("read buf (len %d): \n", len);
	marvell_nfc_trigger_read_op(mtd);

	/* FIFO reads are 4 bytes long */
	for (i = 0; i < (len / 4); ++i) {
		*buf = readl(nfc->regs + NDDB);
		printk("%x ", *buf);
		buf += 4;
	}

	/* Copy the last bytes if any one by one */
	if (remaining_bytes) {
		reg = readl(nfc->regs + NDDB);
		for (i = 0; i < remaining_bytes; ++i) {
			buf[i] = (reg & (0xFF << i)) >> i;
		}
		printk("%x ", *buf);
	}
	printk("\n");
}

static int marvell_nfc_init(struct marvell_nfc *nfc)
{
	int ret;
	u32 ndcr, ndsr;

	printk("%s\n", __FUNCTION__);
//	init_completion(&nfc->complete);
//	printk("enable int\n");
	/* Activate command, read and write data request interrupts */
//	marvell_nfc_enable_int(nfc, CMD_REQ_INT | RDD_REQ_INT | WRD_REQ_INT);
//	printk("ndcr %x, ndsr %x\n", readl(nfc->regs + NDCR), readl(nfc->regs + NDSR));

	/* Activate the NFCv2 */
	printk("run !\n");
	ndcr = readl(nfc->regs + NDCR);
	writel(ndcr | NDCR_ND_RUN, nfc->regs + NDCR);
	printk("ndcr before poll %x\n", readl(nfc->regs +NDCR));
	/* Wait for WRCMDREQ */
	printk("poll on nd\n");
	ret = readl_poll_timeout(nfc->regs + NDSR, ndsr,
				ndsr & NDSR_WRCMDREQ, 1000, 5000);
	printk("ndsr %x\n", ndsr);
	if (ret) {
		dev_err(nfc->dev,
			"Timeout on WRCMDREQ while initializating NFC\n");
		return -EIO;
	}


	/* Reset the command request bit */
	printk("reset cmd req\n");
	writel(ndsr & NDSR_WRCMDREQ, nfc->regs + NDSR);
	//todo remove sleep
	msleep(10);
/*	printk("wait for completion run\n");
	if (!wait_for_completion_timeout(&nfc->complete, CHIP_DELAY_TIMEOUT)) {
		dev_err(nfc->dev, "Timeout on read data request\n");
		return -ETIMEDOUT;
	}
*/
	/*todo : write 3 commands */
	writel(0, nfc->regs + NDCB0);
	writel(0, nfc->regs + NDCB1);
	writel(0, nfc->regs + NDCB2);
	writel(0, nfc->regs + NDCB3);
	printk("end of init\n");

	return 0;
}

/////////////////: todo OUT OF REWORK
#define NDTR0_tCH(c)	(min((c), 7) << 19)
#define NDTR0_tCS(c)	(min((c), 7) << 16)
#define NDTR0_tWH(c)	(min((c), 7) << 11)
#define NDTR0_tWP(c)	(min((c), 7) << 8)
#define NDTR0_tRH(c)	(min((c), 7) << 3)
#define NDTR0_tRP(c)	(min((c), 7) << 0)

#define NDTR1_tR(c)	(min((c), 65535) << 16)
#define NDTR1_tWHR(c)	(min((c), 15) << 4)
#define NDTR1_tAR(c)	(min((c), 15) << 0)

/* convert nano-seconds to nand flash controller clock cycles */
#define ns2cycle(ns, clk)	(int)((ns) * (clk / 1000000) / 1000)

struct pxa3xx_nand_timing {
	unsigned int	tCH;  /* Enable signal hold time */
	unsigned int	tCS;  /* Enable signal setup time */
	unsigned int	tWH;  /* ND_nWE high duration */
	unsigned int	tWP;  /* ND_nWE pulse time */
	unsigned int	tRH;  /* ND_nRE high duration */
	unsigned int	tRP;  /* ND_nRE pulse width */
	unsigned int	tR;   /* ND_nWE high to ND_nRE low for read */
	unsigned int	tWHR; /* ND_nWE high to ND_nRE low for status read */
	unsigned int	tAR;  /* ND_ALE low to ND_nRE low delay */
};

static int marvell_set_timings(struct mtd_info *mtd)
{
	const struct nand_sdr_timings *timings;

	/* use the common timing to make a try */
	timings = onfi_async_timing_mode_to_sdr_timings(0);
	if (IS_ERR(timings))
		return PTR_ERR(timings);

	{
		struct nand_chip *nand = mtd_to_nand(mtd);
		struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
		unsigned long nand_clk = clk_get_rate(nfc->nd_clk);
		uint32_t ndtr0, ndtr1;

		u32 tCH_min = DIV_ROUND_UP(timings->tCH_min, 1000);
		u32 tCS_min = DIV_ROUND_UP(timings->tCS_min, 1000);
		u32 tWH_min = DIV_ROUND_UP(timings->tWH_min, 1000);
		u32 tWP_min = DIV_ROUND_UP(timings->tWC_min - timings->tWH_min, 1000);
		u32 tREH_min = DIV_ROUND_UP(timings->tREH_min, 1000);
		u32 tRP_min = DIV_ROUND_UP(timings->tRC_min - timings->tREH_min, 1000);
		u32 tR = nand->chip_delay * 1000;
		u32 tWHR_min = DIV_ROUND_UP(timings->tWHR_min, 1000);
		u32 tAR_min = DIV_ROUND_UP(timings->tAR_min, 1000);

		/* fallback to a default value if tR = 0 */
		if (!tR)
			tR = 20000;

		ndtr0 = NDTR0_tCH(ns2cycle(tCH_min, nand_clk)) |
			NDTR0_tCS(ns2cycle(tCS_min, nand_clk)) |
			NDTR0_tWH(ns2cycle(tWH_min, nand_clk)) |
			NDTR0_tWP(ns2cycle(tWP_min, nand_clk)) |
			NDTR0_tRH(ns2cycle(tREH_min, nand_clk)) |
			NDTR0_tRP(ns2cycle(tRP_min, nand_clk));
		//todo
		ndtr0 = 0x65081212;

		ndtr1 = NDTR1_tR(ns2cycle(tR, nand_clk)) |
			NDTR1_tWHR(ns2cycle(tWHR_min, nand_clk)) |
			NDTR1_tAR(ns2cycle(tAR_min, nand_clk));
		//todo
		ndtr1 = 0x00ff8155;

		writel(ndtr0, nfc->regs + NDTR0CS0);
		writel(ndtr1, nfc->regs + NDTR1CS0);
	}

	return 0;
}
/////////////////: todo OUT OF REWORK END

static int marvell_nand_chip_init(struct device *dev, struct marvell_nfc *nfc,
				struct device_node *np)
{
	struct mtd_info *mtd;
	struct marvell_nand_chip *chip;
	struct nand_chip *nand;
	int ret;

//	int nsels;
//	int i;
//	u32 tmp;

/*	if (!of_get_property(np, "reg", &nsels))
		return -EINVAL;

	nsels /= sizeof(u32);
	if (!nsels) {
		dev_err(dev, "invalid reg property size\n");
		return -EINVAL;
	}
*/
	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL); //+ nsels * sizeof(struct marvell_nand_chip /* check this, i m not sure if this is the appropriate struct
	if (!chip) {
		dev_err(dev, "could not allocate chip\n");
		return -ENOMEM;
	}

//	chip->nsels = 1//nsels;
	chip->selected = -1;

	/*
	for (i = 0; i < nsels; i++) {
		ret = of_property_read_u32_index(np, "reg", i, &tmp);
		if (ret) {
			dev_err(dev, "could not retrieve reg property: %d\n",
				ret);
			return ret;
		}

		if (tmp > NFC_MAX_CS) {
			dev_err(dev,
				"invalid reg value: %u (max CS = 7)\n",
				tmp);
			return -EINVAL;
		}

		if (test_and_set_bit(tmp, &nfc->assigned_cs)) {
			dev_err(dev, "CS %d already assigned\n", tmp);
			return -EINVAL;
		}

		chip->sels[i].cs = tmp;

		if (!of_property_read_u32_index(np, "allwinner,rb", i, &tmp) &&
			tmp < 2) {
			chip->sels[i].rb.type = RB_NATIVE;
			chip->sels[i].rb.info.nativeid = tmp;
		} else {
			ret = of_get_named_gpio(np, "rb-gpios", i);
			if (ret >= 0) {
				tmp = ret;
				chip->sels[i].rb.type = RB_GPIO;
				chip->sels[i].rb.info.gpio = tmp;
				ret = devm_gpio_request(dev, tmp, "nand-rb");
				if (ret)
					return ret;

				ret = gpio_direction_input(tmp);
				if (ret)
					return ret;
			} else {
				chip->sels[i].rb.type = RB_NONE;
			}
		}
	}
*/

	nand = &chip->nand;
	/* Default tR value specified in the ONFI spec (chapter 4.15.1) */
	nand->chip_delay = 200;
	nand->controller = &nfc->controller;
	/*
	 * Set the ECC mode to the default value in case nothing is specified
	 * in the DT.
	 */
/* TODO */
	nand->ecc.mode = NAND_ECC_NONE;
	nand_set_flash_node(nand, np);
//empty	nand->select_chip = sunxi_nfc_select_chip;
	nand->cmd_ctrl = marvell_nfc_cmd_ctrl;
	nand->dev_ready = marvell_nfc_dev_ready;
//todo	nand->waitfunc = marvell_nfc_waitfunc;
	nand->read_byte = marvell_nfc_read_byte;
	nand->read_word = marvell_nfc_read_word;
	nand->read_buf = marvell_nfc_read_buf;
//	nand->write_buf = marvell_nfc_write_buf;
//todo	nand->setup_data_interface = sunxi_nfc_setup_data_interface;

	mtd = nand_to_mtd(nand);
	mtd->dev.parent = dev;
	printk("todo: timings\n");
	marvell_set_timings(mtd);
	printk("nand_chan_ident\n");
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

//	nand->options |= NAND_SUBPAGE_READ;

/*todo	ret = marvell_nand_ecc_init(mtd, &nand->ecc, np);
	if (ret) {
		dev_err(dev, "ECC init failed: %d\n", ret);
		return ret;
	}
*/
	printk("nand_scan_tail\n");
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
	int nb_chip = 0;

	if (nchips > 8) {
		dev_err(dev, "too many NAND chips: %d (max = 8)\n", nchips);
		return -EINVAL;
	}

	for_each_child_of_node(np, nand_np) {
		printk("nand chip init number %d\n", nb_chip++);
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


//	struct device_node *np = pdev->dev.of_node;
//	struct device_node *nand_np;
//	struct marvell_nand_ctrl *marvell;
//	struct nand_chip *chip;
//	struct mtd_info *mtd;
//	struct resource *res;
//	int nchips = 0;
//	int count = 0;

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

	nfc->nd_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(nfc->nd_clk))
		nfc->nd_clk = NULL;

	ret = clk_prepare_enable(nfc->nd_clk);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "failed to retrieve irq\n");
		return irq;
	}

	marvell_nfc_disable_int(nfc, ALL_INT);
	ret = devm_request_irq(dev, irq, marvell_nfc_irq,
			0, "marvell-nand", nfc);
	if (ret)
		return ret;

	ret = marvell_nfc_init(nfc);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, nfc);
	printk("nand chip init\n");
	marvell_nand_chips_init(dev, nfc);

	/* Only a single chip node is supported */
/*	count = of_get_child_count(np);
	if (count > 1)
		return -EINVAL;

	device_reset_optional(&pdev->dev);

	for_each_child_of_node(np, nand_np) {
		chip = devm_kzalloc(&pdev->dev, sizeof(struct nand_chip),
				    GFP_KERNEL);
		if (!chip)
			return -ENOMEM;

		chip->controller = &marvell->base;

		nand_set_flash_node(chip, nand_np);
		nand_set_controller_data(chip, marvell);

		mtd = nand_to_mtd(chip);
		mtd->dev.parent = &pdev->dev;
		mtd->priv = chip;

		chip->cmd_ctrl = marvell_nand_cmd_ctrl;
		chip->read_buf = marvell_nand_read_buf;
		chip->read_byte = marvell_nand_read_byte;
		chip->write_buf = marvell_nand_write_buf;
		chip->chip_delay = 30;

		// Scan to find existence of the device
		err = nand_scan(mtd, 1);
		if (err)
			return err;

		err = mtd_device_register(mtd, NULL, 0);
		if (err) {
			nand_release(mtd);
			return err;
		}

		marvell->chips[nchips] = chip;
		++nchips;
	}

	// Exit if no chips found
	if (!nchips)
		return -ENODEV;

	platform_set_drvdata(pdev, marvell);
*/

	return 0;
}

static int marvell_nfc_remove(struct platform_device *pdev)
{
	struct marvell_nfc *nfc = platform_get_drvdata(pdev);
//free interrupt (todo)
	marvell_nand_chips_cleanup(nfc);

	clk_disable_unprepare(nfc->nd_clk);

	return 0;
}

static const struct of_device_id marvell_nfc_ids[] = {
	{ .compatible = "marvell,armada370-nand" },
	{},
};
MODULE_DEVICE_TABLE(of, marvell_nand_match);

static struct platform_driver marvell_nfc_driver = {
	.driver	= {
		.name		= "marvell_nand",
		.of_match_table = marvell_nfc_ids,
	},
	.probe	= marvell_nfc_probe,
	.remove	= marvell_nfc_remove,
};

module_platform_driver(marvell_nfc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Marvell NAND controller driver");
