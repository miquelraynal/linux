/*
 * Marvell NAND flash controller driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (C) 2017 Marvell
 * Author: Miquel RAYNAL <miquel.raynal@free-electrons.com>
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/mtd/rawnand.h>
#include <linux/of_platform.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <asm/unaligned.h>
#include <linux/platform_data/mtd-nand-pxa3xx.h>

/* Data FIFO granularity, FIFO reads/writes must be a multiple of this length */
#define FIFO_DEPTH		8
#define FIFO_REP(x)		(x / sizeof(u32))
#define BCH_SEQ_READS		(32 / FIFO_DEPTH)
/* NFCv2 does not support transfers of larger chunks at a time */
#define MAX_CHUNK_SIZE		2112
/* Polling is done at a pace of POLL_PERIOD us until POLL_TIMEOUT is reached */
#define POLL_PERIOD		0
#define POLL_TIMEOUT		100000
/* Interrupt maximum wait period in ms */
#define IRQ_TIMEOUT		1000
/* Latency in clock cycles between SoC pins and NFC logic */
#define MIN_RD_DEL_CNT		3
/* Maximum number of contiguous address cycles */
#define MAX_ADDRESS_CYC		7
/* System control register and bit to enable NAND on some SoCs */
#define GENCONF_SOC_DEVICE_MUX	0x208
#define GENCONF_SOC_DEVICE_MUX_NFC_EN BIT(0)

/* NAND controller data flash control register */
#define NDCR			0x00
/* NAND interface timing parameter 0 register */
#define NDTR0			0x04
/* NAND interface timing parameter 1 register */
#define NDTR1			0x0C
/* NAND controller status register */
#define NDSR			0x14
/* NAND ECC control register */
#define NDECCCTRL		0x28
/* NAND controller data buffer register */
#define NDDB			0x40
/* NAND controller command buffer 0 register */
#define NDCB0			0x48
/* NAND controller command buffer 1 register */
#define NDCB1			0x4C
/* NAND controller command buffer 2 register */
#define NDCB2			0x50
/* NAND controller command buffer 3 register */
#define NDCB3			0x54

/* Data flash control register bitfields */
#define NDCR_ALL_INT		GENMASK(11, 0)
#define NDCR_CS1_CMDDM		BIT(7)
#define NDCR_CS0_CMDDM		BIT(8)
#define NDCR_RDYM		BIT(11)
#define NDCR_ND_ARB_EN		BIT(12)
#define NDCR_RA_START		BIT(15)
#define NDCR_RD_ID_CNT(x)	((x & 0x7) << 16)
#define NDCR_PAGE_SZ(x)		(x >= 2048 ? BIT(24) : 0)
#define NDCR_DWIDTH_M		BIT(26)
#define NDCR_DWIDTH_C		BIT(27)
#define NDCR_ND_RUN		BIT(28)
#define NDCR_ECC_EN		BIT(30)
#define NDCR_SPARE_EN		BIT(31)

/* NAND interface timing parameter registers bitfields */
#define NDTR0_TRP(x)		((min_t(unsigned int, x, 0xF) & 0x7) << 0)
#define NDTR0_TRH(x)		(min_t(unsigned int, x, 0x7) << 3)
#define NDTR0_ETRP(x)		((min_t(unsigned int, x, 0xF) & 0x8) << 3)
#define NDTR0_SEL_NRE_EDGE	BIT(7)
#define NDTR0_TWP(x)		(min_t(unsigned int, x, 0x7) << 8)
#define NDTR0_TWH(x)		(min_t(unsigned int, x, 0x7) << 11)
#define NDTR0_TCS(x)		(min_t(unsigned int, x, 0x7) << 16)
#define NDTR0_TCH(x)		(min_t(unsigned int, x, 0x7) << 19)
#define NDTR0_RD_CNT_DEL(x)	(min_t(unsigned int, x, 0xF) << 22)
#define NDTR0_SELCNTR		BIT(26)
#define NDTR0_TADL(x)		(min_t(unsigned int, x, 0x1F) << 27)

#define NDTR1_TAR(x)		(min_t(unsigned int, x, 0xF) << 0)
#define NDTR1_TWHR(x)		(min_t(unsigned int, x, 0xF) << 4)
#define NDTR1_TRHW(x)		(min_t(unsigned int, x / 16, 0x3) << 8)
#define NDTR1_PRESCALE		BIT(14)
#define NDTR1_WAIT_MODE		BIT(15)
#define NDTR1_TR(x)		(min_t(unsigned int, x, 0xFFFF) << 16)

/* NAND controller status register bitfields */
#define NDSR_WRCMDREQ		BIT(0)
#define NDSR_RDDREQ		BIT(1)
#define NDSR_WRDREQ		BIT(2)
#define NDSR_CORERR		BIT(3)
#define NDSR_UNCERR		BIT(4)
#define NDSR_CMDD(cs)		BIT(8 - cs)
#define NDSR_RDY(rb)		BIT(11 + rb)
#define NDSR_ERRCNT(x)		((x >> 16) & 0x1F)

/* NAND ECC control register bitfields */
#define NDECCTRL_BCH_EN		BIT(0)

/* NAND controller command buffer registers bitfields */
#define NDCB0_CMD1(x)		((x & 0xFF) << 0)
#define NDCB0_CMD2(x)		((x & 0xFF) << 8)
#define NDCB0_ADDR_CYC(x)	((x & 0x7) << 16)
#define NDCB0_DBC		BIT(19)
#define NDCB0_CMD_TYPE(x)	((x & 0x7) << 21)
#define NDCB0_CSEL		BIT(24)
#define NDCB0_RDY_BYP		BIT(27)
#define NDCB0_LEN_OVRD		BIT(28)
#define NDCB0_CMD_XTYPE(x)	((x & 0x7) << 29)

#define NDCB1_COLS(x)		((x & 0xFFFF) << 0)
#define NDCB1_ADDRS(x)		(x << 16)

#define NDCB2_ADDR5(x)		(((x >> 16) & 0xFF) << 0)

#define NDCB3_ADDR6(x)		((x & 0xFF) << 16)
#define NDCB3_ADDR7(x)		((x & 0xFF) << 24)

/* NAND controller command buffer 0 register 'type' and 'xtype' fields */
#define TYPE_READ		0
#define TYPE_WRITE		1
#define TYPE_ERASE		2
#define TYPE_READ_ID		3
#define TYPE_STATUS		4
#define TYPE_RESET		5
#define TYPE_NAKED_CMD		6
#define TYPE_NAKED_ADDR		7
#define TYPE_MASK		7
#define XTYPE_MONOLITHIC_RW	0
#define XTYPE_LAST_NAKED_RW	1
#define XTYPE_FINAL_COMMAND	3
#define XTYPE_READ		4
#define XTYPE_WRITE_DISPATCH	4
#define XTYPE_NAKED_RW		5
#define XTYPE_COMMAND_DISPATCH	6
#define XTYPE_MASK		7

/*
 * Marvell ECC engine works differently than the others, in order to limit the
 * size of the IP, hardware engineers choose to set a fixed strength at 16 bits
 * per subpage, and depending on a the desired strength needed by the NAND chip,
 * a particular layout mixing data/spare/ecc is defined, with a possible last
 * chunk smaller that the others.
 *
 * @writesize:		Full page size on which the layout applies
 * @chunk:		Desired ECC chunk size on which the layout applies
 * @strength:		Desired ECC strength (per chunk size bytes) on which the
 *			layout applies
 * @full_chunk_cnt:	Number of full-sized chunks, which is the number of
 *			repetitions of the pattern:
 *			(data_bytes + spare_bytes + ecc_bytes).
 * @data_bytes:		Number of data bytes per chunk
 * @spare_bytes:	Number of spare bytes per chunk
 * @ecc_bytes:		Number of ecc bytes per chunk
 * @last_chunk_cnt:	If there is a last chunk with a different size than
 *			the first ones, the next fields may not be empty
 * @last_data_bytes:	Number of data bytes in the last chunk
 * @last_spare_bytes:	Number of spare bytes in the last chunk
 * @last_ecc_bytes:	Number of ecc bytes in the last chunk
 */
struct marvell_hw_ecc_layout {
	/* Constraints */
	int writesize;
	int chunk;
	int strength;
	/* Corresponding layout */
	int full_chunk_cnt;
	int data_bytes;
	int spare_bytes;
	int ecc_bytes;
	int last_chunk_cnt;
	int last_data_bytes;
	int last_spare_bytes;
	int last_ecc_bytes;
};

#define MARVELL_LAYOUT(ws, dc, ds, fcc, db, sb, eb, lcc, ldb, lsb, leb) \
	{								\
		.writesize = ws,					\
		.chunk = dc,						\
		.strength = ds,						\
		.full_chunk_cnt = fcc,					\
		.data_bytes = db,					\
		.spare_bytes = sb,					\
		.ecc_bytes = eb,					\
		.last_chunk_cnt = lcc,					\
		.last_data_bytes = ldb,					\
		.last_spare_bytes = lsb,				\
		.last_ecc_bytes = leb,					\
	}

/* Layouts explained in AN-379_Marvell_SoC_NFC_ECC */
static const struct marvell_hw_ecc_layout marvell_nfc_layouts[] = {
	MARVELL_LAYOUT(  512,   512,  1,  1,  512,  0,  6,  0,  0,  0,  0),
	MARVELL_LAYOUT( 2048,   512,  1,  1, 2048, 40, 24,  0,  0,  0,  0),
	MARVELL_LAYOUT( 2048,   512,  4,  1, 2048, 32, 30,  0,  0,  0,  0),
	MARVELL_LAYOUT( 4096,   512,  4,  2, 2048, 32, 30,  0,  0,  0,  0),
	MARVELL_LAYOUT( 4096,   512,  8,  4, 1024,  0, 30,  1,  0, 64, 30),
};

/*
 * The Nand Flash Controller has up to 4 CE and 2 RB pins. The CE selection
 * is made by a field in NDCB0 register, and in another field in NDCB2 register.
 * The datasheet describes the logic with an error: ADDR5 field is once
 * declared at the beginning of NDCB2, and another time at its end. Because the
 * ADDR5 field of NDCB2 may be used by other bytes, it would be more logical
 * to use the last bit of this field instead of the first ones.
 *
 * @ndcb0_csel:		Value of the NDCB0 register with or without the flag
 *			selecting the wanted CE lane. This is set once when
 *			the Device Tree is probed.
 * @rb:			Ready/Busy pin for the flash chip
 */
struct marvell_nand_chip_sel {
	unsigned int cs;
	u32 ndcb0_csel;
	unsigned int rb;
};

/*
 * NAND chip structure: stores NAND chip device related information
 *
 * @chip:		Base NAND chip structure
 * @node:		Used to store NAND chips into a list
 * @layout		NAND layout when using hardware ECC
 * @ndtr0		Timing registers 0 value for this NAND chip
 * @ndtr1		Timing registers 1 value for this NAND chip
 * @selected_die:	Current active CS
 * @nsels:		Number of CS lines required by the NAND chip
 * @sels:		Array of CS lines descriptions
 */
struct marvell_nand_chip {
	struct nand_chip chip;
	struct list_head node;
	const struct marvell_hw_ecc_layout *layout;
	u32 ndtr0;
	u32 ndtr1;
	int addr_cyc;
	int selected_die;
	unsigned int nsels;
	struct marvell_nand_chip_sel sels[0];
};

static inline struct marvell_nand_chip *to_marvell_nand(struct nand_chip *chip)
{
	return container_of(chip, struct marvell_nand_chip, chip);
}

static inline struct marvell_nand_chip_sel *to_nand_sel(struct marvell_nand_chip
							*nand)
{
	return &nand->sels[nand->selected_die];
}

/*
 * NAND controller capabilities for distinction between compatible strings
 *
 * @max_cs_nb:		Number of Chip Select lines available
 * @max_rb_nb:		Number of Ready/Busy lines available
 * @need_system_controller: Indicates if the SoC needs to have access to the
 *                      system controller (ie. to enable the NAND controller)
 * @need_arbiter:	Indicates if NAND/NOR arbitration must be manually
 *                      enabled (PXA only)
 * @legacy_of_bindings:	Indicates if DT parsing must be done using the old
 *			fashion way
 */
struct marvell_nfc_caps {
	unsigned int max_cs_nb;
	unsigned int max_rb_nb;
	bool need_system_controller;
	bool need_arbiter;
	bool legacy_of_bindings;
};

/*
 * NAND controller structure: stores Marvell NAND controller information
 *
 * @controller:		Base controller structure
 * @dev:		Parent device (used to print error messages)
 * @regs:		NAND controller registers
 * @ecc_clk:		ECC block clock, two times the NAND controller clock
 * @complete:		Completion object to wait for NAND controller events
 * @assigned_cs:	Bitmask describing already assigned CS lines
 * @chips:		List containing all the NAND chips attached to
 *			this NAND controller
 * @caps:		NAND controller capabilities for each compatible string
 * @buf:		Controller local buffer to store a part of the read
 *			buffer when the read operation was not 8 bytes aligned
 *			as is the FIFO.
 * @buf_pos:		Position in the 'buf' buffer
 */
struct marvell_nfc {
	struct nand_hw_control controller;
	struct device *dev;
	void __iomem *regs;
	struct clk *ecc_clk;
	struct completion complete;
	unsigned long assigned_cs;
	struct list_head chips;
	struct nand_chip *selected_chip;
	const struct marvell_nfc_caps *caps;

	/*
	 * Buffer handling: @buf will be accessed byte-per-byter but also
	 * int-per-int when exchanging data with the NAND controller FIFO,
	 * 32-bit alignment is then required.
	 */
	u8 buf[FIFO_DEPTH] __aligned(sizeof(u32));
	int buf_pos;
};

static inline struct marvell_nfc *to_marvell_nfc(struct nand_hw_control *ctrl)
{
	return container_of(ctrl, struct marvell_nfc, controller);
}

/*
 * NAND controller timings expressed in NAND Controller clock cycles
 *
 * @tRP:		ND_nRE pulse width
 * @tRH:		ND_nRE high duration
 * @tWP:		ND_nWE pulse time
 * @tWH:		ND_nWE high duration
 * @tCS:		Enable signal setup time
 * @tCH:		Enable signal hold time
 * @tADL:		Address to write data delay
 * @tAR:		ND_ALE low to ND_nRE low delay
 * @tWHR:		ND_nWE high to ND_nRE low for status read
 * @tRHW:		ND_nRE high duration, read to write delay
 * @tR:			ND_nWE high to ND_nRE low for read
 */
struct marvell_nfc_timings {
	/* NDTR0 fields */
	unsigned int tRP;
	unsigned int tRH;
	unsigned int tWP;
	unsigned int tWH;
	unsigned int tCS;
	unsigned int tCH;
	unsigned int tADL;
	/* NDTR1 fields */
	unsigned int tAR;
	unsigned int tWHR;
	unsigned int tRHW;
	unsigned int tR;
};

/*
 * Derives a duration in numbers of clock cycles.
 *
 * @ps: Duration in pico-seconds
 * @period_ns:  Clock period in nano-seconds
 *
 * Convert the duration in nano-seconds, then divide by the period and
 * return the number of clock periods.
 */
#define TO_CYCLES(ps, period_ns) (DIV_ROUND_UP(ps / 1000, period_ns))

/*
 * NAND driver structure filled during the parsing of the ->exec_op() subop
 * subset of instructions.
 *
 * @ndcb:		Array for the values of the NDCBx registers
 * @cle_ale_delay_ns:	Optional delay after the last CMD or ADDR cycle
 * @rdy_timeout_ms:	Timeout for waits on Ready/Busy pin
 * @rdy_delay_ns:	Optional delay after waiting for the RB pin
 * @data_delay_ns:	Optional delay after the data xfer
 * @data_instr_idx:	Index of the data instruction in the subop
 * @data_instr:		Pointer to the data instruction in the subop
 */
struct marvell_nfc_op {
	u32 ndcb[4];
	unsigned int cle_ale_delay_ns;
	unsigned int rdy_timeout_ms;
	unsigned int rdy_delay_ns;
	unsigned int data_delay_ns;
	unsigned int data_instr_idx;
	const struct nand_op_instr *data_instr;
};

/*
 * Internal helper to conditionnally apply a delay (from the above structure,
 * most of the time).
 */
static void cond_delay(unsigned int ns)
{
	if (!ns)
		return;

	if (ns < 10000)
		ndelay(ns);
	else
		udelay(DIV_ROUND_UP(ns, 1000));
}

/*
 * Internal helper to mimic core functions whithout having to distinguish if
 * this is the first read operation on the page or not and hence choose the
 * right function.
 */
int read_page_data(struct nand_chip *chip, unsigned int page,
		   unsigned int column, void *buf, unsigned int len)
{
	if (!column)
		return nand_read_page_op(chip, page, column, buf, len);
	else
		return nand_change_read_column_op(chip, column, buf, len,
						  false);
}

/*
 * The controller has many flags that could generate interrupts, most of them
 * are disabled and polling is used. For the very slow signals, using interrupts
 * may relax the CPU charge.
 */
static void marvell_nfc_disable_int(struct marvell_nfc *nfc, u32 int_mask)
{
	u32 reg;

	/* Writing 1 disables the interrupt */
	reg = readl_relaxed(nfc->regs + NDCR);
	writel_relaxed(reg | int_mask, nfc->regs + NDCR);
}

static void marvell_nfc_enable_int(struct marvell_nfc *nfc, u32 int_mask)
{
	u32 reg;

	/* Writing 0 enables the interrupt */
	reg = readl_relaxed(nfc->regs + NDCR);
	writel_relaxed(reg & ~int_mask, nfc->regs + NDCR);
}

static void marvell_nfc_clear_int(struct marvell_nfc *nfc, u32 int_mask)
{
	writel_relaxed(int_mask, nfc->regs + NDSR);
}

/*
 * The core may ask the controller to use only 8-bit accesses while usually
 * using 16-bit accesses. Later function may blindly call this one with a
 * boolean to indicate if 8-bit accesses must be enabled of disabled without
 * knowing if 16-bit accesses are actually in use.
 */
static void marvell_nfc_force_byte_access(struct nand_chip *chip,
					  bool force_8bit)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 ndcr;

	if (!(chip->options & NAND_BUSWIDTH_16))
		return;

	ndcr = readl_relaxed(nfc->regs + NDCR);

	if (force_8bit)
		ndcr &= ~(NDCR_DWIDTH_M | NDCR_DWIDTH_C);
	else
		ndcr |= NDCR_DWIDTH_M | NDCR_DWIDTH_C;

	writel_relaxed(ndcr, nfc->regs + NDCR);
}

/*
 * Any time a command has to be sent to the controller, the following sequence
 * has to be followed:
 * - call marvell_nfc_prepare_cmd()
 *      -> activate the ND_RUN bit that will kind of 'start a job'
 *      -> wait the signal indicating the NFC is waiting for a command
 * - send the command (cmd and address cycles)
 * - enventually send or receive the data
 * - call marvell_nfc_end_cmd() with the corresponding flag
 *      -> wait the flag to be triggered or cancel the job with a timeout
 *
 * The following functions are helpers to do this job and keep in the
 * specialized functions the code that really does the operations.
 */
static int marvell_nfc_prepare_cmd(struct nand_chip *chip)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 ndcr, val;
	int ret;

	/* Deassert ND_RUN and clear NDSR before issuing any command */
	ndcr = readl_relaxed(nfc->regs + NDCR);
	writel_relaxed(ndcr & ~NDCR_ND_RUN, nfc->regs + NDCR);
	writel_relaxed(readl_relaxed(nfc->regs + NDSR), nfc->regs + NDSR);

	/* Assert ND_RUN bit and wait the NFC to be ready */
	writel_relaxed(ndcr | NDCR_ND_RUN, nfc->regs + NDCR);
	ret = readl_relaxed_poll_timeout(nfc->regs + NDSR, val,
					 val & NDSR_WRCMDREQ,
					 POLL_PERIOD, POLL_TIMEOUT);
	if (ret) {
		dev_err(nfc->dev, "Timeout on WRCMDRE\n");
		return -ETIMEDOUT;
	}

	/* Command may be written, clear WRCMDREQ status bit */
	writel_relaxed(NDSR_WRCMDREQ, nfc->regs + NDSR);

	return 0;
}

static void marvell_nfc_send_cmd(struct nand_chip *chip,
				 struct marvell_nfc_op *nfc_op)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);

	writel_relaxed(to_nand_sel(marvell_nand)->ndcb0_csel | nfc_op->ndcb[0],
		       nfc->regs + NDCB0);
	writel_relaxed(nfc_op->ndcb[1], nfc->regs + NDCB0);
	writel(nfc_op->ndcb[2], nfc->regs + NDCB0);

	/*
	 * Write NDCB0 four times only if LEN_OVRD is set or if ADDR6 or ADDR7
	 * fields are used.
	 */
	if (nfc_op->ndcb[0] & NDCB0_LEN_OVRD ||
	    (nfc_op->ndcb[0] & NDCB0_ADDR_CYC(6)) == NDCB0_ADDR_CYC(6))
		writel(nfc_op->ndcb[3], nfc->regs + NDCB0);
}

static int marvell_nfc_wait_ndrun(struct nand_chip *chip)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 val;
	int ret;

	/*
	 * The command is being processed, wait for the ND_RUN bit to be
	 * cleared by the NFC. If not, we must clear it by hand.
	 */
	ret = readl_relaxed_poll_timeout(nfc->regs + NDCR, val,
					 (val & NDCR_ND_RUN) == 0,
					 POLL_PERIOD, POLL_TIMEOUT);
	if (ret) {
		dev_err(nfc->dev, "Timeout on NAND controller run mode\n");
		writel_relaxed(readl_relaxed(nfc->regs + NDCR) & ~NDCR_ND_RUN,
			       nfc->regs + NDCR);
		return ret;
	}

	return 0;
}

static int marvell_nfc_end_cmd(struct nand_chip *chip, int flag,
			       const char *label)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 val;
	int ret;

	ret = readl_relaxed_poll_timeout(nfc->regs + NDSR, val,
					 val & flag,
					 POLL_PERIOD, POLL_TIMEOUT);
	if (ret) {
		dev_err(nfc->dev, "Timeout on %s\n", label);
		return ret;
	}

	writel_relaxed(flag, nfc->regs + NDSR);

	return 0;
}

static int marvell_nfc_wait_cmdd(struct nand_chip *chip)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	int cs_flag = NDSR_CMDD(to_nand_sel(marvell_nand)->ndcb0_csel);

	return marvell_nfc_end_cmd(chip, cs_flag, "CMDD");
}

static int marvell_nfc_wait_op(struct nand_chip *chip, unsigned int timeout_ms)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	int ret;

	/* Timeout is expressed in ms */
	if (!timeout_ms)
		timeout_ms = IRQ_TIMEOUT;

	init_completion(&nfc->complete);

	marvell_nfc_enable_int(nfc, NDCR_RDYM);
	ret = wait_for_completion_timeout(&nfc->complete,
					  msecs_to_jiffies(timeout_ms));
	marvell_nfc_disable_int(nfc, NDCR_RDYM);
	marvell_nfc_clear_int(nfc, NDSR_RDY(0) | NDSR_RDY(1));

	if (!ret)
		dev_err(nfc->dev, "Timeout waiting for RB signal\n");

	return ret ? 0 : -ETIMEDOUT;
}

static void marvell_nfc_select_chip(struct mtd_info *mtd, int die_nr)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 ndcr;

	if (chip == nfc->selected_chip && die_nr == marvell_nand->selected_die)
		return;

	if (die_nr < 0 || die_nr >= marvell_nand->nsels) {
		nfc->selected_chip = NULL;
		marvell_nand->selected_die = -1;
		return;
	}

	/*
	 * Do not change the timing registers when using the DT property
	 * marvell,nand-keep-config; in that case ->ndtr0 and ->ndtr1 from the
	 * marvell_nand structure are supposedly empty.
	 */
	if (marvell_nand->ndtr0 && marvell_nand->ndtr1) {
		writel_relaxed(marvell_nand->ndtr0, nfc->regs + NDTR0);
		writel_relaxed(marvell_nand->ndtr1, nfc->regs + NDTR1);
	}

	ndcr = readl_relaxed(nfc->regs + NDCR);

	/* Ensure controller is not blocked in operation */
	ndcr &= ~NDCR_ND_RUN;

	/* Adapt bus width */
	if (chip->options & NAND_BUSWIDTH_16)
		ndcr |= NDCR_DWIDTH_M | NDCR_DWIDTH_C;

	/*
	 * Page size as seen by the controller, either 512B or 2kiB. This size
	 * will be the reference for the controller when using LEN_OVRD.
	 */
	ndcr |= NDCR_PAGE_SZ(mtd->writesize);

	/* Update the control register */
	writel_relaxed(ndcr,  nfc->regs + NDCR);

	/* Also reset the interrupt status register */
	marvell_nfc_clear_int(nfc, NDCR_ALL_INT);

	marvell_nand->selected_die = die_nr;
}

static irqreturn_t marvell_nfc_isr(int irq, void *dev_id)
{
	struct marvell_nfc *nfc = dev_id;
	u32 st = readl_relaxed(nfc->regs + NDSR);
	u32 ien = (~readl_relaxed(nfc->regs + NDCR)) & NDCR_ALL_INT;

	/*
	 * RDY interrupt mask is one bit in NDCR while there are two status
	 * bit in NDSR (RDY[cs0/cs2] and RDY[cs1/cs3]).
	 */
	if (st & NDSR_RDY(1))
		st |= NDSR_RDY(0);

	if (!(st & ien))
		return IRQ_NONE;

	marvell_nfc_disable_int(nfc, st & NDCR_ALL_INT);

	complete(&nfc->complete);

	return IRQ_HANDLED;
}

/* HW ECC related functions */
static void marvell_nfc_hw_ecc_enable(struct nand_chip *chip)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 ndcr = readl_relaxed(nfc->regs + NDCR);

	if (!(ndcr & NDCR_ECC_EN)) {
		writel_relaxed(ndcr | NDCR_ECC_EN | NDCR_SPARE_EN,
			       nfc->regs + NDCR);

		/*
		 * When enabling BCH, set threshold to 0 to always know the
		 * number of corrected bitflips.
		 */
		if (chip->ecc.algo == NAND_ECC_BCH)
			writel_relaxed(NDECCTRL_BCH_EN, nfc->regs + NDECCCTRL);
	}
}

static void marvell_nfc_hw_ecc_disable(struct nand_chip *chip)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 ndcr = readl_relaxed(nfc->regs + NDCR);

	if (ndcr & NDCR_ECC_EN) {
		writel_relaxed(ndcr & ~(NDCR_ECC_EN | NDCR_SPARE_EN),
			       nfc->regs + NDCR);
		if (chip->ecc.algo == NAND_ECC_BCH)
			writel_relaxed(0, nfc->regs + NDECCCTRL);
	}
}

static void marvell_nfc_hw_ecc_correct(struct nand_chip *chip,
				       u8 *data, int data_len,
				       u8 *oob, int oob_len,
				       unsigned int *max_bitflips)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	int bf = 0;
	u32 ndsr;

	ndsr = readl_relaxed(nfc->regs + NDSR);

	/* Check uncorrectable error flag */
	if (ndsr & NDSR_UNCERR) {
		writel_relaxed(ndsr, nfc->regs + NDSR);

		/*
		 * Blank pages (all 0xFF) with no ECC are recognized as bad
		 * because hardware ECC engine expects non-empty ECC values
		 * in that case, so whenever an uncorrectable error occurs,
		 * check if the page is actually blank or not.
		 *
		 * It is important to check the emptyness only on oob_len,
		 * which only covers the spare bytes because after a read with
		 * ECC enabled, the ECC bytes in the buffer have been set by the
		 * ECC engine, so they are not 0xFF.
		 */
		if (!data)
			data_len = 0;
		if (!oob)
			oob_len = 0;
		bf = nand_check_erased_ecc_chunk(data, data_len, NULL, 0,
						 oob, oob_len,
						 chip->ecc.strength);
		if (bf < 0) {
			mtd->ecc_stats.failed++;
			return;
		}
	}

	/* Check correctable error flag */
	if (ndsr & NDSR_CORERR) {
		writel_relaxed(ndsr, nfc->regs + NDSR);

		if (chip->ecc.algo == NAND_ECC_BCH)
			bf = NDSR_ERRCNT(ndsr);
		else
			bf = 1;
	}

	/*
	 * Derive max_bitflips either from the number of bitflips detected by
	 * the hardware ECC engine or by nand_check_erased_ecc_chunk().
	 */
	mtd->ecc_stats.corrected += bf;
	*max_bitflips = max_t(unsigned int, *max_bitflips, bf);
}

/* Reads with HW ECC */
static int marvell_nfc_hw_ecc_hmg_read_page(struct mtd_info *mtd,
					    struct nand_chip *chip,
					    u8 *buf, int oob_required,
					    int page)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	int max_bitflips = 0, ret = 0, i;
	u8 *data, *oob;
	struct marvell_nfc_op nfc_op = {
		.ndcb[0] = NDCB0_CMD_TYPE(TYPE_READ) |
			   NDCB0_CMD_XTYPE(XTYPE_MONOLITHIC_RW) |
			   NDCB0_ADDR_CYC(marvell_nand->addr_cyc) |
			   NDCB0_DBC |
			   NDCB0_CMD1(NAND_CMD_READ0) |
			   NDCB0_CMD2(NAND_CMD_READSTART),
		.ndcb[1] = NDCB1_ADDRS(page),
		.ndcb[2] = NDCB2_ADDR5(page),
	};

	/*
	 * With Hamming, OOB is not fully used (and thus not read entirely), not
	 * expected bytes could show up at the end of the OOB buffer if not
	 * explicitly erased.
	 */
	if (oob_required)
		memset(chip->oob_poi, 0xFF, mtd->oobsize);

	ret = marvell_nfc_prepare_cmd(chip);
	if (ret)
		return ret;

	marvell_nfc_hw_ecc_enable(chip);

	data = buf;
	oob = chip->oob_poi;

	/*
	 * Reading spare area is mandatory when using HW ECC or read operation
	 * will trigger uncorrectable ECC errors, but do not read ECC here.
	 */
	marvell_nfc_send_cmd(chip, &nfc_op);

	marvell_nfc_end_cmd(chip, NDSR_RDDREQ,
			    "RDDREQ while draining FIFO (data)");

	/* Read the data... */
	if (data)
		ioread32_rep(nfc->regs + NDDB, data, FIFO_REP(lt->data_bytes));
	else
		for (i = 0; i < FIFO_REP(lt->data_bytes); i++)
			ioread32_rep(nfc->regs + NDDB, nfc->buf,
				     FIFO_REP(FIFO_DEPTH));

	/* ...then the spare bytes */
	ioread32_rep(nfc->regs + NDDB, oob, FIFO_REP(lt->spare_bytes));

	marvell_nfc_hw_ecc_correct(chip, data, lt->data_bytes,
				   oob, lt->spare_bytes, &max_bitflips);

	marvell_nfc_hw_ecc_disable(chip);

	if (oob_required) {
		/* Read ECC bytes without ECC enabled */
		nand_read_page_op(chip, page,
				  lt->data_bytes + lt->spare_bytes,
				  oob + lt->spare_bytes, lt->ecc_bytes);
	}

	return max_bitflips;
}

static void marvell_nfc_hw_ecc_bch_read_chunk(struct nand_chip *chip, int chunk,
					      u8 *data, int data_len,
					      u8 *oob, int oob_len,
					      int oob_required, int page)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	int i, j;
	struct marvell_nfc_op nfc_op = {
		.ndcb[0] = NDCB0_CMD_TYPE(TYPE_READ) |
			   NDCB0_ADDR_CYC(marvell_nand->addr_cyc) |
			   NDCB0_LEN_OVRD,
		.ndcb[1] = NDCB1_ADDRS(page),
		.ndcb[2] = NDCB2_ADDR5(page),
	};

	/*
	 * Reading spare area is mandatory when using HW ECC or read operation
	 * will trigger uncorrectable ECC errors, but do not read ECC here.
	 */
	nfc_op.ndcb[3] = data_len + oob_len;

	if (marvell_nfc_prepare_cmd(chip))
		return;

	if (chunk == 0)
		nfc_op.ndcb[0] |= NDCB0_DBC |
				  NDCB0_CMD1(NAND_CMD_READ0) |
				  NDCB0_CMD2(NAND_CMD_READSTART);

	/*
	 * Trigger the naked read operation only on the last chunk.
	 * Otherwise, use monolithic read.
	 */
	if ((lt->last_chunk_cnt && chunk == lt->full_chunk_cnt) ||
	    (!lt->last_chunk_cnt && chunk == lt->full_chunk_cnt - 1))
		nfc_op.ndcb[0] |= NDCB0_CMD_XTYPE(XTYPE_LAST_NAKED_RW);
	else
		nfc_op.ndcb[0] |= NDCB0_CMD_XTYPE(XTYPE_MONOLITHIC_RW);

	marvell_nfc_send_cmd(chip, &nfc_op);

	/*
	 * According to the datasheet, when reading from NDDB
	 * with BCH enabled, after each 32 bytes reads, we
	 * have to make sure that the NDSR.RDDREQ bit is set.
	 *
	 * Drain the FIFO, 8 32-bit reads at a time, and skip
	 * the polling on the last read.
	 *
	 * Length is a multiple of 32 bytes, hence it is a multiple of 8 too.
	 *
	 */
	for (i = 0; i < data_len; i += FIFO_DEPTH * BCH_SEQ_READS) {
		marvell_nfc_end_cmd(chip, NDSR_RDDREQ,
				    "RDDREQ while draining FIFO (data)");
		if (data) {
			ioread32_rep(nfc->regs + NDDB, data,
				     FIFO_REP(FIFO_DEPTH) * BCH_SEQ_READS);
			data += FIFO_DEPTH * BCH_SEQ_READS;
		} else {
			for (j = 0; j < sizeof(u32); j++)
				ioread32_rep(nfc->regs + NDDB, nfc->buf,
					     FIFO_REP(FIFO_DEPTH));
		}
	}

	for (i = 0; i < oob_len; i += FIFO_DEPTH * BCH_SEQ_READS) {
		marvell_nfc_end_cmd(chip, NDSR_RDDREQ,
				    "RDDREQ while draining FIFO (OOB)");
		ioread32_rep(nfc->regs + NDDB, oob,
			     FIFO_REP(FIFO_DEPTH) * BCH_SEQ_READS);
		oob += FIFO_DEPTH * BCH_SEQ_READS;
	}
}

static int marvell_nfc_hw_ecc_bch_read_page(struct mtd_info *mtd,
					    struct nand_chip *chip,
					    u8 *buf, int oob_required,
					    int page)
{
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	int nchunks = lt->full_chunk_cnt + lt->last_chunk_cnt;
	int max_bitflips = 0;
	u8 *data, *oob;
	int chunk, data_len, oob_len, ecc_len;
	/* Following sizes are used to read the ECC bytes after ECC operation */
	int fixed_oob_size = lt->spare_bytes + lt->ecc_bytes;
	int fixed_chunk_size = lt->data_bytes + fixed_oob_size;

	/*
	 * With BCH, OOB is not fully used (and thus not read entirely), not
	 * expected bytes could show up at the end of the OOB buffer if not
	 * explicitly erased.
	 */
	if (oob_required)
		memset(chip->oob_poi, 0xFF, mtd->oobsize);

	marvell_nfc_hw_ecc_enable(chip);

	for (chunk = 0; chunk < nchunks; chunk++) {
		if (chunk == 0) {
			/* Init pointers to iterate through the chunks */
			if (buf)
				data = buf;
			else
				data = NULL;
			oob = chip->oob_poi;
		} else {
			/* Update pointers */
			if (data)
				data += lt->data_bytes;
			oob += (lt->spare_bytes + lt->ecc_bytes + 2);
		}

		/* Update length */
		if (chunk < lt->full_chunk_cnt) {
			data_len = lt->data_bytes;
			oob_len = lt->spare_bytes;
			ecc_len = lt->ecc_bytes;
		} else {
			data_len = lt->last_data_bytes;
			oob_len = lt->last_spare_bytes;
			ecc_len = lt->last_ecc_bytes;
		}

		/* Read the chunk and detect number of bitflips */
		marvell_nfc_hw_ecc_bch_read_chunk(chip, chunk, data, data_len,
						  oob, oob_len, oob_required,
						  page);

		marvell_nfc_hw_ecc_correct(chip, data, data_len,
					   oob, oob_len, &max_bitflips);
	}

	marvell_nfc_hw_ecc_disable(chip);

	if (!oob_required)
		return max_bitflips;

	/* Read ECC bytes without ECC enabled */
	for (chunk = 0; chunk < lt->full_chunk_cnt; chunk++)
		read_page_data(chip, page,
			       ((chunk + 1) * (fixed_chunk_size)) -
			       lt->ecc_bytes,
			       chip->oob_poi + ((chunk + 1) *
						(fixed_oob_size + 2) -
						(lt->ecc_bytes + 2)),
			       lt->ecc_bytes);

	if (lt->last_chunk_cnt)
		read_page_data(chip, page,
			       (lt->full_chunk_cnt * fixed_chunk_size) +
			       lt->last_data_bytes + lt->last_spare_bytes,
			       chip->oob_poi + (lt->full_chunk_cnt *
						(fixed_oob_size + 2)) +
			       lt->last_spare_bytes,
			       lt->last_ecc_bytes);

	return max_bitflips;
}

static int marvell_nfc_hw_ecc_read_oob(struct mtd_info *mtd,
				       struct nand_chip *chip, int page)
{
	return chip->ecc.read_page(mtd, chip, NULL, true, page);
}

/* Raw reads with HW ECC */
static int marvell_nfc_hw_ecc_read_page_raw(struct mtd_info *mtd,
					    struct nand_chip *chip, u8 *buf,
					    int oob_required, int page)
{
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	int chunk_size = lt->data_bytes + lt->spare_bytes + lt->ecc_bytes;
	u8 *oob = chip->oob_poi;
	int chunk;

	if (oob_required)
		memset(chip->oob_poi, 0xFF, mtd->oobsize);

	for (chunk = 0; chunk < lt->full_chunk_cnt; chunk++) {
		read_page_data(chip, page, chunk * chunk_size, buf,
			       lt->data_bytes);
		buf += lt->data_bytes;

		if (oob_required) {
			nand_read_data_op(chip, oob, lt->spare_bytes +
					  lt->ecc_bytes, false);
			/* Pad user data with 2 bytes when using BCH (30B) */
			oob += lt->spare_bytes + lt->ecc_bytes + 2;
		}
	}

	if (!lt->last_chunk_cnt)
		return 0;

	read_page_data(chip, page, chunk * chunk_size, buf,
		       lt->last_data_bytes);
	if (oob_required)
		nand_read_data_op(chip, oob, lt->last_spare_bytes +
				  lt->last_ecc_bytes, false);

	return 0;
}

static int marvell_nfc_hw_ecc_read_oob_raw(struct mtd_info *mtd,
					   struct nand_chip *chip, int page)
{
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	u8 *oob = chip->oob_poi;
	int chunk_size = lt->data_bytes + lt->spare_bytes + lt->ecc_bytes;
	int chunk;

	for (chunk = 0; chunk < lt->full_chunk_cnt; chunk++) {
		/* Move NAND pointer to the next chunk of OOB data */
		read_page_data(chip, page,
			       chunk * chunk_size + lt->data_bytes,
			       oob, lt->spare_bytes + lt->ecc_bytes);
		/* Pad user data with 2 bytes when using BCH (30B) */
		oob += lt->spare_bytes + lt->ecc_bytes + 2;
	}

	if (lt->last_chunk_cnt)
		nand_read_data_op(chip, oob,
				  lt->last_spare_bytes + lt->last_ecc_bytes,
				  false);

	return 0;
}

/* Writes with HW ECC */
static int marvell_nfc_hw_ecc_hmg_write_page(struct mtd_info *mtd,
					     struct nand_chip *chip,
					     const u8 *buf,
					     int oob_required, int page)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	const u8 *data = buf, *oob = chip->oob_poi;
	int ret, i;
	struct marvell_nfc_op nfc_op = {
		.ndcb[0] = NDCB0_CMD_TYPE(TYPE_WRITE) |
			   NDCB0_CMD_XTYPE(XTYPE_MONOLITHIC_RW) |
			   NDCB0_ADDR_CYC(marvell_nand->addr_cyc) |
			   NDCB0_CMD1(NAND_CMD_SEQIN),
		.ndcb[1] = NDCB1_ADDRS(page),
		.ndcb[2] = NDCB2_ADDR5(page),
	};

	ret = marvell_nfc_prepare_cmd(chip);
	if (ret)
		return ret;

	marvell_nfc_hw_ecc_enable(chip);

		marvell_nfc_send_cmd(chip, &nfc_op);

	marvell_nfc_end_cmd(chip, NDSR_WRDREQ,
			    "WRDREQ while loading FIFO (data)");

	/* Write the data. If buf is empty, write empty bytes (0xFF) */
	if (data) {
		iowrite32_rep(nfc->regs + NDDB, data, FIFO_REP(lt->data_bytes));
	} else {
		data = nfc->buf;
		memset(nfc->buf, 0xFF, FIFO_DEPTH);
		for (i = 0; i < FIFO_REP(lt->data_bytes) / FIFO_REP(FIFO_DEPTH); i++)
			iowrite32_rep(nfc->regs + NDDB, data, FIFO_REP(FIFO_DEPTH));
	}

	/* Then write the OOB data */
	iowrite32_rep(nfc->regs + NDDB, oob, FIFO_REP(lt->spare_bytes));

	ret = marvell_nfc_wait_op(chip,
				  chip->data_interface.timings.sdr.tPROG_max);

	marvell_nfc_hw_ecc_disable(chip);

	if (ret & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
}

static void marvell_nfc_hw_ecc_bch_write_chunk(struct nand_chip *chip,
					       int chunk, const u8 *data,
					       u8 *oob, int oob_required,
					       int page)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	int data_len, oob_len, i;
	struct marvell_nfc_op nfc_op = {
		.ndcb[0] = NDCB0_CMD_TYPE(TYPE_WRITE) | NDCB0_LEN_OVRD,
	};

	/* OOB area to write is only spare area when using HW ECC */
	if (chunk < lt->full_chunk_cnt) {
		data_len = lt->data_bytes;
		oob_len = lt->spare_bytes;
	} else {
		data_len = lt->last_data_bytes;
		oob_len = lt->last_spare_bytes;
	}

	nfc_op.ndcb[3] = data_len + oob_len;

	/*
	 * First operation dispatches the CMD_SEQIN command, issue the address
	 * cycles and asks for the first chunk of data.
	 * Last operation dispatches the PAGEPROG command and also asks for the
	 * last chunk of data.
	 * All operations in the middle (if any) will issue a naked write and
	 * also ask for data.
	 */
	if (chunk == 0) {
		nfc_op.ndcb[0] |= NDCB0_CMD_XTYPE(XTYPE_WRITE_DISPATCH) |
				  NDCB0_ADDR_CYC(marvell_nand->addr_cyc) |
				  NDCB0_CMD1(NAND_CMD_SEQIN);
		nfc_op.ndcb[1] |= NDCB1_ADDRS(page);
		nfc_op.ndcb[2] |= NDCB2_ADDR5(page);
	} else if (
		(lt->last_chunk_cnt && (chunk == lt->full_chunk_cnt)) ||
		(!lt->last_chunk_cnt &&	(chunk == lt->full_chunk_cnt - 1))) {
		nfc_op.ndcb[0] |= NDCB0_CMD2(NAND_CMD_PAGEPROG) | NDCB0_DBC |
				  NDCB0_CMD_XTYPE(XTYPE_LAST_NAKED_RW);
	} else {
		nfc_op.ndcb[0] |= NDCB0_CMD_XTYPE(XTYPE_NAKED_RW);
	}

	/*
	 * If this is the first chunk, the previous command also embedded
	 * the write operation, no need to repeat it.
	 */
	if (marvell_nfc_prepare_cmd(chip))
		return;

	marvell_nfc_send_cmd(chip, &nfc_op);

	marvell_nfc_end_cmd(chip, NDSR_WRDREQ,
			    "WRDREQ while loading FIFO (data)");

	/* Effectively write the data to the data buffer */
	if (data) {
		data += chunk * lt->data_bytes;
		iowrite32_rep(nfc->regs + NDDB, data, FIFO_REP(data_len));
	} else {
		memset(nfc->buf, 0xFF, FIFO_DEPTH);
		data = nfc->buf;
		for (i = 0; i < FIFO_REP(data_len) / FIFO_REP(FIFO_DEPTH); i++)
			iowrite32_rep(nfc->regs + NDDB, data,
				      FIFO_REP(FIFO_DEPTH));
	}

	/* Pad user data with 2 bytes when using BCH (30B) */
	oob += chunk * lt->spare_bytes;
	iowrite32_rep(nfc->regs + NDDB, oob, FIFO_REP(oob_len));
}

static int marvell_nfc_hw_ecc_bch_write_page(struct mtd_info *mtd,
					     struct nand_chip *chip,
					     const u8 *buf,
					     int oob_required, int page)
{
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	int nchunks = lt->full_chunk_cnt + lt->last_chunk_cnt;
	int chunk, ret;

	marvell_nfc_hw_ecc_enable(chip);

	for (chunk = 0; chunk < nchunks; chunk++) {
		marvell_nfc_hw_ecc_bch_write_chunk(chip, chunk, buf,
						   chip->oob_poi,
						   oob_required, page);
		/*
		 * Waiting only for CMDD or PAGED is not enough, ECC are
		 * partially written. No flag is set once the operation is
		 * really finished but the ND_RUN bit is cleared, so wait for it
		 * before stepping into the next command.
		 */
		marvell_nfc_wait_ndrun(chip);
	}

	ret = marvell_nfc_wait_op(chip,
				  chip->data_interface.timings.sdr.tPROG_max);

	marvell_nfc_hw_ecc_disable(chip);

	if (ret & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
}

static int marvell_nfc_hw_ecc_write_oob(struct mtd_info *mtd,
					struct nand_chip *chip, int page)
{
	return chip->ecc.write_page(mtd, chip, NULL, true, page);
}

/* Raw writes with HW ECC */
static int marvell_nfc_hw_ecc_write_page_raw(struct mtd_info *mtd,
					     struct nand_chip *chip,
					     const u8 *buf,
					     int oob_required, int page)
{
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	int nchunks = lt->full_chunk_cnt + lt->last_chunk_cnt;
	int oob_size = lt->spare_bytes + lt->ecc_bytes;
	int last_oob_size = lt->last_spare_bytes + lt->last_ecc_bytes;
	int chunk;

	nand_prog_page_begin_op(chip, page, 0, NULL, 0);

	for (chunk = 0; chunk < nchunks; chunk++) {
		/*
		 * OOB are not 8-bytes aligned anyway so change the column
		 * at each cycle
		 */
		nand_change_write_column_op(chip, chunk * (lt->data_bytes +
							   oob_size),
					    NULL, 0, false);

		if (chunk < lt->full_chunk_cnt)
			nand_write_data_op(chip, buf + (chunk * lt->data_bytes),
					   lt->data_bytes, false);
		else
			nand_write_data_op(chip, buf + (chunk * lt->data_bytes),
					   lt->last_data_bytes, false);

		if (!oob_required)
			continue;

		if (chunk < lt->full_chunk_cnt)
			nand_write_data_op(chip, chip->oob_poi +
					   (chunk * (oob_size + 2)),
					   oob_size, false);
		else
			nand_write_data_op(chip, chip->oob_poi +
					   (chunk * (oob_size + 2)),
					   last_oob_size, false);
	}

	return nand_prog_page_end_op(chip);
}

static int marvell_nfc_hw_ecc_write_oob_raw(struct mtd_info *mtd,
					    struct nand_chip *chip, int page)
{
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	int chunk_size = lt->data_bytes + lt->spare_bytes + lt->ecc_bytes;
	int nchunks = lt->full_chunk_cnt + lt->last_chunk_cnt;
	int oob_size = lt->spare_bytes + lt->ecc_bytes;
	int last_oob_size = lt->last_spare_bytes + lt->last_ecc_bytes;
	int chunk;

	nand_prog_page_begin_op(chip, page, 0, NULL, 0);

	for (chunk = 0; chunk < nchunks; chunk++) {
		nand_change_write_column_op(chip, lt->data_bytes +
					    (chunk * chunk_size), NULL, 0,
					    false);

		if (chunk < lt->full_chunk_cnt)
			nand_write_data_op(chip, chip->oob_poi +
					   (chunk * (oob_size + 2)),
					   oob_size, false);
		else
			nand_write_data_op(chip, chip->oob_poi +
					   (chunk * (oob_size + 2)),
					   last_oob_size, false);
	}

	return nand_prog_page_end_op(chip);
}

/* NAND framework ->exec_op() hooks and related helpers */
static void marvell_nfc_parse_instructions(const struct nand_subop *subop,
					   struct marvell_nfc_op *nfc_op)
{
	const struct nand_op_instr *instr = NULL;
	bool first_cmd = true;
	unsigned int op_id;
	int i;

	/* Reset the input structure as most of its fields will be OR'ed */
	memset(nfc_op, 0, sizeof(struct marvell_nfc_op));

	for (op_id = 0; op_id < subop->ninstrs; op_id++) {
		unsigned int offset, naddrs;
		const u8 *addrs;
		int len = nand_subop_get_data_len(subop, op_id);

		instr = &subop->instrs[op_id];

		switch (instr->type) {
		case NAND_OP_CMD_INSTR:
			if (first_cmd)
				nfc_op->ndcb[0] |=
					NDCB0_CMD1(instr->ctx.cmd.opcode);
			else
				nfc_op->ndcb[0] |=
					NDCB0_CMD2(instr->ctx.cmd.opcode) |
					NDCB0_DBC;

			nfc_op->cle_ale_delay_ns = instr->delay_ns;
			first_cmd = false;
			break;

		case NAND_OP_ADDR_INSTR:
			offset = nand_subop_get_addr_start_off(subop, op_id);
			naddrs = nand_subop_get_num_addr_cyc(subop, op_id);
			addrs = &instr->ctx.addr.addrs[offset];

			nfc_op->ndcb[0] |= NDCB0_ADDR_CYC(naddrs);

			for (i = 0; i < min_t(unsigned int, 4, naddrs); i++)
				nfc_op->ndcb[1] |= addrs[i] << (8 * i);

			if (naddrs >= 5)
				nfc_op->ndcb[2] |= NDCB2_ADDR5(addrs[5]);
			if (naddrs >= 6)
				nfc_op->ndcb[3] |= NDCB3_ADDR6(addrs[6]);
			if (naddrs == 7)
				nfc_op->ndcb[3] |= NDCB3_ADDR7(addrs[7]);

			nfc_op->cle_ale_delay_ns = instr->delay_ns;
			break;

		case NAND_OP_DATA_IN_INSTR:
			nfc_op->data_instr = instr;
			nfc_op->data_instr_idx = op_id;
			nfc_op->ndcb[0] |=
				NDCB0_CMD_TYPE(TYPE_READ) |
				NDCB0_CMD_XTYPE(XTYPE_MONOLITHIC_RW) |
				NDCB0_LEN_OVRD;
			nfc_op->ndcb[3] |= round_up(len, FIFO_DEPTH);
			nfc_op->data_delay_ns = instr->delay_ns;
			break;

		case NAND_OP_DATA_OUT_INSTR:
			nfc_op->data_instr = instr;
			nfc_op->data_instr_idx = op_id;
			nfc_op->ndcb[0] |=
				NDCB0_CMD_TYPE(TYPE_WRITE) |
				NDCB0_CMD_XTYPE(XTYPE_MONOLITHIC_RW) |
				NDCB0_LEN_OVRD;
			nfc_op->ndcb[3] |= round_up(len, FIFO_DEPTH);
			nfc_op->data_delay_ns = instr->delay_ns;
			break;

		case NAND_OP_WAITRDY_INSTR:
			nfc_op->rdy_timeout_ms = instr->ctx.waitrdy.timeout_ms;
			nfc_op->rdy_delay_ns = instr->delay_ns;
			break;
		}
	}
}

static void marvell_nfc_xfer_data(struct nand_chip *chip,
				  const struct nand_subop *subop,
				  struct marvell_nfc_op *nfc_op)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	unsigned int op_id = nfc_op->data_instr_idx;
	int len = nand_subop_get_data_len(subop, op_id);
	int offset = nand_subop_get_data_start_off(subop, op_id);
	int last_len = len % FIFO_DEPTH;
	int last_full_offset = round_down(len, FIFO_DEPTH);
	const struct nand_op_instr *instr = nfc_op->data_instr;
	int i;

	if (instr->ctx.data.force_8bit)
		marvell_nfc_force_byte_access(chip, true);

	if (instr->type == NAND_OP_DATA_IN_INSTR) {
		u8 *in = instr->ctx.data.buf.in + offset;

		for (i = 0; i < last_full_offset; i += FIFO_DEPTH)
			ioread32_rep(nfc->regs + NDDB, in + i,
				     FIFO_REP(FIFO_DEPTH));

		if (last_len) {
			ioread32_rep(nfc->regs + NDDB, nfc->buf,
				     FIFO_REP(FIFO_DEPTH));
			memcpy(in + last_full_offset, nfc->buf, last_len);
		}
	} else {
		const u8 *out = instr->ctx.data.buf.out + offset;

		for (i = 0; i < last_full_offset; i += FIFO_DEPTH)
			iowrite32_rep(nfc->regs + NDDB, out + i,
				      FIFO_REP(FIFO_DEPTH));

		if (last_len) {
			memcpy(nfc->buf, out + last_full_offset, last_len);
			iowrite32_rep(nfc->regs + NDDB, nfc->buf,
				      FIFO_REP(FIFO_DEPTH));
		}
	}

	if (instr->ctx.data.force_8bit)
		marvell_nfc_force_byte_access(chip, false);
}

static int marvell_nfc_monolithic_access_exec(struct nand_chip *chip,
					      const struct nand_subop *subop)
{
	struct marvell_nfc_op nfc_op;
	bool reading;
	int ret;

	marvell_nfc_parse_instructions(subop, &nfc_op);
	reading = nfc_op.data_instr->type == NAND_OP_DATA_IN_INSTR;

	ret = marvell_nfc_prepare_cmd(chip);
	if (ret)
		return ret;

	marvell_nfc_send_cmd(chip, &nfc_op);
	ret = marvell_nfc_end_cmd(chip, NDSR_RDDREQ | NDSR_WRDREQ,
				  "RDDREQ/WRDREQ while draining raw data");
	cond_delay(nfc_op.cle_ale_delay_ns);

	if (reading) {
		if (nfc_op.rdy_timeout_ms)
			ret = marvell_nfc_wait_op(chip, nfc_op.rdy_timeout_ms);
		cond_delay(nfc_op.rdy_delay_ns);
	}

	if (ret)
		return ret;

	marvell_nfc_xfer_data(chip, subop, &nfc_op);
	ret = marvell_nfc_wait_cmdd(chip);
	cond_delay(nfc_op.data_delay_ns);

	if (!reading) {
		if (nfc_op.rdy_timeout_ms)
			ret = marvell_nfc_wait_op(chip, nfc_op.rdy_timeout_ms);
		cond_delay(nfc_op.rdy_delay_ns);
	}

	return ret;
}

static int marvell_nfc_reset_cmd_type_exec(struct nand_chip *chip,
					   const struct nand_subop *subop)
{
	struct marvell_nfc_op nfc_op;
	int ret;

	marvell_nfc_parse_instructions(subop, &nfc_op);
	nfc_op.ndcb[0] |= NDCB0_CMD_TYPE(TYPE_RESET);

	ret = marvell_nfc_prepare_cmd(chip);
	if (ret)
		return ret;

	marvell_nfc_send_cmd(chip, &nfc_op);
	ret = marvell_nfc_wait_ndrun(chip);
	cond_delay(nfc_op.cle_ale_delay_ns);

	if (nfc_op.rdy_timeout_ms)
		ret = marvell_nfc_wait_op(chip, nfc_op.rdy_timeout_ms);
	cond_delay(nfc_op.rdy_delay_ns);

	return ret;
}

static int marvell_nfc_erase_cmd_type_exec(struct nand_chip *chip,
					   const struct nand_subop *subop)
{
	struct marvell_nfc_op nfc_op;
	int ret;

	marvell_nfc_parse_instructions(subop, &nfc_op);
	nfc_op.ndcb[0] |= NDCB0_CMD_TYPE(TYPE_ERASE);

	ret = marvell_nfc_prepare_cmd(chip);
	if (ret)
		return ret;

	marvell_nfc_send_cmd(chip, &nfc_op);
	ret = marvell_nfc_wait_ndrun(chip);
	cond_delay(nfc_op.cle_ale_delay_ns);

	if (nfc_op.rdy_timeout_ms)
		ret = marvell_nfc_wait_op(chip, nfc_op.rdy_timeout_ms);
	cond_delay(nfc_op.rdy_delay_ns);

	return ret;
}

static int marvell_nfc_naked_access_exec(struct nand_chip *chip,
					 const struct nand_subop *subop)
{
	struct marvell_nfc_op nfc_op;
	int ret;

	marvell_nfc_parse_instructions(subop, &nfc_op);

	/*
	 * Naked access are different in that they need to be flagged as naked
	 * by the controller. Reset the controller registers fields that inform
	 * on the type and refill them according to the ongoing operation.
	 */
	nfc_op.ndcb[0] &= ~(NDCB0_CMD_TYPE(TYPE_MASK) |
			    NDCB0_CMD_XTYPE(XTYPE_MASK));
	switch (subop->instrs[0].type) {
	case NAND_OP_CMD_INSTR:
		nfc_op.ndcb[0] |= NDCB0_CMD_TYPE(TYPE_NAKED_CMD);
		break;
	case NAND_OP_ADDR_INSTR:
		nfc_op.ndcb[0] |= NDCB0_CMD_TYPE(TYPE_NAKED_ADDR);
		break;
	case NAND_OP_DATA_IN_INSTR:
		nfc_op.ndcb[0] |= NDCB0_CMD_TYPE(TYPE_READ) |
				  NDCB0_CMD_XTYPE(XTYPE_LAST_NAKED_RW);
		break;
	case NAND_OP_DATA_OUT_INSTR:
		nfc_op.ndcb[0] |= NDCB0_CMD_TYPE(TYPE_WRITE) |
				  NDCB0_CMD_XTYPE(XTYPE_LAST_NAKED_RW);
		break;
	default:
		/* This should never happen */
		break;
	}

	ret = marvell_nfc_prepare_cmd(chip);
	if (ret)
		return ret;

	marvell_nfc_send_cmd(chip, &nfc_op);

	if (!nfc_op.data_instr) {
		ret = marvell_nfc_wait_ndrun(chip);
		cond_delay(nfc_op.cle_ale_delay_ns);
		return ret;
	}

	ret = marvell_nfc_end_cmd(chip, NDSR_RDDREQ | NDSR_WRDREQ,
				  "RDDREQ/WRDREQ while draining raw data");
	if (ret)
		return ret;

	marvell_nfc_xfer_data(chip, subop, &nfc_op);
	ret = marvell_nfc_wait_cmdd(chip);
	cond_delay(nfc_op.data_delay_ns);

	return ret;
}

static int marvell_nfc_naked_waitrdy_exec(struct nand_chip *chip,
					  const struct nand_subop *subop)
{
	struct marvell_nfc_op nfc_op;
	int ret;

	marvell_nfc_parse_instructions(subop, &nfc_op);

	ret = marvell_nfc_wait_op(chip, nfc_op.rdy_timeout_ms);
	cond_delay(nfc_op.rdy_delay_ns);

	return ret;
}

static const struct nand_op_parser marvell_nfc_op_parser = NAND_OP_PARSER(
	/* Monolithic read/write */
	NAND_OP_PARSER_PATTERN(
		marvell_nfc_monolithic_access_exec,
		NAND_OP_PARSER_PAT_CMD_ELEM(false),
		NAND_OP_PARSER_PAT_ADDR_ELEM(true, MAX_ADDRESS_CYC),
		NAND_OP_PARSER_PAT_CMD_ELEM(true),
		NAND_OP_PARSER_PAT_WAITRDY_ELEM(true),
		NAND_OP_PARSER_PAT_DATA_IN_ELEM(false, MAX_CHUNK_SIZE)),
	NAND_OP_PARSER_PATTERN(
		marvell_nfc_monolithic_access_exec,
		NAND_OP_PARSER_PAT_CMD_ELEM(false),
		NAND_OP_PARSER_PAT_ADDR_ELEM(false, MAX_ADDRESS_CYC),
		NAND_OP_PARSER_PAT_DATA_OUT_ELEM(false, MAX_CHUNK_SIZE),
		NAND_OP_PARSER_PAT_CMD_ELEM(true),
		NAND_OP_PARSER_PAT_WAITRDY_ELEM(true)),
	/* Isolated commands (reset, erase, begin prog,...) */
	NAND_OP_PARSER_PATTERN(
		marvell_nfc_erase_cmd_type_exec,
		NAND_OP_PARSER_PAT_CMD_ELEM(false),
		NAND_OP_PARSER_PAT_ADDR_ELEM(false, MAX_ADDRESS_CYC),
		NAND_OP_PARSER_PAT_CMD_ELEM(false),
		NAND_OP_PARSER_PAT_WAITRDY_ELEM(true)),
	NAND_OP_PARSER_PATTERN(
		marvell_nfc_reset_cmd_type_exec,
		NAND_OP_PARSER_PAT_CMD_ELEM(false),
		NAND_OP_PARSER_PAT_WAITRDY_ELEM(false)),
	/* Naked commands */
	NAND_OP_PARSER_PATTERN(
		marvell_nfc_naked_access_exec,
		NAND_OP_PARSER_PAT_CMD_ELEM(false)),
	NAND_OP_PARSER_PATTERN(
		marvell_nfc_naked_access_exec,
		NAND_OP_PARSER_PAT_ADDR_ELEM(false, MAX_ADDRESS_CYC)),
	NAND_OP_PARSER_PATTERN(
		marvell_nfc_naked_access_exec,
		NAND_OP_PARSER_PAT_DATA_IN_ELEM(false, MAX_CHUNK_SIZE)),
	NAND_OP_PARSER_PATTERN(
		marvell_nfc_naked_access_exec,
		NAND_OP_PARSER_PAT_DATA_OUT_ELEM(false, MAX_CHUNK_SIZE)),
	NAND_OP_PARSER_PATTERN(
		marvell_nfc_naked_waitrdy_exec,
		NAND_OP_PARSER_PAT_WAITRDY_ELEM(false)),
	);

static int marvell_nfc_exec_op(struct nand_chip *chip,
			       const struct nand_operation *op,
			       bool check_only)
{
	return nand_op_parser_exec_op(chip, &marvell_nfc_op_parser,
				      op, check_only);
}

/*
 * HW ECC layouts, identical to old pxa3xx_nand driver,
 * to be fully backward compatible.
 */
static int marvell_nand_ooblayout_ecc(struct mtd_info *mtd, int section,
				      struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	int nchunks = lt->full_chunk_cnt;

	if (section >= nchunks)
		return -ERANGE;

	oobregion->offset = ((lt->spare_bytes + lt->ecc_bytes) * section) +
		lt->spare_bytes;
	oobregion->length = lt->ecc_bytes;

	return 0;
}

static int marvell_nand_ooblayout_free(struct mtd_info *mtd, int section,
				       struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	const struct marvell_hw_ecc_layout *lt = to_marvell_nand(chip)->layout;
	int nchunks = lt->full_chunk_cnt;

	if (section >= nchunks)
		return -ERANGE;

	if (!lt->spare_bytes)
		return 0;

	oobregion->offset = section * (lt->spare_bytes + lt->ecc_bytes);
	oobregion->length = lt->spare_bytes;
	if (!section) {
		/*
		 * Bootrom looks in bytes 0 & 5 for bad blocks for the
		 * 4KB page / 4bit BCH combination.
		 */
		if (mtd->writesize == 4096 && lt->data_bytes == 2048) {
			oobregion->offset += 6;
			oobregion->length -= 6;
		} else {
			oobregion->offset += 2;
			oobregion->length -= 2;
		}
	}

	return 0;
}

static const struct mtd_ooblayout_ops marvell_nand_ooblayout_ops = {
	.ecc = marvell_nand_ooblayout_ecc,
	.free = marvell_nand_ooblayout_free,
};

static int marvell_nand_hw_ecc_ctrl_init(struct mtd_info *mtd,
					 struct nand_ecc_ctrl *ecc)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	const struct marvell_hw_ecc_layout *l;
	int i;

	to_marvell_nand(chip)->layout = NULL;
	for (i = 0; i < ARRAY_SIZE(marvell_nfc_layouts); i++) {
		l = &marvell_nfc_layouts[i];
		if (mtd->writesize == l->writesize &&
		    ecc->size == l->chunk && ecc->strength == l->strength) {
			to_marvell_nand(chip)->layout = l;
			break;
		}
	}

	if (!to_marvell_nand(chip)->layout) {
		dev_err(nfc->dev,
			"ECC strength %d at page size %d is not supported\n",
			ecc->strength, mtd->writesize);
		return -ENOTSUPP;
	}

	mtd_set_ooblayout(mtd, &marvell_nand_ooblayout_ops);
	ecc->steps = l->full_chunk_cnt + l->last_chunk_cnt;
	ecc->size = l->data_bytes;

	if (ecc->strength == 1) {
		chip->ecc.algo = NAND_ECC_HAMMING;
		ecc->read_page = marvell_nfc_hw_ecc_hmg_read_page;
		ecc->write_page = marvell_nfc_hw_ecc_hmg_write_page;
	} else {
		chip->ecc.algo = NAND_ECC_BCH;
		ecc->read_page = marvell_nfc_hw_ecc_bch_read_page;
		ecc->write_page = marvell_nfc_hw_ecc_bch_write_page;
		ecc->strength = 16;
	}

	ecc->read_oob = marvell_nfc_hw_ecc_read_oob;
	ecc->write_oob = marvell_nfc_hw_ecc_write_oob;

	ecc->read_page_raw = marvell_nfc_hw_ecc_read_page_raw;
	ecc->write_page_raw = marvell_nfc_hw_ecc_write_page_raw;
	ecc->read_oob_raw = marvell_nfc_hw_ecc_read_oob_raw;
	ecc->write_oob_raw = marvell_nfc_hw_ecc_write_oob_raw;

	return 0;
}

static int marvell_nand_ecc_init(struct mtd_info *mtd,
				 struct nand_ecc_ctrl *ecc)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	int ret;

	if (!ecc->size)
		ecc->size = chip->ecc_step_ds;

	if (!ecc->strength)
		ecc->strength = chip->ecc_strength_ds;

	if (!ecc->size || !ecc->strength)
		return -EINVAL;

	switch (ecc->mode) {
	case NAND_ECC_HW:
		ret = marvell_nand_hw_ecc_ctrl_init(mtd, ecc);
		if (ret)
			return ret;
		break;
	case NAND_ECC_NONE:
		chip->ecc.algo = 0;
	case NAND_ECC_SOFT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static u8 bbt_pattern[] = {'M', 'V', 'B', 'b', 't', '0' };
static u8 bbt_mirror_pattern[] = {'1', 't', 'b', 'B', 'V', 'M' };

static struct nand_bbt_descr bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
		   NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	8,
	.len = 6,
	.veroffs = 14,
	.maxblocks = 8,	/* Last 8 blocks in each chip */
	.pattern = bbt_pattern
};

static struct nand_bbt_descr bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
		   NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	8,
	.len = 6,
	.veroffs = 14,
	.maxblocks = 8,	/* Last 8 blocks in each chip */
	.pattern = bbt_mirror_pattern
};

static int marvell_nfc_setup_data_interface(struct mtd_info *mtd, int chipnr,
					    const struct nand_data_interface
					    *conf)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	unsigned int period_ns = 1000000000 / clk_get_rate(nfc->ecc_clk) * 2;
	const struct nand_sdr_timings *sdr;
	struct marvell_nfc_timings nfc_tmg;
	int read_delay;

	sdr = nand_get_sdr_timings(conf);
	if (IS_ERR(sdr))
		return PTR_ERR(sdr);

	/*
	 * SDR timings are given in pico-seconds while NFC timings must be
	 * expressed in NAND controller clock cycles, which is half of the
	 * frequency of the accessible ECC clock retrieved by clk_get_rate().
	 * This is not written anywhere in the datasheet but was observed
	 * with an oscilloscope.
	 *
	 * NFC datasheet gives equations from which thoses calculations
	 * are derived, they tend to be slightly more restrictives than the
	 * given core timings and may improve the overall speed.
	 */
	nfc_tmg.tRP = TO_CYCLES(DIV_ROUND_UP(sdr->tRC_min, 2), period_ns) - 1;
	nfc_tmg.tRH = nfc_tmg.tRP;
	nfc_tmg.tWP = TO_CYCLES(DIV_ROUND_UP(sdr->tWC_min, 2), period_ns) - 1;
	nfc_tmg.tWH = nfc_tmg.tWP;
	nfc_tmg.tCS = TO_CYCLES(sdr->tCS_min, period_ns);
	nfc_tmg.tCH = TO_CYCLES(sdr->tCH_min, period_ns) - 1;
	nfc_tmg.tADL = TO_CYCLES(sdr->tADL_min, period_ns);
	/*
	 * Read delay is the time of propagation from SoC pins to NFC internal
	 * logic. With non-EDO timings, this is MIN_RD_DEL_CNT clock cycles. In
	 * EDO mode, an additional delay of tRH must be taken into account so
	 * the data is sampled on the falling edge instead of the rising edge.
	 */
	read_delay = sdr->tRC_min >= 30000 ?
		MIN_RD_DEL_CNT : MIN_RD_DEL_CNT + nfc_tmg.tRH;

	nfc_tmg.tAR = TO_CYCLES(sdr->tAR_min, period_ns);
	/*
	 * tWHR and tRHW are supposed to be read to write delays (and vice
	 * versa) but in some cases, ie. when doing a change column, they must
	 * be greater than that to be sure tCCS delay is respected.
	 */
	nfc_tmg.tWHR = TO_CYCLES(max_t(int, sdr->tWHR_min, sdr->tCCS_min),
				 period_ns) - 2,
	nfc_tmg.tRHW = TO_CYCLES(max_t(int, sdr->tRHW_min, sdr->tCCS_min),
				 period_ns);

	/* Use WAIT_MODE (wait for RB line) instead of only relying on delays */
	nfc_tmg.tR = TO_CYCLES(sdr->tWB_max, period_ns);

	if (chipnr < 0)
		return 0;

	marvell_nand->ndtr0 =
		NDTR0_TRP(nfc_tmg.tRP) |
		NDTR0_TRH(nfc_tmg.tRH) |
		NDTR0_ETRP(nfc_tmg.tRP) |
		NDTR0_TWP(nfc_tmg.tWP) |
		NDTR0_TWH(nfc_tmg.tWH) |
		NDTR0_TCS(nfc_tmg.tCS) |
		NDTR0_TCH(nfc_tmg.tCH) |
		NDTR0_RD_CNT_DEL(read_delay) |
		NDTR0_SELCNTR |
		NDTR0_TADL(nfc_tmg.tADL);

	marvell_nand->ndtr1 =
		NDTR1_TAR(nfc_tmg.tAR) |
		NDTR1_TWHR(nfc_tmg.tWHR) |
		NDTR1_TRHW(nfc_tmg.tRHW) |
		NDTR1_WAIT_MODE |
		NDTR1_TR(nfc_tmg.tR);

	writel_relaxed(marvell_nand->ndtr0, nfc->regs + NDTR0);
	writel_relaxed(marvell_nand->ndtr1, nfc->regs + NDTR1);

	return 0;
}

static int marvell_nand_chip_init(struct device *dev, struct marvell_nfc *nfc,
				  struct device_node *np)
{
	struct pxa3xx_nand_platform_data *pdata = dev_get_platdata(dev);
	struct marvell_nand_chip *marvell_nand;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	int nsels, ret, i;
	u32 cs, rb;

	/*
	 * The legacy "num-cs" property indicates the number of CS on the only
	 * chip connected to the controller (legacy bindings does not support
	 * more than one chip). CS are only incremented one by one while the RB
	 * pin is always the #0.
	 *
	 * When not using legacy bindings, a couple of "reg" and "marvell,rb"
	 * properties must be filled. For each chip, expressed as a subnode,
	 * "reg" points to the CS lines and "marvell,rb" to the RB line.
	 */
	if (pdata) {
		nsels = 1;
	} else if (nfc->caps->legacy_of_bindings) {
		if (!of_get_property(np, "num-cs", &nsels)) {
			dev_err(dev, "missing num-cs property\n");
			return -EINVAL;
		}
	} else {
		if (!of_get_property(np, "reg", &nsels)) {
			dev_err(dev, "missing reg property\n");
			return -EINVAL;
		}
	}

	nsels /= sizeof(u32);
	if (!nsels) {
		dev_err(dev, "invalid reg property size\n");
		return -EINVAL;
	}

	/* Alloc the nand chip structure */
	marvell_nand = devm_kzalloc(dev, sizeof(*marvell_nand) +
				    (nsels *
				     sizeof(struct marvell_nand_chip_sel)),
				    GFP_KERNEL);
	if (!marvell_nand) {
		dev_err(dev, "could not allocate chip structure\n");
		return -ENOMEM;
	}

	marvell_nand->nsels = nsels;
	marvell_nand->selected_die = -1;

	for (i = 0; i < nsels; i++) {
		if (pdata || nfc->caps->legacy_of_bindings) {
			/*
			 * Legacy bindings use the CS lines in natural
			 * order (0, 1, ...)
			 */
			cs = i;
		} else {
			/* Retrieve CS id */
			ret = of_property_read_u32_index(np, "reg", i, &cs);
			if (ret) {
				dev_err(dev, "could not retrieve reg property: %d\n",
					ret);
				return ret;
			}
		}

		if (cs >= nfc->caps->max_cs_nb) {
			dev_err(dev, "invalid reg value: %u (max CS = %d)\n",
				cs, nfc->caps->max_cs_nb);
			return -EINVAL;
		}

		if (test_and_set_bit(cs, &nfc->assigned_cs)) {
			dev_err(dev, "CS %d already assigned\n", cs);
			return -EINVAL;
		}

		/*
		 * The cs variable represents the chip select id, which must be
		 * converted in bit fields for NDCB0 and NDCB2 to select the
		 * right chip. Unfortunately, due to a lack of information on
		 * the subject and incoherent documentation, the user should not
		 * use CS1 and CS3 at all as asserting them is not supported in
		 * a reliable way (due to multiplexing inside ADDR5 field).
		 */
		marvell_nand->sels[i].cs = cs;
		switch (cs) {
		case 0:
		case 2:
			marvell_nand->sels[i].ndcb0_csel = 0;
			break;
		case 1:
		case 3:
			marvell_nand->sels[i].ndcb0_csel = NDCB0_CSEL;
			break;
		default:
			return -EINVAL;
		}

		/* Retrieve RB id */
		if (pdata || nfc->caps->legacy_of_bindings) {
			/* Legacy bindings always use RB #0 */
			rb = 0;
		} else {
			ret = of_property_read_u32_index(np, "marvell,rb", i,
							 &rb);
			if (ret) {
				dev_err(dev,
					"could not retrieve RB property: %d\n",
					ret);
				return ret;
			}
		}

		if (rb >= nfc->caps->max_rb_nb) {
			dev_err(dev, "invalid reg value: %u (max RB = %d)\n",
				rb, nfc->caps->max_rb_nb);
			return -EINVAL;
		}

		marvell_nand->sels[i].rb = rb;
	}

	chip = &marvell_nand->chip;
	chip->controller = &nfc->controller;
	nand_set_flash_node(chip, np);

	chip->exec_op = marvell_nfc_exec_op;
	chip->select_chip = marvell_nfc_select_chip;
	if ((pdata && !pdata->keep_config) ||
	    !of_property_read_bool(np, "marvell,nand-keep-config"))
		chip->setup_data_interface = marvell_nfc_setup_data_interface;

	mtd = nand_to_mtd(chip);
	mtd->dev.parent = dev;

	/*
	 * Default to HW ECC engine mode. If the nand-ecc-mode property is given
	 * in the DT node, this entry will be overwritten in nand_scan_ident().
	 */
	chip->ecc.mode = NAND_ECC_HW;

	ret = nand_scan_ident(mtd, marvell_nand->nsels, NULL);
	if (ret) {
		dev_err(dev, "could not identify the nand chip\n");
		return ret;
	}

	if (pdata && pdata->flash_bbt)
		chip->bbt_options |= NAND_BBT_USE_FLASH;

	if (chip->bbt_options & NAND_BBT_USE_FLASH) {
		/*
		 * We'll use a bad block table stored in-flash and don't
		 * allow writing the bad block marker to the flash.
		 */
		chip->bbt_options |= NAND_BBT_NO_OOB_BBM;
		chip->bbt_td = &bbt_main_descr;
		chip->bbt_md = &bbt_mirror_descr;
	}

	/*
	 * With RA_START bit set in NDCR, columns takes two address cycles. This
	 * means addressing a chip with more than 256 pages needs a fifth
	 * address cycle. Addressing a chip using CS 2 or 3 should also needs
	 * this additional cycle but due to insistance in the documentation and
	 * lack of hardware to test this situation, this case has been dropped
	 * and is not supported by this driver.
	 */
	marvell_nand->addr_cyc = 4;
	if (chip->options & NAND_ROW_ADDR_3)
		marvell_nand->addr_cyc = 5;

	if (pdata) {
		chip->ecc.size = pdata->ecc_step_size;
		chip->ecc.strength = pdata->ecc_strength;
	}

	ret = marvell_nand_ecc_init(mtd, &chip->ecc);
	if (ret) {
		dev_err(dev, "ECC init failed: %d\n", ret);
		return ret;
	}

	if (chip->ecc.mode == NAND_ECC_HW) {
		/*
		 * Subpage write not available with hardware ECC, prohibit also
		 * subpage read as in userspace subpage acces would still be
		 * allowed and subpage write, if used, would lead to numerous
		 * uncorrectable ECC errors.
		 */
		chip->options |= NAND_NO_SUBPAGE_WRITE;
	}

	if (pdata || nfc->caps->legacy_of_bindings) {
		/*
		 * We keep the MTD name unchanged to avoid breaking platforms
		 * where the MTD cmdline parser is used and the bootloader
		 * has not been updated to use the new naming scheme.
		 */
		mtd->name = "pxa3xx_nand-0";
	} else if (!mtd->name) {
		/*
		 * If the new bindings are used and the bootloader has not been
		 * updated to pass a new mtdparts parameter on the cmdline, you
		 * should define the following property in your NAND node, ie:
		 *
		 *	label = "main-storage";
		 *
		 * This way, mtd->name will be set by the core when
		 * nand_set_flash_node() is called.
		 */
		mtd->name = devm_kasprintf(nfc->dev, GFP_KERNEL,
					   "%s:nand.%d", dev_name(nfc->dev),
					   marvell_nand->sels[0].cs);
		if (!mtd->name) {
			dev_err(nfc->dev, "Failed to allocate mtd->name\n");
			return -ENOMEM;
		}
	}

	ret = nand_scan_tail(mtd);
	if (ret) {
		dev_err(dev, "nand_scan_tail failed: %d\n", ret);
		return ret;
	}

	if (pdata)
		/* Legacy bindings support only one chip */
		ret = mtd_device_register(mtd, pdata->parts[0],
					  pdata->nr_parts[0]);
	else
		ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		dev_err(dev, "failed to register mtd device: %d\n", ret);
		nand_release(mtd);
		return ret;
	}

	list_add_tail(&marvell_nand->node, &nfc->chips);

	return 0;
}

static int marvell_nand_chips_init(struct device *dev, struct marvell_nfc *nfc)
{
	struct device_node *np = dev->of_node;
	struct device_node *nand_np;
	int max_cs = nfc->caps->max_cs_nb;
	int nchips;
	int ret;

	if (!np)
		nchips = 1;
	else
		nchips = of_get_child_count(np);

	if (nchips > max_cs) {
		dev_err(dev, "too many NAND chips: %d (max = %d CS)\n", nchips,
			max_cs);
		return -EINVAL;
	}

	/*
	 * Legacy bindings do not use child nodes to exhibit NAND chip
	 * properties and layout. Instead, NAND properties are mixed with the
	 * controller's and a single subnode presents the memory layout.
	 */
	if (nfc->caps->legacy_of_bindings) {
		ret = marvell_nand_chip_init(dev, nfc, np);
		return ret;
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
		nand_release(nand_to_mtd(&entry->chip));
		list_del(&entry->node);
	}
}

static int marvell_nfc_init(struct marvell_nfc *nfc)
{
	struct device_node *np = nfc->dev->of_node;
	u32 enable_arbiter = 0;

	/*
	 * Some SoCs like A7k/A8k need to enable manually the NAND
	 * controller to avoid being bootloader dependent. This is done
	 * through the use of a single bit in the System Functions registers.
	 */
	if (nfc->caps->need_system_controller) {
		struct regmap *sysctrl_base = syscon_regmap_lookup_by_phandle(
			np, "marvell,system-controller");
		u32 reg;

		if (IS_ERR(sysctrl_base))
			return PTR_ERR(sysctrl_base);

		regmap_read(sysctrl_base, GENCONF_SOC_DEVICE_MUX, &reg);
		reg |= GENCONF_SOC_DEVICE_MUX_NFC_EN;
		regmap_write(sysctrl_base, GENCONF_SOC_DEVICE_MUX, reg);
	}

	/*
	 * Enable arbiter for proper NOR/NAND arbitration (PXA only, other SoCs
	 * have this bit marked reserved).
	 */
	if (nfc->caps->need_arbiter)
		enable_arbiter = NDCR_ND_ARB_EN;

	/*
	 * ECC operations and interruptions are only enabled when specifically
	 * needed. ECC shall not be activated in the early stages (fails probe)
	 */
	writel_relaxed(NDCR_RA_START | NDCR_ALL_INT | enable_arbiter,
		       nfc->regs + NDCR);
	writel_relaxed(0xFFFFFFFF, nfc->regs + NDSR);
	writel_relaxed(0, nfc->regs + NDECCCTRL);

	return 0;
}

static int marvell_nfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *r;
	struct marvell_nfc *nfc;
	int ret;
	int irq;

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

	nfc->ecc_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(nfc->ecc_clk))
		return PTR_ERR(nfc->ecc_clk);

	ret = clk_prepare_enable(nfc->ecc_clk);
	if (ret)
		return ret;

	marvell_nfc_disable_int(nfc, NDCR_ALL_INT);
	marvell_nfc_clear_int(nfc, NDCR_ALL_INT);
	ret = devm_request_irq(dev, irq, marvell_nfc_isr,
			       0, "marvell-nfc", nfc);
	if (ret)
		goto out_clk_unprepare;

	/* Get NAND controller capabilities */
	if (pdev->id_entry)
		nfc->caps = (void *)pdev->id_entry->driver_data;
	else
		nfc->caps = of_device_get_match_data(&pdev->dev);

	if (!nfc->caps) {
		dev_err(dev, "Could not retrieve NFC caps\n");
		ret = -EINVAL;
		goto out_clk_unprepare;
	}

	/* Init the controller and then probe the chips */
	ret = marvell_nfc_init(nfc);
	if (ret)
		goto out_clk_unprepare;

	platform_set_drvdata(pdev, nfc);

	ret = marvell_nand_chips_init(dev, nfc);
	if (ret)
		goto out_clk_unprepare;

	return 0;

out_clk_unprepare:
	clk_disable_unprepare(nfc->ecc_clk);

	return ret;
}

static int marvell_nfc_remove(struct platform_device *pdev)
{
	struct marvell_nfc *nfc = platform_get_drvdata(pdev);

	marvell_nand_chips_cleanup(nfc);

	clk_disable_unprepare(nfc->ecc_clk);

	return 0;
}

static const struct marvell_nfc_caps marvell_armada_8k_nfc_caps = {
	.max_cs_nb = 4,
	.max_rb_nb = 2,
	.need_system_controller = true,
};

static const struct marvell_nfc_caps marvell_armada370_nfc_caps = {
	.max_cs_nb = 4,
	.max_rb_nb = 2,
};

static const struct marvell_nfc_caps marvell_pxa3xx_nfc_caps = {
	.max_cs_nb = 2,
	.max_rb_nb = 1,
	.need_arbiter = true,
};

static const struct marvell_nfc_caps marvell_armada_8k_nfc_legacy_caps = {
	.max_cs_nb = 4,
	.max_rb_nb = 2,
	.need_system_controller = true,
	.legacy_of_bindings = true,
};

static const struct marvell_nfc_caps marvell_armada370_nfc_legacy_caps = {
	.max_cs_nb = 4,
	.max_rb_nb = 2,
	.legacy_of_bindings = true,
};

static const struct marvell_nfc_caps marvell_pxa3xx_nfc_legacy_caps = {
	.max_cs_nb = 2,
	.max_rb_nb = 1,
	.need_arbiter = true,
	.legacy_of_bindings = true,
};

static const struct platform_device_id marvell_nfc_platform_ids[] = {
	{
		.name = "pxa3xx-nand",
		.driver_data = (kernel_ulong_t)&marvell_pxa3xx_nfc_legacy_caps,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, marvell_nfc_platform_ids);

static const struct of_device_id marvell_nfc_of_ids[] = {
	{
		.compatible = "marvell,armada-8k-nand-controller",
		.data = &marvell_armada_8k_nfc_caps,
	},
	{
		.compatible = "marvell,armada370-nand-controller",
		.data = &marvell_armada370_nfc_caps,
	},
	{
		.compatible = "marvell,pxa3xx-nand-controller",
		.data = &marvell_pxa3xx_nfc_caps,
	},
	/* Support for old/deprecated bindings: */
	{
		.compatible = "marvell,armada-8k-nand",
		.data = &marvell_armada_8k_nfc_legacy_caps,
	},
	{
		.compatible = "marvell,armada370-nand",
		.data = &marvell_armada370_nfc_legacy_caps,
	},
	{
		.compatible = "marvell,pxa3xx-nand",
		.data = &marvell_pxa3xx_nfc_legacy_caps,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, marvell_nfc_of_ids);

static struct platform_driver marvell_nfc_driver = {
	.driver	= {
		.name		= "marvell-nfc",
		.of_match_table = marvell_nfc_of_ids,
	},
	.id_table = marvell_nfc_platform_ids,
	.probe = marvell_nfc_probe,
	.remove	= marvell_nfc_remove,
};
module_platform_driver(marvell_nfc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Marvell NAND controller driver");
