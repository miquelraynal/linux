// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2024 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/wait.h>

#include <linux/interrupt.h>
#include <linux/of.h>
#include <asm/irq.h>

#if 1 /* TODO : remove it */
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#endif

/* spi registers offset */
#define REG_CTL         0x00
#define REG_CLKDIV      0x04
#define REG_SSCTL       0x08
#define REG_PDMACTL     0x0C
#define REG_FIFOCTL     0x10
#define REG_STATUS      0x14
#define REG_STATUS2     0x18
#define REG_TX          0x20
#define REG_RX          0x30
#define REG_INTERNAL    0x48

/* spi register bit */
#define DTREN           (0x01 << 23)
#define QUADIOEN        (0x01 << 22)
#define DUALIOEN        (0x01 << 21)
#define DATDIR          (0x01 << 20)
#define UNITIEN         (0x01 << 17)
#define TXNEG           (0x01 << 2)
#define RXNEG           (0x01 << 1)
#define LSB             (0x01 << 13)
#define SELECTLEV       (0x01 << 2)
#define SELECTPOL       (0x01 << 3)
#define SELECTSLAVE0    0x01
#define SELECTSLAVE1    0x02
#define SPIEN           0x01
#define DWIDTH_MASK     0x1F00
#define DWIDTH_POS      8
#define BYTE_REORDER    0x80000
#define TXPDMAEN        0x01
#define RXPDMAEN        0x02
#define RXRST           0x01
#define TXRST           0x02
#define RXFBCLR         0x100
#define TXFBCLR         0x200
#define BUSY            0x01
#define UNITIF          0x02
#define RXEMPTY         0x100
#define RXFULL          0x200
#define SPIENSTS        0x8000
#define TXEMPTY         0x10000
#define TXFULL          0x20000
#define FIFOCLR         0x400000
#define TXRXRST         0x800000

#define OP_BUSW_1	0
#define OP_BUSW_2	1
#define OP_BUSW_4	2

/* define for PDMA */
#define ALIGNMENT_4     4
#define USE_PDMA_LEN    100

#define SPI_GENERAL_TIMEOUT_MS	1000

struct nuvoton_qspi_info {
	unsigned int num_cs;
	unsigned int lsb;
	unsigned int txneg;
	unsigned int rxneg;
	unsigned int divider;
	unsigned int sleep;
	unsigned int txbitlen;
	unsigned int clkpol;
	int bus_num;
	unsigned int spimode;
	unsigned int hz;
	unsigned int quad;
	unsigned int mrxphase;
};

struct nuvoton_spi {
	struct completion               txdone;
	struct completion               rxdone;
	void __iomem                    *regs;
	int                             irq;
	unsigned int                    len;
	unsigned int                    count;
	const void                      *tx;
	void                            *rx;
	struct clk                      *clk;
	struct spi_controller           *host;
	struct device                   *dev;
	struct nuvoton_qspi_info	*pdata;
	spinlock_t                      lock;
	struct resource                 *res;
	int                             slave_txdone_state;
	int                             slave_rxdone_state;
	struct wait_queue_head          slave_txdone;
	struct wait_queue_head          slave_rxdone;
	unsigned int                    phyaddr;
	unsigned int			cur_speed_hz;
};

static int nuvoton_spi_clk_enable(struct nuvoton_spi *nuvoton)
{
	int ret;

	ret = clk_prepare_enable(nuvoton->clk);

	return ret;
}

static void nuvoton_spi_clk_disable(struct nuvoton_spi *nuvoton)
{
	clk_disable_unprepare(nuvoton->clk);
}

static inline void nuvoton_set_divider(struct nuvoton_spi *hw)
{
	__raw_writel(hw->pdata->divider, hw->regs + REG_CLKDIV);
}

static int nuvoton_spi_clk_setup(struct nuvoton_spi *hw, unsigned long freq)
{
	unsigned int clk;
	unsigned int div;

	//clk = clk_get_rate(hw->clk);
	clk = 180000000;
	div = clk / freq - 1;
	//HACK: force minimum frequency
	//div = 511; // This is 351kHz
	freq = clk / (div + 1);
	hw->pdata->hz = freq;
	hw->pdata->divider = div;
	printk("%s [%d] div: %d, freq %luHz\n", __func__, __LINE__, div, freq);

	nuvoton_set_divider(hw);

	return 0;
}

static inline void nuvoton_setup_txbitlen(struct nuvoton_spi *hw,
	unsigned int txbitlen)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);
	val &= ~0x1f00;
	if (txbitlen != 32)
		val |= (txbitlen << 8);

	__raw_writel(val, hw->regs + REG_CTL);

}

static inline void nuvoton_set_clock_polarity(struct nuvoton_spi *hw, unsigned int polarity)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	if (polarity)
		val |= SELECTPOL;
	else
		val &= ~SELECTPOL;
	__raw_writel(val, hw->regs + REG_CTL);
}

static int nuvoton_spi_set_freq(struct nuvoton_spi *nuvoton, unsigned long freq)
{
	int ret;

	if (nuvoton->cur_speed_hz == freq)
		return 0;

	ret = nuvoton_spi_clk_setup(nuvoton, freq);
	if (ret)
		return ret;

	nuvoton->cur_speed_hz = freq;

	return 0;
}

static inline void nuvoton_tx_rx_edge(struct nuvoton_spi *hw, unsigned int tx_edge, unsigned int rx_edge)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	if (tx_edge)
		val |= TXNEG;
	else
		val &= ~TXNEG;

	if (rx_edge)
		val |= RXNEG;
	else
		val &= ~RXNEG;

	__raw_writel(val, hw->regs + REG_CTL);

}

static inline void nuvoton_send_first(struct nuvoton_spi *hw, unsigned int lsb)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	if (lsb)
		val |= LSB;
	else
		val &= ~LSB;
	__raw_writel(val, hw->regs + REG_CTL);
}

static inline void nuvoton_set_sleep(struct nuvoton_spi *hw, unsigned int sleep)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	val &= ~(0x0f << 4);

	if (sleep)
		val |= (sleep << 4);

	__raw_writel(val, hw->regs + REG_CTL);
}

static int nuvoton_spi_update_state(struct spi_device *spi)
{
	struct nuvoton_spi *hw = spi_controller_get_devdata(spi->controller);
	unsigned char spimode;

	spi->mode = hw->pdata->spimode;

	if (hw->pdata->quad)
		spi->mode |= (SPI_TX_QUAD | SPI_RX_QUAD);

	//Mode 0: CPOL=0, CPHA=0; active high
	//Mode 1: CPOL=0, CPHA=1; active low
	//Mode 2: CPOL=1, CPHA=0; active low
	//Mode 3: CPOL=1, CPHA=1; active high
	if (spi->mode & SPI_CPOL)
		hw->pdata->clkpol = 1;
	else
		hw->pdata->clkpol = 0;

	spimode = spi->mode & 0xff; //remove dual/quad bit

	if ((spimode == SPI_MODE_0) || (spimode == SPI_MODE_3)) {
		hw->pdata->txneg = 1;
		hw->pdata->rxneg = 0;
	} else {
		hw->pdata->txneg = 0;
		hw->pdata->rxneg = 1;
	}

	if (spi->mode & SPI_LSB_FIRST)
		hw->pdata->lsb = 1;
	else
		hw->pdata->lsb = 0;

	return 0;
}

static int nuvoton_spi_setupxfer(struct spi_device *spi)
{
	struct nuvoton_spi *hw = spi_controller_get_devdata(spi->controller);
	int ret;

	ret = nuvoton_spi_update_state(spi);
	if (ret)
		return ret;

	nuvoton_setup_txbitlen(hw, hw->pdata->txbitlen);
	nuvoton_tx_rx_edge(hw, hw->pdata->txneg, hw->pdata->rxneg);
	nuvoton_set_clock_polarity(hw, hw->pdata->clkpol);
	nuvoton_send_first(hw, hw->pdata->lsb);
	nuvoton_set_divider(hw);

	return 0;
}

static void nuvoton_spi_hw_init(struct nuvoton_spi *hw)
{
	spin_lock_init(&hw->lock);

	if (hw->pdata->spimode & SPI_CPOL)
		hw->pdata->clkpol = 1;
	else
		hw->pdata->clkpol = 0;

	if ((hw->pdata->spimode == SPI_MODE_0) || (hw->pdata->spimode == SPI_MODE_3)) {
		hw->pdata->txneg = 1;
		hw->pdata->rxneg = 0;
	} else {
		hw->pdata->txneg = 0;
		hw->pdata->rxneg = 1;
	}

	nuvoton_tx_rx_edge(hw, hw->pdata->txneg, hw->pdata->rxneg);
	nuvoton_send_first(hw, hw->pdata->lsb);
	nuvoton_set_sleep(hw, hw->pdata->sleep);
	nuvoton_setup_txbitlen(hw, hw->pdata->txbitlen);
	nuvoton_set_clock_polarity(hw, hw->pdata->clkpol);

	__raw_writel((__raw_readl(hw->regs + REG_INTERNAL) & ~0xF000) | (hw->pdata->mrxphase << 12),
			hw->regs + REG_INTERNAL); /* MRxPhase(QSPI_INTERNAL[15:12] */

	__raw_writel(__raw_readl(hw->regs + REG_CTL) | SPIEN, hw->regs + REG_CTL); /* enable SPI */
	while ((__raw_readl(hw->regs + REG_STATUS) & SPIENSTS) == 0)
		;

	__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | (TXRST | RXRST), hw->regs + REG_FIFOCTL);
	while (__raw_readl(hw->regs + REG_STATUS) & TXRXRST)
		;

}

static inline unsigned int hw_tx(struct nuvoton_spi *hw, unsigned int count)
{
	const unsigned char *tx_byte = hw->tx;
	const unsigned short *tx_short = hw->tx;
	const unsigned int *tx_int = hw->tx;
	unsigned int bwp = hw->pdata->txbitlen;

	if (bwp <= 8)
		return tx_byte ? tx_byte[count] : 0;
	else if (bwp <= 16)
		return tx_short ? tx_short[count] : 0;
	else
		return tx_int ? tx_int[count] : 0;
}

static inline void hw_rx(struct nuvoton_spi *hw, unsigned int data, int count)
{
	unsigned char *rx_byte = hw->rx;
	unsigned short *rx_short = hw->rx;
	unsigned int *rx_int = hw->rx;
	int bwp = hw->pdata->txbitlen;

	if (bwp <= 8)
		rx_byte[count] = data;
	else if (bwp <= 16)
		rx_short[count] = data;
	else
		rx_int[count] = data;
}

static noinline int nuvoton_spi_rst_fifo(struct nuvoton_spi *hw)
{
	unsigned long end;

	writel(__raw_readl(hw->regs + REG_FIFOCTL) | (TXRST | RXRST), hw->regs + REG_FIFOCTL);
	end = jiffies + msecs_to_jiffies(SPI_GENERAL_TIMEOUT_MS);
	while (__raw_readl(hw->regs + REG_STATUS) & TXRXRST) {
		cpu_relax();
		if (time_after(jiffies, end)) {
			printk("SPI TXRXRST timeout: %d\n", __LINE__);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int nuvoton_spi_poll_status(struct nuvoton_spi *hw, u32 mask)
{
	unsigned long end;

	end = jiffies + msecs_to_jiffies(SPI_GENERAL_TIMEOUT_MS);
	while ((__raw_readl(hw->regs + REG_STATUS) & mask) == mask) {
		cpu_relax();
		if (time_after(jiffies, end)) {
			printk("SPI BUSY timeout: %x\n", mask);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static noinline int nuvoton_spi_data_xfer(struct nuvoton_spi *hw, const void *txbuf,
					  void *rxbuf, unsigned int len)
{
	unsigned int  i;
	int ret;

	ret = nuvoton_spi_rst_fifo(hw);
	if (ret)
		return ret;

	hw->tx = txbuf;
	hw->rx = rxbuf;

	if (hw->rx) {
		for (i = 0; i < len; i++) {
			__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
			ret = nuvoton_spi_poll_status(hw, RXEMPTY);
			if (ret)
				return ret;
			hw_rx(hw, __raw_readl(hw->regs + REG_RX), i);
		}
	} else {
		for (i = 0; i < len; i++) {
			ret = nuvoton_spi_poll_status(hw, TXFULL);
			if (ret)
				return ret;
			__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
		}
	}

	ret = nuvoton_spi_poll_status(hw, BUSY);
	if (ret)
		return ret;

	return 0;
}

static bool nuvoton_spi_mem_supports_op(struct spi_mem *mem,
					const struct spi_mem_op *op)
{
	if (op->data.buswidth > 4 || op->addr.buswidth > 4 ||
		op->dummy.buswidth > 4 || op->cmd.buswidth > 1)
		return false;

	if (op->addr.nbytes > 7)
		return false;

	return true;
}

static void nuvoton_spi_set_cs(struct spi_device *spi, bool lvl)
{
	struct nuvoton_spi *nuvoton = spi_controller_get_devdata(spi->controller);
	int chip_select;
	unsigned long end;
	unsigned int val;

	chip_select = spi_get_chipselect(spi, 0);

	val = __raw_readl(nuvoton->regs + REG_SSCTL);

	if (chip_select == 0) {
		if (!lvl)
			val |= SELECTSLAVE0;
		else
			val &= ~SELECTSLAVE0;
	} else {
		if (!lvl)
			val |= SELECTSLAVE1;
		else
			val &= ~SELECTSLAVE1;
	}

	end = jiffies + msecs_to_jiffies(3000);
	while (__raw_readl(nuvoton->regs + REG_STATUS) & BUSY) {
		if (time_after(jiffies, end)) {
			printk("SPI BUSY timeout: %d, %s - %d\n", __LINE__, __func__, lvl);
			return;
		}
	}

	__raw_writel(val, nuvoton->regs + REG_SSCTL);
}

static struct nuvoton_qspi_info *nuvoton_spi_parse_dt(struct device *dev)
{
	struct nuvoton_qspi_info *sci;
	u32 temp;

	sci = devm_kzalloc(dev, sizeof(*sci), GFP_KERNEL);
	if (!sci)
		return ERR_PTR(-ENOMEM);

	if (of_property_read_u32(dev->of_node, "num_cs", &temp)) {
		dev_warn(dev, "can't get num_cs from dt\n");
		sci->num_cs = 2;
	} else {
		sci->num_cs = temp;
	}

	if (of_property_read_u32(dev->of_node, "lsb", &temp)) {
		dev_warn(dev, "can't get lsb from dt\n");
		sci->lsb = 0;
	} else {
		sci->lsb = temp;
	}

	if (of_property_read_u32(dev->of_node, "sleep", &temp)) {
		dev_warn(dev, "can't get sleep from dt\n");
		sci->sleep = 0;
	} else {
		sci->sleep = temp;
	}

	if (of_property_read_u32(dev->of_node, "txbitlen", &temp)) {
		dev_warn(dev, "can't get txbitlen from dt\n");
		sci->txbitlen = 8;
	} else {
		sci->txbitlen = temp;
	}

	if (of_property_read_u32(dev->of_node, "bus_num", &temp)) {
		dev_warn(dev, "can't get bus_num from dt\n");
		sci->bus_num = 0;
	} else {
		sci->bus_num = temp;
	}

	if (of_property_read_u32(dev->of_node, "spimode", &temp)) {
		dev_warn(dev, "can't get spimode from dt\n");
		sci->spimode = 0;
	} else {
		sci->spimode = temp;
	}

	if (of_property_read_u32(dev->of_node, "quad", &temp)) {
		dev_warn(dev, "can't get quad from dt\n");
		sci->quad = 0;
	} else {
		sci->quad = temp;
	}

	if (of_property_read_u32(dev->of_node, "mrxphase", &temp)) {
		dev_warn(dev, "can't get mrxphase from dt\n");
		sci->mrxphase = 0;
	} else {
		sci->mrxphase = temp;
	}

	return sci;
}

static int nuvoton_spi_mem_exec_op(struct spi_mem *mem,
					const struct spi_mem_op *op)
{
	struct nuvoton_spi *nuvoton = spi_controller_get_devdata(mem->spi->controller);
	unsigned long flags;
	int i, ret;
	u8 addr[8];
	u32 ctl;

	spin_lock_irqsave(&nuvoton->lock, flags);

	ret = nuvoton_spi_set_freq(nuvoton, op->max_freq);
	if (ret) {
		printk("nuvoton_spi_set_freq failed!\n");
		goto out;
	}
	nuvoton_spi_setupxfer(mem->spi);

	nuvoton_spi_set_cs(mem->spi, 0); //Activate CS

	if (0) //op->cmd.dtr || op->addr.dtr || op->dummy.dtr || op->data.dtr)
		printk("%d%c-%d%c-%d%c-%d%c\n",
		       op->cmd.buswidth, op->cmd.dtr ? 'D' : 'S',
		       op->addr.buswidth, op->addr.dtr ? 'D' : 'S',
		       op->dummy.buswidth, op->dummy.dtr ? 'D' : 'S',
		       op->data.buswidth, op->data.dtr ? 'D' : 'S');

	ctl = __raw_readl(nuvoton->regs + REG_CTL);
	ctl &= ~(QUADIOEN | DUALIOEN | DTREN | DATDIR);
	if (op->cmd.buswidth == 4)
		ctl |= QUADIOEN;
	else if (op->cmd.buswidth == 2)
		ctl |= DUALIOEN;
	if (op->cmd.dtr)
		ctl |= DTREN;
	__raw_writel(ctl | DATDIR, nuvoton->regs + REG_CTL);

	ret = nuvoton_spi_data_xfer(nuvoton, &op->cmd.opcode, NULL, 1);
	if (ret) {
		printk("nuvoton_spi_data_xfer failed!! %d\n", __LINE__);
		goto out;
	}

	ctl = __raw_readl(nuvoton->regs + REG_CTL);
	ctl &= ~(QUADIOEN | DUALIOEN | DTREN | DATDIR);
	if (op->addr.buswidth == 4)
		ctl |= QUADIOEN;
	else if (op->addr.buswidth == 2)
		ctl |= DUALIOEN;
	if (op->addr.dtr)
		ctl |= DTREN;
	__raw_writel(ctl | DATDIR, nuvoton->regs + REG_CTL);

	for (i = 0; i < op->addr.nbytes; i++)
		addr[i] = op->addr.val >> (8 * (op->addr.nbytes - i - 1));

	ret = nuvoton_spi_data_xfer(nuvoton, addr, NULL, op->addr.nbytes);
	if (ret) {
		printk("nuvoton_spi_data_xfer failed!! %d\n", __LINE__);
		goto out;
	}

	ctl = __raw_readl(nuvoton->regs + REG_CTL);
	ctl &= ~(QUADIOEN | DUALIOEN | DTREN | DATDIR);
	if (op->dummy.buswidth == 4)
		ctl |= QUADIOEN;
	else if (op->dummy.buswidth == 2)
		ctl |= DUALIOEN;
	if (op->dummy.dtr)
		ctl |= DTREN;
	__raw_writel(ctl | DATDIR, nuvoton->regs + REG_CTL);

	ret = nuvoton_spi_data_xfer(nuvoton, NULL, NULL, op->dummy.nbytes);
	if (ret) {
		printk("nuvoton_spi_data_xfer failed!! %d\n", __LINE__);
		goto out;
	}

	ctl = __raw_readl(nuvoton->regs + REG_CTL);
	ctl &= ~(QUADIOEN | DUALIOEN | DTREN | DATDIR);
	if (op->data.buswidth == 4)
		ctl |= QUADIOEN;
	else if (op->data.buswidth == 2)
		ctl |= DUALIOEN;
	if (op->data.dir == SPI_MEM_DATA_OUT)
		ctl |= DATDIR;
	if (op->data.dtr)
		ctl |= DTREN;
	__raw_writel(ctl, nuvoton->regs + REG_CTL);

	ret = nuvoton_spi_data_xfer(nuvoton,
				    op->data.dir == SPI_MEM_DATA_OUT ?
				    op->data.buf.out : NULL,
				    op->data.dir == SPI_MEM_DATA_IN ?
				    op->data.buf.in : NULL,
				    op->data.nbytes);

out:
	/* Restore to single mode, direction input */
	ctl = __raw_readl(nuvoton->regs + REG_CTL);
	ctl &= ~(QUADIOEN | DUALIOEN | DATDIR | DTREN);
	__raw_writel(ctl, nuvoton->regs + REG_CTL);

	nuvoton_spi_set_cs(mem->spi, 1); //Deactivate CS

	spin_unlock_irqrestore(&nuvoton->lock, flags);

	return ret;
}

static const struct spi_controller_mem_ops nuvoton_spi_mem_ops = {
	.supports_op = nuvoton_spi_mem_supports_op,
	.exec_op = nuvoton_spi_mem_exec_op,
};

static const struct spi_controller_mem_caps nuvoton_mem_caps = {
	.dtr = true,
	.per_op_freq = true,
};

static int nuvoton_spi_transfer_one(struct spi_controller *host,
					struct spi_device *spi,
					struct spi_transfer *t)
{
	struct nuvoton_spi *nuvoton = spi_controller_get_devdata(host);
	unsigned int busw = OP_BUSW_1;
	int ret;

	nuvoton_spi_setupxfer(spi);

	if (t->rx_buf && t->tx_buf) {
		if (((spi->mode & SPI_TX_QUAD) &&
		     !(spi->mode & SPI_RX_QUAD)) ||
		    ((spi->mode & SPI_TX_DUAL) &&
		     !(spi->mode & SPI_RX_DUAL)))
			return -ENOTSUPP;
	}

	if (t->tx_buf) {
		if (spi->mode & SPI_TX_QUAD)
			busw = OP_BUSW_4;
		else if (spi->mode & SPI_TX_DUAL)
			busw = OP_BUSW_2;
	} else if (t->rx_buf) {
		if (spi->mode & SPI_RX_QUAD)
			busw = OP_BUSW_4;
		else if (spi->mode & SPI_RX_DUAL)
			busw = OP_BUSW_2;
	}

	if (busw == OP_BUSW_4)
		__raw_writel(((__raw_readl(nuvoton->regs + REG_CTL) | QUADIOEN) & ~DATDIR),
				nuvoton->regs + REG_CTL);//Enable Quad mode, direction input

	ret = nuvoton_spi_set_freq(nuvoton, t->speed_hz);
	if (ret)
		return ret;

	ret = nuvoton_spi_data_xfer(nuvoton, t->tx_buf, t->rx_buf, t->len);
	if (ret)
		return ret;

	spi_finalize_current_transfer(host);

	return 0;
}

static int __maybe_unused nuvoton_spi_runtime_suspend(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct nuvoton_spi *nuvoton = spi_controller_get_devdata(host);

	nuvoton_spi_clk_disable(nuvoton);
	clk_disable_unprepare(nuvoton->clk);

	return 0;
}

static int __maybe_unused nuvoton_spi_runtime_resume(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct nuvoton_spi *nuvoton = spi_controller_get_devdata(host);
	int ret;

	ret = clk_prepare_enable(nuvoton->clk);
	if (ret) {
		dev_err(dev, "Cannot enable ps_clock.\n");
		return ret;
	}

	return nuvoton_spi_clk_enable(nuvoton);
}

static const struct dev_pm_ops nuvoton_spi_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(nuvoton_spi_runtime_suspend,
				nuvoton_spi_runtime_resume, NULL)
};

static int nuvoton_spi_probe(struct platform_device *pdev)
{
	struct spi_controller   *host;
	struct nuvoton_spi *nuvoton;
	int ret;
	u32   val32[4];
	int err = 0;
	int status = 0;

#if 1 /* TODO: remove it */
	{
		/*
		unsigned int val;
		struct regmap *regmap;
		regmap = syscon_regmap_lookup_by_phandle(dev_of_node(&pdev->dev), "nuvoton,sys");
		printk("0 regmap = 0x%lx\n", (unsigned long)regmap);
        	if (IS_ERR(regmap)) {
                	dev_err(nuvoton->dev, "Failed to get SYS registers: %ld\n",
                        	PTR_ERR(regmap));
                	return PTR_ERR(regmap);
        	}
		printk("regmap = 0x%lx\n", (unsigned long)regmap);
		regmap_read(regmap, 0x98, &val);
		printk("1 PD multi-function pin = 0x%x\n", val);
		regmap_write(regmap, 0x98, (val & ~0xFFFFFF) | 0x555555);
		regmap_read(regmap, 0x98, &val);
		printk("2 PD multi-function pin = 0x%x\n", val);
		*/

		long unsigned int sysbase;
		sysbase = (long unsigned int)ioremap(0x40460000, 0x200);
		*(long unsigned int*)(sysbase + 0x98) &= ~0xFFFFFF;
		*(long unsigned int*)(sysbase + 0x98) |= 0x555555;
	}
#endif

	host = spi_alloc_host(&pdev->dev, sizeof(struct nuvoton_spi));
	if (!host)
                return -ENOMEM;

	platform_set_drvdata(pdev, host);

        nuvoton = spi_controller_get_devdata(host);

	host->num_chipselect = 2;
	host->mem_ops = &nuvoton_spi_mem_ops;
	host->mem_caps = &nuvoton_mem_caps;

	host->set_cs = nuvoton_spi_set_cs;
	host->transfer_one = nuvoton_spi_transfer_one;
	host->bits_per_word_mask = SPI_BPW_MASK(8);
	host->mode_bits = SPI_CPOL | SPI_CPHA |
				SPI_RX_DUAL | SPI_TX_DUAL |
				SPI_RX_QUAD | SPI_TX_QUAD;

	host->dev.of_node = pdev->dev.of_node;

	nuvoton->host = spi_controller_get(host);
	nuvoton->pdata = nuvoton_spi_parse_dt(&pdev->dev);
	nuvoton->dev = &pdev->dev;
        nuvoton->host = host;
        spin_lock_init(&nuvoton->lock);

	if (nuvoton->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_pdata;
	}

	init_completion(&nuvoton->txdone);
	init_completion(&nuvoton->rxdone);

	nuvoton->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(nuvoton->clk)) {
		dev_err(&pdev->dev, "unable to get SYS clock, err=%d\n",
			status);
		goto err_clk;
	}
	clk_prepare_enable(nuvoton->clk);

	if (of_property_read_u32_array(pdev->dev.of_node, "reg", val32, 4) != 0) {
		dev_err(&pdev->dev, "can not get bank!\n");
		return -EINVAL;
	}

	nuvoton->phyaddr = val32[1];
	pr_debug("nuvoton->phyaddr = 0x%lx\n", (ulong)nuvoton->phyaddr);

	nuvoton->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (nuvoton->res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_pdata;
	}

	nuvoton->regs = devm_ioremap_resource(&pdev->dev, nuvoton->res);
	nuvoton->irq = platform_get_irq(pdev, 0);
	if (nuvoton->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_pdata;
	}

	nuvoton_spi_hw_init(nuvoton);

	ret = devm_spi_register_controller(&pdev->dev, host);
	if (ret) {
		dev_err(&pdev->dev, "devm_spi_register_controller failed\n");
		pm_runtime_disable(&pdev->dev);
	}

	return ret;

err_clk:
	spi_controller_put(nuvoton->host);
err_pdata:
	clk_disable(nuvoton->clk);
	clk_put(nuvoton->clk);

	return err;
}

static void nuvoton_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *host = spi_controller_get(platform_get_drvdata(pdev));

	pm_runtime_get_sync(&pdev->dev);

        pm_runtime_put_noidle(&pdev->dev);
        pm_runtime_disable(&pdev->dev);
        pm_runtime_set_suspended(&pdev->dev);

	spi_controller_put(host);

}

static const struct of_device_id nuvoton_qspi_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-qspi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nuvoton_qspi_of_match);

static struct platform_driver nuvoton_qspi_driver = {
	.driver = {
		.name = "ma35d1-qspi",
		.of_match_table = nuvoton_qspi_of_match,
		.pm = &nuvoton_spi_dev_pm_ops,
	},
	.probe = nuvoton_spi_probe,
	.remove = nuvoton_spi_remove,
};
module_platform_driver(nuvoton_qspi_driver);

MODULE_ALIAS("platform:nuvoton-qspi");
MODULE_DESCRIPTION("Nuvoton MA35D1 QSPI Controller driver!");
MODULE_AUTHOR("Chi-Wen Weng <cwweng@nuvoton.com>");
MODULE_LICENSE("GPL");
