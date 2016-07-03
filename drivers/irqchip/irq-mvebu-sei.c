/*
 * Copyright (C) 2017 Marvell Technology Group Ltd.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPLv2 or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 *  a) This library is free software; you can redistribute it and/or
 *     modify it under the terms of the GNU General Public License as
 *     published by the Free Software Foundation; either version 2 of the
 *     License, or (at your option) any later version.
 *
 *     This library is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 * Or, alternatively,
 *
 *  b) Permission is hereby granted, free of charge, to any person
 *     obtaining a copy of this software and associated documentation
 *     files (the "Software"), to deal in the Software without
 *     restriction, including without limitation the rights to use,
 *     copy, modify, merge, publish, distribute, sublicense, and/or
 *     sell copies of the Software, and to permit persons to whom the
 *     Software is furnished to do so, subject to the following
 *     conditions:
 *
 *     The above copyright notice and this permission notice shall be
 *     included in all copies or substantial portions of the Software.
 *
 *     THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *     EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *     OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *     NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *     HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *     WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *     FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *     OTHER DEALINGS IN THE SOFTWARE.
 */

#define pr_fmt(fmt) "mvebu-sei: " fmt

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqchip.h>

#define GICP_SECR(idx)		(0x0  + (idx * 0x4)) /* Cause register */
#define GICP_SEMR(idx)		(0x20 + (idx * 0x4)) /* Mask register */

#define SEI_IRQS_PER_REG_CNT	32
#define SEI_IRQ_MASK_VAL	0
/* 0 means Not Masked and 1 means Masked in sei irq mask bit */
#define SEI_IRQ_MASKED		1
/* Write 1 (to all bits) to clear the cause bit */
#define SEI_IRQ_CAUSE_VAL	0xFFFFFFFF


#define SEI_IRQ_REG_CNT		2
#define SEI_IRQ_REG_IDX(irq_id)	(irq_id / SEI_IRQS_PER_REG_CNT)
#define SEI_IRQ_REG_BIT(irq_id)	(irq_id % SEI_IRQS_PER_REG_CNT)

struct sei_data {
	struct resource res;	/* SEI register resource */
	void __iomem *base;	/* SEI register base */
	u32 parent_irq;		/* Parent IRQ */
	u32 nr_irqs;		/* The number interrupts in this SEI */
	struct irq_domain *domain;
};

static void mvebu_sei_reset(struct sei_data *sei)
{
	unsigned int i;

	/* ACK and mask all interrupts */
	for (i = 0; i < SEI_IRQ_REG_CNT; i++) {
		writel(SEI_IRQ_CAUSE_VAL, sei->base + GICP_SECR(i));
		writel(SEI_IRQ_MASK_VAL, sei->base + GICP_SEMR(i));
	}
}

static void mvebu_sei_mask_irq(struct irq_data *d)
{
	struct sei_data *sei = irq_data_get_irq_chip_data(d);
	u32 reg_idx = SEI_IRQ_REG_IDX(d->hwirq);
	u32 reg;

	reg =  readl(sei->base + GICP_SEMR(reg_idx));
	/* 1 disables the interrupt */
	reg |= (SEI_IRQ_MASKED << SEI_IRQ_REG_BIT(d->hwirq));
	writel(reg, sei->base + GICP_SEMR(reg_idx));
}

static void mvebu_sei_unmask_irq(struct irq_data *d)
{
	struct sei_data *sei = irq_data_get_irq_chip_data(d);
	u32 reg_idx = SEI_IRQ_REG_IDX(d->hwirq);
	u32 reg;

	reg =  readl(sei->base + GICP_SEMR(reg_idx));
	/* 0 enables the interrupt */
	reg &= ~(SEI_IRQ_MASKED << SEI_IRQ_REG_BIT(d->hwirq));
	writel(reg, sei->base + GICP_SEMR(reg_idx));
}

static struct irq_chip mvebu_sei_irq_chip = {
	.name			= "SEI",
	.irq_mask		= mvebu_sei_mask_irq,
	.irq_unmask		= mvebu_sei_unmask_irq,
};

static int mvebu_sei_irq_map(struct irq_domain *domain, unsigned int virq,
			     irq_hw_number_t hwirq)
{
	struct sei_data *sei = domain->host_data;

	irq_set_chip_data(virq, sei);
	irq_set_chip_and_handler(virq, &mvebu_sei_irq_chip, handle_level_irq);

	irq_set_status_flags(virq, IRQ_LEVEL);
	irq_set_probe(virq);

	return 0;
}

static const struct irq_domain_ops mvebu_sei_domain_ops = {
	.map = mvebu_sei_irq_map,
	.xlate = irq_domain_xlate_onecell,
};

static void mvebu_sei_handle_cascade_irq(struct irq_desc *desc)
{
	struct sei_data *sei = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long irq_bit, irqmap;
	unsigned long irqn; /* The absolute irq id /(out of 64 irqs)*/
	unsigned int cascade_irq;
	unsigned int reg_idx;

	chained_irq_enter(chip, desc);

	/* Read both SEI cause registers (64 bits) */
	for (reg_idx = 0; reg_idx < SEI_IRQ_REG_CNT; reg_idx++) {
		irqmap = readl_relaxed(sei->base + GICP_SECR(reg_idx));

		/* Call handler for each set bit */
		for_each_set_bit(irq_bit, &irqmap, SEI_IRQS_PER_REG_CNT) {
			/* Calculate IRQ according to Cause register ID */
			irqn = irq_bit + reg_idx * SEI_IRQS_PER_REG_CNT;

			cascade_irq = irq_find_mapping(sei->domain, irqn);
			generic_handle_irq(cascade_irq); /* Call IRQ handler */
		}

		/*
		 * SEI cause register is Write-1-Clear (i.e. the interrupt
		 * indication is cleared when writing 1 to it) so we
		 * write the same value to clear the interrupt indication
		 */
		writel(irqmap, sei->base + GICP_SECR(reg_idx));
	}

	chained_irq_exit(chip, desc);
}

static int __init mvebu_sei_of_init(struct device_node *node,
				    struct device_node *parent)
{
	int ret;
	struct sei_data *sei;

	sei = kzalloc(sizeof(struct sei_data), GFP_KERNEL);
	if (!sei)
		return -ENOMEM;

	ret = of_address_to_resource(node, 0, &sei->res);
	if (ret) {
		pr_err("Failed to allocate sei resource.\n");
		goto err_free_sei;
	}

	sei->base = ioremap(sei->res.start, resource_size(&sei->res));
	if (!sei->base) {
		pr_err("Failed to map sei resource\n");
		ret = -ENOMEM;
		goto err_free_sei;
	}

	/* Set number of IRQs */
	sei->nr_irqs = SEI_IRQS_PER_REG_CNT * SEI_IRQ_REG_CNT;

	sei->domain = irq_domain_add_linear(node, sei->nr_irqs,
					    &mvebu_sei_domain_ops, sei);
	if (!sei->domain) {
		pr_err("Failed to allocate irq domain\n");
		ret = -ENOMEM;
		goto err_iounmap;
	}

	sei->parent_irq = irq_of_parse_and_map(node, 0);
	if (sei->parent_irq <= 0) {
		pr_err("Failed to parse parent interrupt\n");
		ret = -EINVAL;
		goto err_free_domain;
	}

	irq_set_chained_handler(sei->parent_irq, mvebu_sei_handle_cascade_irq);
	irq_set_handler_data(sei->parent_irq, sei);

	mvebu_sei_reset(sei);
	pr_info("registered with %d irqs\n", sei->nr_irqs);

	return 0;

err_free_domain:
	irq_domain_remove(sei->domain);
err_iounmap:
	iounmap(sei->base);
err_free_sei:
	kfree(sei);

	return ret;
}

IRQCHIP_DECLARE(mvebu_sei, "marvell,sei", mvebu_sei_of_init);
