// SPDX-License-Identifier: GPL-2.0

#define pr_fmt(fmt) "mvebu-sei: " fmt

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/msi.h>
#include <linux/platform_device.h>
#include <linux/irqchip.h>

#include <dt-bindings/interrupt-controller/arm-gic.h>

/* Cause register */
#define GICP_SECR(idx)		(0x0  + ((idx) * 0x4))
/* Mask register */
#define GICP_SEMR(idx)		(0x20 + ((idx) * 0x4))
#define GICP_SET_SEI_OFFSET	0x30

#define SEI_IRQ_COUNT_PER_REG	32
#define SEI_IRQ_REG_COUNT	2
#define SEI_IRQ_COUNT		(SEI_IRQ_COUNT_PER_REG * SEI_IRQ_REG_COUNT)
#define SEI_IRQ_REG_IDX(irq_id)	((irq_id) / SEI_IRQ_COUNT_PER_REG)
#define SEI_IRQ_REG_BIT(irq_id)	((irq_id) % SEI_IRQ_COUNT_PER_REG)

struct mvebu_sei_interrupt_range {
	u32 first;
	u32 size;
};

struct mvebu_sei {
	struct device *dev;
	void __iomem *base;
	struct resource *res;
	struct irq_domain *ap_domain;
	struct irq_domain *cp_domain;
	struct mvebu_sei_interrupt_range ap_interrupts;
	struct mvebu_sei_interrupt_range cp_interrupts;
	/* Lock on MSI allocations/releases */
	struct mutex cp_msi_lock;
	DECLARE_BITMAP(cp_msi_bitmap, SEI_IRQ_COUNT);
};

static int mvebu_sei_domain_to_sei_irq(struct mvebu_sei *sei,
				       struct irq_domain *domain,
				       irq_hw_number_t hwirq)
{
	if (domain == sei->ap_domain)
		return sei->ap_interrupts.first + hwirq;
	else
		return sei->cp_interrupts.first + hwirq;
}

static void mvebu_sei_reset(struct mvebu_sei *sei)
{
	u32 reg_idx;

	/* Clear IRQ cause registers */
	for (reg_idx = 0; reg_idx < SEI_IRQ_REG_COUNT; reg_idx++)
		writel_relaxed(0xFFFFFFFF, sei->base + GICP_SECR(reg_idx));
}

static void mvebu_sei_mask_irq(struct irq_data *d)
{
	struct mvebu_sei *sei = irq_data_get_irq_chip_data(d);
	u32 sei_irq = mvebu_sei_domain_to_sei_irq(sei, d->domain, d->hwirq);
	u32 reg_idx = SEI_IRQ_REG_IDX(sei_irq);
	u32 reg;

	/* 1 disables the interrupt */
	reg = readl_relaxed(sei->base + GICP_SEMR(reg_idx));
	reg |= BIT(SEI_IRQ_REG_BIT(sei_irq));
	writel_relaxed(reg, sei->base + GICP_SEMR(reg_idx));
}

static void mvebu_sei_unmask_irq(struct irq_data *d)
{
	struct mvebu_sei *sei = irq_data_get_irq_chip_data(d);
	u32 sei_irq = mvebu_sei_domain_to_sei_irq(sei, d->domain, d->hwirq);
	u32 reg_idx = SEI_IRQ_REG_IDX(sei_irq);
	u32 reg;

	/* 0 enables the interrupt */
	reg = readl_relaxed(sei->base + GICP_SEMR(reg_idx));
	reg &= ~BIT(SEI_IRQ_REG_BIT(sei_irq));
	writel_relaxed(reg, sei->base + GICP_SEMR(reg_idx));
}

static void mvebu_sei_compose_msi_msg(struct irq_data *data,
				      struct msi_msg *msg)
{
	struct mvebu_sei *sei = data->chip_data;
	phys_addr_t set = sei->res->start + GICP_SET_SEI_OFFSET;

	msg->data = mvebu_sei_domain_to_sei_irq(sei, data->domain, data->hwirq);
	msg->address_lo = lower_32_bits(set);
	msg->address_hi = upper_32_bits(set);
}

static int mvebu_sei_ap_set_type(struct irq_data *data, unsigned int type)
{
	if (!(type & IRQ_TYPE_LEVEL_HIGH))
		return -EINVAL;

	return 0;
}

static int mvebu_sei_cp_set_type(struct irq_data *data, unsigned int type)
{
	if (!(type & IRQ_TYPE_EDGE_RISING))
		return -EINVAL;

	return 0;
}

static struct irq_chip mvebu_sei_ap_wired_irq_chip = {
	.name			= "AP wired SEI",
	.irq_mask		= mvebu_sei_mask_irq,
	.irq_unmask		= mvebu_sei_unmask_irq,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_set_type		= mvebu_sei_ap_set_type,
};

static struct irq_chip mvebu_sei_cp_msi_irq_chip = {
	.name			= "CP MSI SEI",
	.irq_mask		= mvebu_sei_mask_irq,
	.irq_unmask		= mvebu_sei_unmask_irq,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_set_type		= mvebu_sei_cp_set_type,
	.irq_compose_msi_msg	= mvebu_sei_compose_msi_msg,
};

static int mvebu_sei_irq_domain_alloc(struct irq_domain *domain,
				      unsigned int virq, unsigned int nr_irqs,
				      void *args)
{
	struct mvebu_sei *sei = domain->host_data;
	struct irq_fwspec *fwspec = args;
	struct irq_chip *irq_chip;
	int sei_hwirq, hwirq;
	int ret;

	/* The software only supports single allocations for now */
	if (nr_irqs != 1)
		return -ENOTSUPP;

	if (domain == sei->ap_domain) {
		irq_chip = &mvebu_sei_ap_wired_irq_chip;
		hwirq = fwspec->param[0];
	} else {
		irq_chip = &mvebu_sei_cp_msi_irq_chip;
		mutex_lock(&sei->cp_msi_lock);
		hwirq = bitmap_find_free_region(sei->cp_msi_bitmap,
						sei->cp_interrupts.size, 0);
		mutex_unlock(&sei->cp_msi_lock);
		if (hwirq < 0)
			return -ENOSPC;
	}

	sei_hwirq = mvebu_sei_domain_to_sei_irq(sei, domain, hwirq);

	fwspec->fwnode = domain->parent->fwnode;
	fwspec->param_count = 3;
	fwspec->param[0] = GIC_SPI;
	fwspec->param[1] = sei_hwirq;
	fwspec->param[2] = IRQ_TYPE_EDGE_RISING;

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, fwspec);
	if (ret)
		goto release_region;

	ret = irq_domain_set_hwirq_and_chip(domain, virq, hwirq, irq_chip, sei);
	if (ret)
		goto free_irq_parents;

	return 0;

free_irq_parents:
	irq_domain_free_irqs_parent(domain, virq, nr_irqs);
release_region:
	if (domain == sei->cp_domain) {
		mutex_lock(&sei->cp_msi_lock);
		bitmap_release_region(sei->cp_msi_bitmap, hwirq, 0);
		mutex_unlock(&sei->cp_msi_lock);
	}

	return ret;
}

static void mvebu_sei_irq_domain_free(struct irq_domain *domain,
				      unsigned int virq, unsigned int nr_irqs)
{
	struct mvebu_sei *sei = domain->host_data;
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	u32 irq_nb = sei->ap_interrupts.size + sei->cp_interrupts.size;

	if (nr_irqs != 1 || d->hwirq >= irq_nb) {
		dev_err(sei->dev, "Invalid hwirq %lu\n", d->hwirq);
		return;
	}

	irq_domain_free_irqs_parent(domain, virq, nr_irqs);

	mutex_lock(&sei->cp_msi_lock);
	bitmap_release_region(sei->cp_msi_bitmap, d->hwirq, 0);
	mutex_unlock(&sei->cp_msi_lock);
}

static int mvebu_sei_ap_match(struct irq_domain *d, struct device_node *node,
			      enum irq_domain_bus_token bus_token)
{
	struct mvebu_sei *sei = d->host_data;

	if (sei->dev->of_node != node)
		return 0;

	if (d == sei->ap_domain)
		return 1;

	return 0;
}

static const struct irq_domain_ops mvebu_sei_ap_domain_ops = {
	.match = mvebu_sei_ap_match,
	.xlate = irq_domain_xlate_onecell,
	.alloc = mvebu_sei_irq_domain_alloc,
	.free = mvebu_sei_irq_domain_free,
};

static const struct irq_domain_ops mvebu_sei_cp_domain_ops = {
	.alloc = mvebu_sei_irq_domain_alloc,
	.free = mvebu_sei_irq_domain_free,
};

static struct irq_chip mvebu_sei_msi_irq_chip = {
	.name		= "SEI MSI controller",
	.irq_set_type	= mvebu_sei_cp_set_type,
};

static struct msi_domain_ops mvebu_sei_msi_ops = {
};

static struct msi_domain_info mvebu_sei_msi_domain_info = {
	.flags	= MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS,
	.ops	= &mvebu_sei_msi_ops,
	.chip	= &mvebu_sei_msi_irq_chip,
};

static void mvebu_sei_handle_cascade_irq(struct irq_desc *desc)
{
	struct mvebu_sei *sei = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	DECLARE_BITMAP(irqmap, SEI_IRQ_COUNT);
	u32 idx, irqn;

	chained_irq_enter(chip, desc);

	/* Create the bitmap of the pending interrupts */
	for (idx = 0; idx < BITS_TO_LONGS(SEI_IRQ_COUNT); idx++)
		irqmap[idx] = readl_relaxed(sei->base + GICP_SECR(idx));

	/* Call handler for each pending interrupt */
	for_each_set_bit(irqn, irqmap, SEI_IRQ_COUNT) {
		u32 virq, hwirq;

		/*
		 * Finding Linux mapping (virq) needs the right domain
		 * and the relative hwirq (which start at 0 in both
		 * cases, while irqn is relative to all SEI interrupts).
		 */
		if (irqn < sei->ap_interrupts.size) {
			hwirq = irqn;
			virq = irq_find_mapping(sei->ap_domain, hwirq);
		} else {
			hwirq = irqn - sei->ap_interrupts.size;
			virq = irq_find_mapping(sei->cp_domain, hwirq);
		}

		/* Call IRQ handler */
		generic_handle_irq(virq);
	}

	/* Clear the pending interrupts by writing 1 to the set bits */
	for (idx = 0; idx < BITS_TO_LONGS(SEI_IRQ_COUNT); idx++)
		if (irqmap[idx])
			writel_relaxed(irqmap[idx], sei->base + GICP_SECR(idx));

	chained_irq_exit(chip, desc);
}

static int mvebu_sei_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node, *parent;
	struct irq_domain *parent_domain, *plat_domain;
	struct mvebu_sei *sei;
	const __be32 *property;
	u32 parent_irq, size;
	int ret;

	sei = devm_kzalloc(&pdev->dev, sizeof(*sei), GFP_KERNEL);
	if (!sei)
		return -ENOMEM;

	sei->dev = &pdev->dev;

	mutex_init(&sei->cp_msi_lock);

	sei->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sei->base = devm_ioremap_resource(sei->dev, sei->res);
	if (!sei->base) {
		dev_err(sei->dev, "Failed to remap SEI resource\n");
		return -ENODEV;
	}

	mvebu_sei_reset(sei);

	/*
	 * Reserve the single (top-level) parent SPI IRQ from which all the
	 * interrupts handled by this driver will be signaled.
	 */
	parent_irq = irq_of_parse_and_map(node, 0);
	if (parent_irq <= 0) {
		dev_err(sei->dev, "Failed to retrieve top-level SPI IRQ\n");
		return -ENODEV;
	}

	/*
	 * SEIs can be triggered from the AP through wired interrupts and from
	 * the CPs through MSIs.
	 */

	/* Get a reference to the parent domain to create a hierarchy */
	parent = of_irq_find_parent(node);
	if (!parent) {
		dev_err(sei->dev, "Failed to find parent IRQ node\n");
		ret = -ENODEV;
		goto dispose_irq;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		dev_err(sei->dev, "Failed to find parent IRQ domain\n");
		ret = -ENODEV;
		goto dispose_irq;
	}

	/*
	 * Retrieve the IRQ organization (AP/CP): the index of the first one and
	 * the number of them for each domain.
	 */
	property = of_get_property(node, "marvell,sei-ap-ranges", &size);
	if (!property || (size != (2 * sizeof(u32)))) {
		dev_err(sei->dev, "Missing 'marvell,sei-ap-ranges' property\n");
		ret = -ENODEV;
		goto dispose_irq;
	}

	sei->ap_interrupts.first = be32_to_cpu(property[0]);
	sei->ap_interrupts.size = be32_to_cpu(property[1]);

	property = of_get_property(node, "marvell,sei-cp-ranges", &size);
	if (!property || (size != (2 * sizeof(u32)))) {
		dev_err(sei->dev, "Missing 'marvell,sei-cp-ranges' property\n");
		ret = -ENODEV;
		goto dispose_irq;
	}

	sei->cp_interrupts.first = be32_to_cpu(property[0]);
	sei->cp_interrupts.size = be32_to_cpu(property[1]);

	/* Create the 'wired' hierarchy */
	sei->ap_domain = irq_domain_create_hierarchy(parent_domain, 0,
						     sei->ap_interrupts.size,
						     of_node_to_fwnode(node),
						     &mvebu_sei_ap_domain_ops,
						     sei);
	if (!sei->ap_domain) {
		dev_err(sei->dev, "Failed to create AP IRQ domain\n");
		ret = -ENOMEM;
		goto dispose_irq;
	}

	/* Create the 'MSI' hierarchy */
	sei->cp_domain = irq_domain_create_hierarchy(parent_domain, 0,
						     sei->cp_interrupts.size,
						     of_node_to_fwnode(node),
						     &mvebu_sei_cp_domain_ops,
						     sei);
	if (!sei->cp_domain) {
		pr_err("Failed to create CPs IRQ domain\n");
		ret = -ENOMEM;
		goto remove_ap_domain;
	}

	plat_domain = platform_msi_create_irq_domain(of_node_to_fwnode(node),
						     &mvebu_sei_msi_domain_info,
						     sei->cp_domain);
	if (!plat_domain) {
		pr_err("Failed to create CPs MSI domain\n");
		ret = -ENOMEM;
		goto remove_cp_domain;
	}

	platform_set_drvdata(pdev, sei);

	irq_set_chained_handler(parent_irq, mvebu_sei_handle_cascade_irq);
	irq_set_handler_data(parent_irq, sei);

	return 0;

remove_cp_domain:
	irq_domain_remove(sei->cp_domain);
remove_ap_domain:
	irq_domain_remove(sei->ap_domain);
dispose_irq:
	irq_dispose_mapping(parent_irq);

	return ret;
}

static const struct of_device_id mvebu_sei_of_match[] = {
	{ .compatible = "marvell,armada-8k-sei", },
	{},
};

static struct platform_driver mvebu_sei_driver = {
	.probe  = mvebu_sei_probe,
	.driver = {
		.name = "mvebu-sei",
		.of_match_table = mvebu_sei_of_match,
	},
};
builtin_platform_driver(mvebu_sei_driver);
