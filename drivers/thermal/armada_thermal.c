/*
 * Marvell EBU Armada SoCs thermal sensor driver
 *
 * Copyright (C) 2013 Marvell
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/thermal.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#define TO_MCELSIUS(c)			(c * 1000)

/* Thermal Manager Control and Status Register */
#define PMU_TDC0_SW_RST_MASK		(0x1 << 1)
#define PMU_TM_DISABLE_OFFS		0
#define PMU_TM_DISABLE_MASK		(0x1 << PMU_TM_DISABLE_OFFS)
#define PMU_TDC0_REF_CAL_CNT_OFFS	11
#define PMU_TDC0_REF_CAL_CNT_MASK	(0x1ff << PMU_TDC0_REF_CAL_CNT_OFFS)
#define PMU_TDC0_OTF_CAL_MASK		(0x1 << 30)
#define PMU_TDC0_START_CAL_MASK		(0x1 << 25)

#define A375_UNIT_CONTROL_SHIFT		27
#define A375_UNIT_CONTROL_MASK		0x7
#define A375_READOUT_INVERT		BIT(15)
#define A375_HW_RESETn			BIT(8)

/* Legacy bindings */
#define LEGACY_CONTROL_MEM_LEN		0x4

/* Current bindings with the 2 control registers under the same memory area */
#define LEGACY_CONTROL1_OFFSET		0x0
#define CONTROL0_OFFSET			0x0
#define CONTROL1_OFFSET			0x4

/* Errata fields */
#define CONTROL0_TSEN_TC_TRIM_MASK	0x7
#define CONTROL0_TSEN_TC_TRIM_VAL	0x3

/* TSEN refers to the temperature sensors within the AP */
#define CONTROL0_TSEN_START		BIT(0)
#define CONTROL0_TSEN_RESET		BIT(1)
#define CONTROL0_TSEN_ENABLE		BIT(2)

/* EXT_TSEN refers to the external temperature sensors, out of the AP */
#define CONTROL1_EXT_TSEN_SW_RESET	BIT(7)
#define CONTROL1_EXT_TSEN_HW_RESETn	BIT(8)
#define CONTROL1_EXT_TSEN_INT_EN	BIT(25) //todo: check this is always valid

#define STATUS_POLL_PERIOD_US		1000
#define STATUS_POLL_TIMEOUT_US		100000

struct armada_thermal_data;

/* Marvell EBU Thermal Sensor Dev Structure */
struct armada_thermal_priv {
	struct device *dev;
	void __iomem *status;
	void __iomem *control0;
	void __iomem *control1;
	struct regmap *dfx;
	struct armada_thermal_data *data;
};

struct armada_thermal_data {
	/* Initialize the sensor */
	void (*init_sensor)(struct platform_device *pdev,
			    struct armada_thermal_priv *);

	/* Test for a valid sensor value (optional) */
	bool (*is_valid)(struct armada_thermal_priv *);

	/* Formula coeficients: temp = (b - m * reg) / div */
	s64 coef_b;
	s64 coef_m;
	u32 coef_div;
	bool inverted;
	bool signed_sample;

	/* Register shift and mask to access the sensor temperature */
	unsigned int temp_shift;
	unsigned int temp_mask;
	unsigned int thresh_shift;
	unsigned int hyst_shift;
	unsigned int hyst_mask;
	u32 is_valid_bit;
	bool needs_control0;

	/* DFX syscon access */
	unsigned int dfx_irq_cause_reg;
	unsigned int dfx_irq_mask_reg;
	unsigned int dfx_overheat_irq;
	unsigned int dfx_server_irq_mask_reg;
	unsigned int dfx_server_irq_en;
};

static void armadaxp_init_sensor(struct platform_device *pdev,
				 struct armada_thermal_priv *priv)
{
	u32 reg;

	reg = readl_relaxed(priv->control1);
	reg |= PMU_TDC0_OTF_CAL_MASK;
	writel(reg, priv->control1);

	/* Reference calibration value */
	reg &= ~PMU_TDC0_REF_CAL_CNT_MASK;
	reg |= (0xf1 << PMU_TDC0_REF_CAL_CNT_OFFS);
	writel(reg, priv->control1);

	/* Reset the sensor */
	reg = readl_relaxed(priv->control1);
	writel((reg | PMU_TDC0_SW_RST_MASK), priv->control1);

	writel(reg, priv->control1);

	/* Enable the sensor */
	reg = readl_relaxed(priv->status);
	reg &= ~PMU_TM_DISABLE_MASK;
	writel(reg, priv->status);
}

static void armada370_init_sensor(struct platform_device *pdev,
				  struct armada_thermal_priv *priv)
{
	u32 reg;

	reg = readl_relaxed(priv->control1);
	reg |= PMU_TDC0_OTF_CAL_MASK;
	writel(reg, priv->control1);

	/* Reference calibration value */
	reg &= ~PMU_TDC0_REF_CAL_CNT_MASK;
	reg |= (0xf1 << PMU_TDC0_REF_CAL_CNT_OFFS);
	writel(reg, priv->control1);

	reg &= ~PMU_TDC0_START_CAL_MASK;
	writel(reg, priv->control1);

	msleep(10);
}

static void armada375_init_sensor(struct platform_device *pdev,
				  struct armada_thermal_priv *priv)
{
	u32 reg;

	reg = readl(priv->control1);
	reg &= ~(A375_UNIT_CONTROL_MASK << A375_UNIT_CONTROL_SHIFT);
	reg &= ~A375_READOUT_INVERT;
	reg &= ~A375_HW_RESETn;

	writel(reg, priv->control1);
	msleep(20);

	reg |= A375_HW_RESETn;
	writel(reg, priv->control1);
	msleep(50);
}

static void armada_wait_sensor_validity(struct armada_thermal_priv *priv)
{
	u32 reg;

	readl_relaxed_poll_timeout(priv->status, reg,
				   reg & priv->data->is_valid_bit,
				   STATUS_POLL_PERIOD_US,
				   STATUS_POLL_TIMEOUT_US);
}

static unsigned int armada_celsius_to_reg_temp(struct armada_thermal_data *data,
					       unsigned int celsius)
{
	s64 b = data->coef_b;
	s64 m = data->coef_m;
	s64 div = data->coef_div;
	s64 temp = TO_MCELSIUS(celsius);
	unsigned int sample;

	if (data->inverted)
		sample = div_s64(((temp * div) + b), m);
	else
		sample = div_s64((b - (temp * div)), m);

	return sample & data->temp_mask;
}

static unsigned int armada_celsius_to_reg_hyst(struct armada_thermal_data *data,
					       unsigned int celsius)
{
	/*
	 * The documentation states:
	 * high/low watermark = threshold +/- 0.4761 * 2^(hysteresis + 2)
	 * which is the mathematical derivation for:
	 * 0x0 <=> 1.9°C, 0x1 <=> 3.8°C, 0x2 <=> 7.6°C, 0x3 <=> 15.2
	 */
	unsigned int hyst_levels_mc[] = {1900, 3800, 7600, 15200};
	unsigned int hyst_mc = TO_MCELSIUS(celsius);
	int i;

	/*
	 * We will always take the smallest possible hysteresis to avoid risking
	 * the hardware integrity by enlarging the threshold by +8°C in the
	 * worst case.
	 */
	for (i = sizeof(hyst_levels_mc) - 1; i > 0; i--)
		if (hyst_mc >= hyst_levels_mc[i])
			break;

	return i & data->hyst_mask;
}

static void armada_set_threshold(struct armada_thermal_priv *priv,
				 unsigned int thresh_c, unsigned int hyst_c)
{
	struct armada_thermal_data *data = priv->data;
	unsigned int threshold = armada_celsius_to_reg_temp(data, thresh_c);
	unsigned int hysteresis = armada_celsius_to_reg_hyst(data, hyst_c);
	u32 ctrl1 = readl_relaxed(priv->control1);

	/* Set Threshold */
	ctrl1 &= ~(data->temp_mask << data->thresh_shift);
	ctrl1 |= threshold << data->thresh_shift;

	/* Set Hysteresis */
	ctrl1 &= ~(data->hyst_mask << data->hyst_shift);
	ctrl1 |= hysteresis << data->hyst_shift;

	writel(ctrl1, priv->control1);
}

static void armada_enable_overheat_interrupt(struct armada_thermal_priv *priv)
{
	struct armada_thermal_data *data = priv->data;
	u32 reg;

	if (!priv->dfx) {
		/*
		 * This is only shown at info level because a board with
		 * multiple CP will have a single interrupt line for the
		 * overheat interrupt and once this interrupt is
		 * reserved, others CP will not be able to probe their
		 * own overheat interrupt.
		 */
		dev_info(priv->dev,
			 "Syscon/overheat interrupt not available\n");
		return;
	}

	/* Clear DFX temperature IRQ cause */
	regmap_read(priv->dfx, data->dfx_irq_cause_reg, &reg);

	/* Enable DFX Temperature IRQ */
	regmap_read(priv->dfx, data->dfx_irq_mask_reg, &reg);
	reg |= data->dfx_overheat_irq;
	regmap_write(priv->dfx, data->dfx_irq_mask_reg, reg);

	/* Enable DFX server IRQ */
	regmap_read(priv->dfx, data->dfx_server_irq_mask_reg, &reg);
	reg |= data->dfx_server_irq_en;
	regmap_write(priv->dfx, data->dfx_server_irq_mask_reg, reg);

	/* Enable overheat interrupt */
	writel_relaxed(readl(priv->control1) | CONTROL1_EXT_TSEN_INT_EN,
		       priv->control1);
}

static void armada380_init_sensor(struct platform_device *pdev,
				  struct armada_thermal_priv *priv)
{
	u32 reg = readl_relaxed(priv->control1);
	//todo: clean threshold/hyst
	unsigned int threshold = 46, hysteresis = 2;

	/* Disable the HW/SW reset */
	reg |= CONTROL1_EXT_TSEN_HW_RESETn;
	reg &= ~CONTROL1_EXT_TSEN_SW_RESET;
	writel(reg, priv->control1);

	/* Set Tsen Tc Trim to correct default value (errata #132698) */
	if (priv->control0) {
		reg = readl_relaxed(priv->control0);
		reg &= ~CONTROL0_TSEN_TC_TRIM_MASK;
		reg |= CONTROL0_TSEN_TC_TRIM_VAL;
		writel(reg, priv->control0);
	}

	armada_set_threshold(priv, threshold, hysteresis);

	/* Wait the sensors to be valid or the core will warn the user */
	armada_wait_sensor_validity(priv);

	armada_enable_overheat_interrupt(priv);
}

static void armada_ap806_init_sensor(struct platform_device *pdev,
				     struct armada_thermal_priv *priv)
{
	//todo: clean threshold/hyst
	unsigned int threshold = 44, hysteresis = 2;
	u32 reg;

	reg = readl_relaxed(priv->control0);
	reg &= ~CONTROL0_TSEN_RESET;
	reg |= CONTROL0_TSEN_START | CONTROL0_TSEN_ENABLE;
	writel(reg, priv->control0);

	armada_set_threshold(priv, threshold, hysteresis);

	/* Wait the sensors to be valid or the core will warn the user */
	armada_wait_sensor_validity(priv);

	armada_enable_overheat_interrupt(priv);
}

static bool armada_is_valid(struct armada_thermal_priv *priv)
{
	u32 reg = readl_relaxed(priv->status);

	return reg & priv->data->is_valid_bit;
}

static int armada_get_temp(struct thermal_zone_device *thermal,
			   int *temp)
{
	struct armada_thermal_priv *priv = thermal->devdata;
	u32 reg, div;
	s64 sample, b, m;

	/* Valid check */
	if (priv->data->is_valid && !priv->data->is_valid(priv)) {
		dev_err(&thermal->device,
			"Temperature sensor reading not valid\n");
		return -EIO;
	}

	reg = readl_relaxed(priv->status);
	reg = (reg >> priv->data->temp_shift) & priv->data->temp_mask;
	if (priv->data->signed_sample)
		/* The most significant bit is the sign bit */
		sample = sign_extend32(reg, fls(priv->data->temp_mask) - 1);
	else
		sample = reg;

	/* Get formula coeficients */
	b = priv->data->coef_b;
	m = priv->data->coef_m;
	div = priv->data->coef_div;

	if (priv->data->inverted)
		*temp = div_s64((m * sample) - b, div);
	else
		*temp = div_s64(b - (m * sample), div);

	return 0;
}

static struct thermal_zone_device_ops ops = {
	.get_temp = armada_get_temp,
};

static irqreturn_t armada_overheat_irq_handler(int irq, void *blob)
{
	struct armada_thermal_priv *priv = (struct armada_thermal_priv *)blob;
	struct armada_thermal_data *data = priv->data;
	u32 reg;

	/* Mask DFX Temperature overheat IRQ */
	regmap_read(priv->dfx, data->dfx_irq_mask_reg, &reg);
	reg &= ~data->dfx_overheat_irq;
	regmap_write(priv->dfx, data->dfx_irq_mask_reg, reg);

	/* Clear DFX temperature IRQs cause */
	regmap_read(priv->dfx, data->dfx_irq_cause_reg, &reg);
	if (reg & data->dfx_overheat_irq)
		dev_warn(priv->dev,
			 "Overheat critical threshold temperature reached\n");

	/* Unmask DFX Temperature overheat IRQ */
	regmap_read(priv->dfx, data->dfx_irq_mask_reg, &reg);
	reg |= data->dfx_overheat_irq;
	regmap_write(priv->dfx, data->dfx_irq_mask_reg, reg);

	return IRQ_HANDLED;
}

static const struct armada_thermal_data armadaxp_data = {
	.init_sensor = armadaxp_init_sensor,
	.temp_shift = 10,
	.temp_mask = 0x1ff,
	.coef_b = 3153000000ULL,
	.coef_m = 10000000ULL,
	.coef_div = 13825,
};

static const struct armada_thermal_data armada370_data = {
	.is_valid = armada_is_valid,
	.init_sensor = armada370_init_sensor,
	.is_valid_bit = BIT(9),
	.temp_shift = 10,
	.temp_mask = 0x1ff,
	.coef_b = 3153000000ULL,
	.coef_m = 10000000ULL,
	.coef_div = 13825,
};

static const struct armada_thermal_data armada375_data = {
	.is_valid = armada_is_valid,
	.init_sensor = armada375_init_sensor,
	.is_valid_bit = BIT(10),
	.temp_shift = 0,
	.temp_mask = 0x1ff,
	.coef_b = 3171900000ULL,
	.coef_m = 10000000ULL,
	.coef_div = 13616,
	.needs_control0 = true,
};

static const struct armada_thermal_data armada380_data = {
	.is_valid = armada_is_valid,
	.init_sensor = armada380_init_sensor,
	.is_valid_bit = BIT(10),
	.temp_shift = 0,
	.temp_mask = 0x3ff,
	.thresh_shift = 16,
	.hyst_shift = 26,
	.hyst_mask = 0x3,
	.coef_b = 1172499100ULL,
	.coef_m = 2000096ULL,
	.coef_div = 4201,
	.inverted = true,
	.dfx_irq_cause_reg = 0x10,
	.dfx_irq_mask_reg = 0x14,
	.dfx_overheat_irq = BIT(1), /* todo: maybe BIT(2) enables a "low threshold" int */
	.dfx_server_irq_en = BIT(1), /* todo same as above */
	.dfx_server_irq_mask_reg = 0x4,
};

static const struct armada_thermal_data armada_ap806_data = {
	.is_valid = armada_is_valid,
	.init_sensor = armada_ap806_init_sensor,
	.is_valid_bit = BIT(16),
	.temp_shift = 0,
	.temp_mask = 0x3ff,
	.thresh_shift = 3,
	.hyst_shift = 19,
	.hyst_mask = 0x3,
	.coef_b = -150000LL,
	.coef_m = 423ULL,
	.coef_div = 1,
	.inverted = true,
	.signed_sample = true,
	.needs_control0 = true,
	.dfx_irq_cause_reg = 0x8,
	.dfx_irq_mask_reg = 0xC,
	.dfx_overheat_irq = BIT(22),
	.dfx_server_irq_mask_reg = 0x4,
	.dfx_server_irq_en = BIT(1),
};

static const struct armada_thermal_data armada_cp110_data = {
	.is_valid = armada_is_valid,
	.init_sensor = armada380_init_sensor,
	.is_valid_bit = BIT(10),
	.temp_shift = 0,
	.temp_mask = 0x3ff,
	.thresh_shift = 16,
	.hyst_shift = 26,
	.hyst_mask = 0x3,
	.coef_b = 1172499100ULL,
	.coef_m = 2000096ULL,
	.coef_div = 4201,
	.inverted = true,
	.needs_control0 = true,
	.dfx_irq_cause_reg = 0x8,
	.dfx_irq_mask_reg = 0xC,
	.dfx_overheat_irq = BIT(20),
	.dfx_server_irq_mask_reg = 0x4,
	.dfx_server_irq_en = BIT(1),
};

static const struct of_device_id armada_thermal_id_table[] = {
	{
		.compatible = "marvell,armadaxp-thermal",
		.data       = &armadaxp_data,
	},
	{
		.compatible = "marvell,armada370-thermal",
		.data       = &armada370_data,
	},
	{
		.compatible = "marvell,armada375-thermal",
		.data       = &armada375_data,
	},
	{
		.compatible = "marvell,armada380-thermal",
		.data       = &armada380_data,
	},
	{
		.compatible = "marvell,armada-ap806-thermal",
		.data       = &armada_ap806_data,
	},
	{
		.compatible = "marvell,armada-cp110-thermal",
		.data       = &armada_cp110_data,
	},
	{
		/* sentinel */
	},
};
MODULE_DEVICE_TABLE(of, armada_thermal_id_table);

static int armada_thermal_probe(struct platform_device *pdev)
{
	void __iomem *control = NULL;
	struct thermal_zone_device *thermal;
	const struct of_device_id *match;
	struct armada_thermal_priv *priv;
	struct resource *res;
	int irq;

	match = of_match_device(armada_thermal_id_table, &pdev->dev);
	if (!match)
		return -ENODEV;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Save device in data structure */
	priv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->status = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->status))
		return PTR_ERR(priv->status);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	control = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(control))
		return PTR_ERR(control);

	priv->data = (struct armada_thermal_data *)match->data;

	/*
	 * Legacy DT bindings only described "control1" register (also referred
	 * as "control MSB" on old documentation). New bindings cover
	 * "control0/control LSB" and "control1/control MSB" registers within
	 * the same resource, which is then of size 8 instead of 4.
	 */
	if (resource_size(res) == LEGACY_CONTROL_MEM_LEN) {
		/* ->control0 unavailable in this configuration */
		if (priv->data->needs_control0) {
			dev_err(&pdev->dev, "No access to control0 register\n");
			return -EINVAL;
		}

		priv->control1 = control + LEGACY_CONTROL1_OFFSET;
	} else {
		priv->control0 = control + CONTROL0_OFFSET;
		priv->control1 = control + CONTROL1_OFFSET;
	}

	/* DFX syscon may be used for overheat interrutps handling */
	printk("---> %s\n", dev_name(&pdev->dev));
	priv->dfx = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						    "marvell,dfx-system-controller");
	irq = platform_get_irq(pdev, 0);
	if (IS_ERR(priv->dfx) || irq < 0 ||
	    devm_request_irq(&pdev->dev, irq, armada_overheat_irq_handler, 0,
			     pdev->name, priv))
		priv->dfx = NULL;

	priv->data->init_sensor(pdev, priv);

	thermal = thermal_zone_device_register(dev_name(&pdev->dev), 0, 0, priv,
					       &ops, NULL, 0, 0);
	if (IS_ERR(thermal)) {
		dev_err(&pdev->dev,
			"Failed to register thermal zone device\n");
		return PTR_ERR(thermal);
	}

	platform_set_drvdata(pdev, thermal);

	return 0;
}

static int armada_thermal_exit(struct platform_device *pdev)
{
	struct thermal_zone_device *armada_thermal =
		platform_get_drvdata(pdev);

	thermal_zone_device_unregister(armada_thermal);

	return 0;
}

static struct platform_driver armada_thermal_driver = {
	.probe = armada_thermal_probe,
	.remove = armada_thermal_exit,
	.driver = {
		.name = "armada_thermal",
		.of_match_table = armada_thermal_id_table,
	},
};

module_platform_driver(armada_thermal_driver);

MODULE_AUTHOR("Ezequiel Garcia <ezequiel.garcia@free-electrons.com>");
MODULE_DESCRIPTION("Marvell EBU Armada SoCs thermal driver");
MODULE_LICENSE("GPL v2");
