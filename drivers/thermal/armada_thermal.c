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

#define MCELSIUS(t)			(t * 1000)
#define CELSIUS(t)			(t / 1000)

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

#define STATUS_POLL_PERIOD_US		1000
#define STATUS_POLL_TIMEOUT_US		100000

struct armada_thermal_data;

/* Marvell EBU Thermal Sensor Dev Structure */
struct armada_thermal_priv {
	struct device *dev;
	void __iomem *status;
	void __iomem *control0;
	void __iomem *control1;
	void __iomem *dfx;
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
	u32 is_valid_bit;
	bool needs_control0;
	bool dfx_interrupt;
};

inline unsigned int tsen_thresh_val_calc(unsigned int celsius_temp,
					 struct armada_thermal_data *data)
{
	int thresh_val;

	thresh_val = ((MCELSIUS(celsius_temp) * data->coef_div) +
		      data->coef_b) / data->coef_m;

	return thresh_val & data->temp_mask;
}

inline unsigned int tsen_thresh_celsius_calc(int thresh_val,
					     int hyst,
					     struct armada_thermal_data *data)
{
	unsigned int mcelsius_temp;

	mcelsius_temp = (((data->coef_m * (thresh_val + hyst)) -
			  data->coef_b) / data->coef_div);

	return CELSIUS(mcelsius_temp);
}

inline unsigned int ap806_thresh_val_calc(unsigned int celsius_temp,
					  struct armada_thermal_data *data)
{
	int thresh_val;

	thresh_val = (int)((MCELSIUS(celsius_temp) * data->coef_div) -
			   data->coef_b) / (int)data->coef_m;

	/*
	 * TSEN output format is signed as a 2s complement number
	 * ranging from-512 to +511. when MSB is set, need to
	 * calculate the complement number
	 */
	if (thresh_val < 0)
		thresh_val += AP806_TSEN_OUTPUT_COMP;

	return thresh_val & data->temp_mask;
}

inline unsigned int ap806_thresh_celsius_calc(int thresh_val, int hyst,
					      struct armada_thermal_data *data)
{
	unsigned int mcelsius_temp;

	/*
	 * This calculation is the opposite to the calculation done in
	 * ap806_thresh_val_calc(). In order to get the right result,
	 * need to revert the 2s complement calculation,
	 * which occurs when threshold bigger then TSEN max value(511).
	 */
	if (thresh_val >= AP806_TSEN_OUTPUT_MSB)
		thresh_val -= AP806_TSEN_OUTPUT_COMP;

	mcelsius_temp = (((data->coef_m * (thresh_val + hyst)) +
			  data->coef_b) / data->coef_div);

	return CELSIUS(mcelsius_temp);
}

static void tsen_temp_set_threshold(struct platform_device *pdev,
				    struct armada_thermal_priv *priv)
{
	int temp, reg, hyst;
	unsigned int thresh;
	struct armada_thermal_data *data = priv->data;
	struct device_node *np = pdev->dev.of_node;

	/* Get threshold value from DT */
	if (of_property_read_u32(np, "threshold", &thresh)) {
		thresh = THRESH_DEFAULT_TEMP;
		dev_warn(&pdev->dev, "no threshold in DT, using default\n");
	}

	/* Get hysteresis value from DT */
	if (of_property_read_u32(np, "hysteresis", &hyst)) {
		hyst = THRESH_DEFAULT_HYST;
		dev_warn(&pdev->dev, "no hysteresis in DT, using default\n");
	}

	temp = tsen_thresh_val_calc(thresh, data);
	reg = readl_relaxed(priv->control + TSEN_CONTROL_MSB_OFFSET);

	/* Set Threshold */
	reg &= ~(data->temp_mask << TSEN_THRESH_OFFSET);
	reg |= (temp << TSEN_THRESH_OFFSET);

	/* Set Hysteresis */
	reg &= ~(TSEN_THRESH_HYST_MASK << TSEN_THRESH_HYST_OFFSET);
	reg |= (hyst << TSEN_THRESH_HYST_OFFSET);

	writel(reg, priv->control + TSEN_CONTROL_MSB_OFFSET);

	/* Hysteresis calculation is 2^(2+n) */
	hyst = 1 << (hyst + 2);

	temp = tsen_thresh_val_calc(thresh, data);

	dev_info(&pdev->dev, "Overheat threshold between %d..%d\n",
		 tsen_thresh_celsius_calc(temp, -hyst, data),
		 tsen_thresh_celsius_calc(temp, hyst, data));
}

static void ap806_temp_set_threshold(struct platform_device *pdev,
				     struct armada_thermal_priv *priv)
{
	int temp, reg, hyst;
	unsigned int thresh;
	struct armada_thermal_data *data = priv->data;
	struct device_node *np = pdev->dev.of_node;

	/* Get threshold value from DT */
	if (of_property_read_u32(np, "threshold", &thresh)) {
		thresh = THRESH_DEFAULT_TEMP;
		dev_warn(&pdev->dev, "no threshold in DT, using default\n");
	}

	/* Get hysteresis value from DT */
	if (of_property_read_u32(np, "hysteresis", &hyst)) {
		hyst = THRESH_DEFAULT_HYST;
		dev_warn(&pdev->dev, "no hysteresis in DT, using default\n");
	}

	temp = ap806_thresh_val_calc(thresh, data);

	dev_dbg(&pdev->dev, "armada_thermal: Threshold is %d Hyst is %d\n",
		thresh, hyst);

	reg = readl_relaxed(priv->control + TSEN_CONTROL_MSB_OFFSET);

	/* Set Threshold */
	reg &= ~(data->temp_mask << EXT_TSEN_THRESH_OFFSET);
	reg |= (temp << EXT_TSEN_THRESH_OFFSET);

	/* Set Hysteresis */
	reg &= ~(EXT_TSEN_THRESH_HYST_MASK << TSEN_THRESH_HYST_OFFSET);
	reg |= (hyst << EXT_TSEN_THRESH_HYST_OFFSET);

	writel(reg, priv->control + TSEN_CONTROL_MSB_OFFSET);

	/* Hysteresis calculation is 2^(2+n) */
	hyst = 1 << (hyst + 2);

	dev_info(&pdev->dev, "Overheat threshold between %d..%d\n",
		 ap806_thresh_celsius_calc(temp, -hyst, data),
		 ap806_thresh_celsius_calc(temp, hyst, data));
}

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

static void armada380_init_sensor(struct platform_device *pdev,
				  struct armada_thermal_priv *priv)
{
	u32 reg = readl_relaxed(priv->control1);

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

	/* Set thresholds */
	tsen_temp_set_threshold(pdev, priv);

	/* Clear on Read DFX temperature irqs cause */
	reg = readl_relaxed(priv->dfx + 0x10); // +0x8 pour cp110

	/* Unmask DFX Temperature overheat and cooldown irqs */
	reg = readl_relaxed(priv->dfx + 0x14); // 0xc pour cp110
	reg |= (0x3 << 1); // bit 20 (CP110_TSEN_INT_MASK) pour cp110
	writel(reg, priv->dfx + 0x14); // 0xc pour cp110

	/* Unmask DFX Server irq */
	reg = readl_relaxed(priv->dfx + 0x4);
	reg |= (0x3 << 1); // bit(1) pour cp110 (TSEN_INT_SUM_MASK)
	writel(reg, priv->dfx + 0x4);

	/* Wait the sensors to be valid or the core will warn the user */
	armada_wait_sensor_validity(priv);
}

static void armada_ap806_init_sensor(struct platform_device *pdev,
				     struct armada_thermal_priv *priv)
{
	u32 reg;

	reg = readl_relaxed(priv->control0);
	reg &= ~CONTROL0_TSEN_RESET;
	reg |= CONTROL0_TSEN_START | CONTROL0_TSEN_ENABLE;
	writel(reg, priv->control0);

	/* Set thresholds */
	ap806_temp_set_threshold(pdev, priv);

	/* Clear on Read DFX temperature irqs cause */
	reg = readl_relaxed(priv->dfx + 0x8);

	/* Unmask DFX Temperature overheat and cooldown irqs */
	reg = readl_relaxed(priv->dfx + 0xC);
	reg |= AP806_TSEN_INT_MASK;
	writel(reg, priv->dfx + 0xC);

	/* Unmask DFX Server irq */
	reg = readl_relaxed(priv->dfx + 0x4);
	reg |= TSEN_INT_SUM_MASK;
	writel(reg, priv->dfx + 0x4);

	/* Wait the sensors to be valid or the core will warn the user */
	armada_wait_sensor_validity(priv);
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

static irqreturn_t a38x_temp_irq_handler(int irq, void *data)
{
	struct armada_thermal_priv *priv = (struct armada_thermal_priv *)data;
	u32 reg;

	/* Mask Temp irq */
	reg = readl_relaxed(priv->dfx + 0x14);
	reg &= ~(0x3 << 1);
	writel(reg, priv->dfx + 0x14);

	/* Clear Temp irq cause */
	reg = readl_relaxed(priv->dfx + 0x10);

	if (reg & (0x3 << 1))
		dev_warn(priv->dev, "Overheat critical %s threshold temperature reached\n",
			 (reg & (1 << 1)) ? "high" : "low");

	/* UnMask Temp irq */
	reg = readl_relaxed(priv->dfx + 0x14);
	reg |= (0x3 << 1);
	writel(reg, priv->dfx + 0x14);

	return IRQ_HANDLED;
}

static irqreturn_t ap806_temp_irq_handler(int irq, void *data)
{
	struct armada_thermal_priv *priv = (struct armada_thermal_priv *)data;
	u32 reg;

	/* Mask Temp irq */
	reg = readl_relaxed(priv->dfx + 0xC);
	reg &= ~AP806_TSEN_INT_MASK;
	writel(reg, priv->dfx + 0xC);

	/* Read & Clear Temp irq cause */
	reg = readl_relaxed(priv->dfx + 0x8);

	if (reg & AP806_TSEN_INT_MASK)
		dev_warn(priv->dev, "Overheat critical high threshold temperature reached\n");

	/* UnMask Temp irq */
	reg = readl_relaxed(priv->dfx + 0xC);
	reg |= AP806_TSEN_INT_MASK;
	writel(reg, priv->dfx + 0xC);

	return IRQ_HANDLED;
}

irqreturn_t cp110_temp_irq_handler(int irq, void *data)
{
	struct armada_thermal_priv *priv = (struct armada_thermal_priv *)data;
	u32 reg;

	/* Mask Temp irq */
	reg = readl_relaxed(priv->dfx + 0xC);
	reg &= ~CP110_TSEN_INT_MASK;
	writel(reg, priv->dfx + 0xC);

	/* Clear Temp irq cause */
	reg = readl_relaxed(priv->dfx + 0x8);

	if (reg & CP110_TSEN_INT_MASK)
		dev_warn(priv->dev, "Overheat critical high threshold temperature reached\n");

	/* UnMask Temp irq */
	reg = readl_relaxed(priv->dfx + 0xC);
	reg |= CP110_TSEN_INT_MASK;
	writel(reg, priv->dfx + 0xC);

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
	.temp_irq_handler = a38x_temp_irq_handler,
	.is_valid_bit = BIT(10),
	.temp_shift = 0,
	.temp_mask = 0x3ff,
	.coef_b = 1172499100ULL,
	.coef_m = 2000096ULL,
	.coef_div = 4201,
	.inverted = true,
	.dfx_interrupt = 1,
};

static const struct armada_thermal_data armada_ap806_data = {
	.is_valid = armada_is_valid,
	.init_sensor = armada_ap806_init_sensor,
	.temp_irq_handler = ap806_temp_irq_handler,
	.is_valid_bit = BIT(16),
	.temp_shift = 0,
	.temp_mask = 0x3ff,
	.coef_b = -150000LL,
	.coef_m = 423ULL,
	.coef_div = 1,
	.inverted = true,
	.signed_sample = true,
	.needs_control0 = true,
	.dfx_interrupt = 1,
};

static const struct armada_thermal_data armada_cp110_data = {
	.is_valid = armada_is_valid,
	.init_sensor = armada380_init_sensor,
	.temp_irq_handler = cp110_temp_irq_handler,
	.is_valid_bit = BIT(10),
	.temp_shift = 0,
	.temp_mask = 0x3ff,
	.coef_b = 1172499100ULL,
	.coef_m = 2000096ULL,
	.coef_div = 4201,
	.inverted = true,
	.needs_control0 = true,
	.dfx_interrupt = 1,
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

	if (priv->data->dfx_interrupt) {
		/* DFX interrupts are supported by some of the devices */
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		priv->dfx = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(priv->dfx))
			return PTR_ERR(priv->dfx);
	}

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

	priv->data->init_sensor(pdev, priv);

	thermal = thermal_zone_device_register(dev_name(&pdev->dev), 0, 0, priv,
					       &ops, NULL, 0, 0);
	if (IS_ERR(thermal)) {
		dev_err(&pdev->dev,
			"Failed to register thermal zone device\n");
		return PTR_ERR(thermal);
	}

	/* Register overheat interrupt */
	irq = platform_get_irq(pdev, 0);
	if (irq >= 0) {
		if (priv->data->temp_irq_handler == NULL)
			dev_warn(&pdev->dev, "Interrupt handler was not implemented.\n");

		/* Register handler */
		if (devm_request_irq(&pdev->dev, irq,
				     priv->data->temp_irq_handler,
				     0, pdev->name, priv) < 0)
			dev_warn(&pdev->dev,
				 "Interrupt %d is not available.\n", irq);
	} else {
		dev_dbg(&pdev->dev, "armada_thermal: no irq was assigned\n");
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
