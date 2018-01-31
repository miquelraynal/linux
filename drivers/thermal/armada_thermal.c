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
#include <linux/workqueue.h>

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
#define CONTROL1_EXT_TSEN_INT_EN	BIT(25) //todo: check this is still valid for a38x

#define STATUS_POLL_PERIOD_US		1000
#define STATUS_POLL_TIMEOUT_US		100000
#define OVERHEAT_INT_POLL_DELAY		(1 * HZ)

struct armada_thermal_data;

/* Marvell EBU Thermal Sensor Dev Structure */
struct armada_thermal_priv {
	struct platform_device *pdev;
	struct regmap *syscon;
	struct armada_thermal_data *data;
	struct delayed_work wait_end_overheat_work;
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

	/* Syscon access */
	unsigned int syscon_control0_off;
	unsigned int syscon_control1_off;
	unsigned int syscon_status_off;
	unsigned int dfx_irq_cause_off;
	unsigned int dfx_irq_mask_off;
	unsigned int dfx_overheat_irq;
	unsigned int dfx_server_irq_mask_off;
	unsigned int dfx_server_irq_en;
};

struct armada_thermal_priv *work_to_thermal(struct work_struct *work)
{
	return container_of(work, struct armada_thermal_priv,
			    wait_end_overheat_work.work);
}

static void armadaxp_init_sensor(struct platform_device *pdev,
				 struct armada_thermal_priv *priv)
{
	struct armada_thermal_data *data = priv->data;
	u32 reg;

	regmap_read(priv->syscon, data->syscon_control1_off, &reg);
	reg |= PMU_TDC0_OTF_CAL_MASK;
	regmap_write(priv->syscon, data->syscon_control1_off, reg);

	/* Reference calibration value */
	reg &= ~PMU_TDC0_REF_CAL_CNT_MASK;
	reg |= (0xf1 << PMU_TDC0_REF_CAL_CNT_OFFS);
	regmap_write(priv->syscon, data->syscon_control1_off, reg);

	/* Reset the sensor */
	regmap_read(priv->syscon, data->syscon_control1_off, &reg);
	reg |= PMU_TDC0_SW_RST_MASK;
	regmap_write(priv->syscon, data->syscon_control1_off, reg);

	/* Enable the sensor */
	regmap_read(priv->syscon, data->syscon_status_off, &reg);
	reg &= ~PMU_TM_DISABLE_MASK;
	regmap_write(priv->syscon, data->syscon_status_off, reg);
}

static void armada370_init_sensor(struct platform_device *pdev,
				  struct armada_thermal_priv *priv)
{
	struct armada_thermal_data *data = priv->data;
	u32 reg;

	regmap_read(priv->syscon, data->syscon_control1_off, &reg);
	reg |= PMU_TDC0_OTF_CAL_MASK;
	regmap_write(priv->syscon, data->syscon_control1_off, reg);

	/* Reference calibration value */
	reg &= ~PMU_TDC0_REF_CAL_CNT_MASK;
	reg |= (0xf1 << PMU_TDC0_REF_CAL_CNT_OFFS);
	regmap_write(priv->syscon, data->syscon_control1_off, reg);

	/* Reset the sensor */
	reg &= ~PMU_TDC0_START_CAL_MASK;
	regmap_write(priv->syscon, data->syscon_control1_off, reg);

	msleep(10);
}

static void armada375_init_sensor(struct platform_device *pdev,
				  struct armada_thermal_priv *priv)
{
	struct armada_thermal_data *data = priv->data;
	u32 reg;

	regmap_read(priv->syscon, data->syscon_control1_off, &reg);
	reg &= ~(A375_UNIT_CONTROL_MASK << A375_UNIT_CONTROL_SHIFT);
	reg &= ~A375_READOUT_INVERT;
	reg &= ~A375_HW_RESETn;
	regmap_write(priv->syscon, data->syscon_control1_off, reg);

	msleep(20);

	reg |= A375_HW_RESETn;
	regmap_write(priv->syscon, data->syscon_control1_off, reg);

	msleep(50);
}

static void armada_wait_sensor_validity(struct armada_thermal_priv *priv)
{
	u32 reg;

	regmap_read_poll_timeout(priv->syscon, priv->data->syscon_status_off,
				 reg, reg & priv->data->is_valid_bit,
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
	u32 ctrl1;

	regmap_read(priv->syscon, data->syscon_control1_off, &ctrl1);

	/* Set Threshold */
	ctrl1 &= ~(data->temp_mask << data->thresh_shift);
	ctrl1 |= threshold << data->thresh_shift;

	/* Set Hysteresis */
	ctrl1 &= ~(data->hyst_mask << data->hyst_shift);
	ctrl1 |= hysteresis << data->hyst_shift;

	regmap_write(priv->syscon, data->syscon_control1_off, ctrl1);
}

static void armada_enable_overheat_interrupt(struct armada_thermal_priv *priv)
{
	struct armada_thermal_data *data = priv->data;
	u32 reg;

	/* Clear DFX temperature IRQ cause */
	regmap_read(priv->syscon, data->dfx_irq_cause_off, &reg);

	/* Enable DFX Temperature IRQ */
	regmap_read(priv->syscon, data->dfx_irq_mask_off, &reg);
	reg |= data->dfx_overheat_irq;
	regmap_write(priv->syscon, data->dfx_irq_mask_off, reg);

	/* Enable DFX server IRQ */
	regmap_read(priv->syscon, data->dfx_server_irq_mask_off, &reg);
	reg |= data->dfx_server_irq_en;
	regmap_write(priv->syscon, data->dfx_server_irq_mask_off, reg);

	/* Enable overheat interrupt */
	regmap_read(priv->syscon, data->syscon_control1_off, &reg);
	reg |= CONTROL1_EXT_TSEN_INT_EN;
	regmap_write(priv->syscon, data->syscon_control1_off, reg);
}

static void armada380_init_sensor(struct platform_device *pdev,
				  struct armada_thermal_priv *priv)
{
	struct armada_thermal_data *data = priv->data;
	u32 reg;
	//todo: clean threshold/hyst
	unsigned int threshold = 53, hysteresis = 2;

	/* Disable the HW/SW reset */
	regmap_read(priv->syscon, data->syscon_control1_off, &reg);
	reg |= CONTROL1_EXT_TSEN_HW_RESETn;
	reg &= ~CONTROL1_EXT_TSEN_SW_RESET;
	regmap_write(priv->syscon, data->syscon_control1_off, reg);

	/* Set Tsen Tc Trim to correct default value (errata #132698) */
	regmap_read(priv->syscon, data->syscon_control0_off, &reg);
	reg &= ~CONTROL0_TSEN_TC_TRIM_MASK;
	reg |= CONTROL0_TSEN_TC_TRIM_VAL;
	regmap_write(priv->syscon, data->syscon_control0_off, reg);

	armada_set_threshold(priv, threshold, hysteresis);

	/* Wait the sensors to be valid or the core will warn the user */
	armada_wait_sensor_validity(priv);

	armada_enable_overheat_interrupt(priv);
}

static void armada_ap806_init_sensor(struct platform_device *pdev,
				     struct armada_thermal_priv *priv)
{
	struct armada_thermal_data *data = priv->data;
	//todo: clean threshold/hyst
	unsigned int threshold = 50, hysteresis = 2;
	u32 reg;

	regmap_read(priv->syscon, data->syscon_control0_off, &reg);
	reg &= ~CONTROL0_TSEN_RESET;
	reg |= CONTROL0_TSEN_START | CONTROL0_TSEN_ENABLE;
	regmap_write(priv->syscon, data->syscon_control0_off, reg);

	armada_set_threshold(priv, threshold, hysteresis);

	/* Wait the sensors to be valid or the core will warn the user */
	armada_wait_sensor_validity(priv);

	armada_enable_overheat_interrupt(priv);
}

static bool armada_is_valid(struct armada_thermal_priv *priv)
{
	u32 reg;

	regmap_read(priv->syscon, priv->data->syscon_status_off, &reg);

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

	regmap_read(priv->syscon, priv->data->syscon_status_off, &reg);
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

static bool armada_temp_is_over_threshold(struct armada_thermal_priv * priv)
{
	struct armada_thermal_data *data = priv->data;
	u32 reg;

	/*
	 * If an overheat interrupt occurred, poll DFX overheat IRQ cause
	 * register until the right bit gets cleared
	 */
	regmap_read(priv->syscon, data->dfx_irq_cause_off, &reg);
	if (reg & data->dfx_overheat_irq) {
		/*
		 * Threshold reached, as CP share the same interrupt, we need to
		 * mask the overheat interrupt until the temperature reaches
		 * back the low-threshold, otherwise if another CP reaches its
		 * threshold temperature, it will re-trigger an interrupt for
		 * CP that are already over heating.
		 */
		regmap_read(priv->syscon, data->dfx_irq_mask_off, &reg);
		reg &= ~data->dfx_overheat_irq;
		regmap_write(priv->syscon, data->dfx_irq_mask_off, reg);

		schedule_delayed_work(&priv->wait_end_overheat_work,
				      OVERHEAT_INT_POLL_DELAY);
		return true;
	}

	/*
	 * The temperature level is acceptable again, unmask the overheat
	 * interrupt.
	 */
	regmap_read(priv->syscon, data->dfx_irq_mask_off, &reg);
	reg |= data->dfx_overheat_irq;
	regmap_write(priv->syscon, data->dfx_irq_mask_off, reg);

	return false;
}

/*
 * Overheat interrupt must be cleared by reading the DFX interrupt cause after
 * the temperature has fallen down to the low threshold, otherwise future
 * interrupts will not be served. This work polls the corresponding register
 * until the overheat flag gets cleared.
 */
static void armada_wait_end_overheat(struct work_struct *work)
{
	armada_temp_is_over_threshold(work_to_thermal(work));
}

static irqreturn_t armada_overheat_isr(int irq, void *blob)
{
	struct armada_thermal_priv *priv = (struct armada_thermal_priv *)blob;
	struct thermal_zone_device *thermal = platform_get_drvdata(priv->pdev);
	u32 reg;
	int temp;

	/*
	 * Check if the interrupt is masked or not. As the line is shared
	 * between CP, this handler will be executed again when another CP will
	 * trigger an overheat interrupt. Once the interrupt is triggered for
	 * one CP, it is masked in the DFX register until the temperature
	 * reaches under the threshold, when it is unmasked.
	 */
	regmap_read(priv->syscon, priv->data->dfx_irq_mask_off, &reg);
	if (!(reg & priv->data->dfx_overheat_irq))
		return IRQ_NONE;

	if (armada_temp_is_over_threshold(priv)) {
		armada_get_temp(thermal, &temp);
		dev_warn(&priv->pdev->dev,
			 "critical temperature reached (Celsius): %d\n",
			 temp / 1000);
	}

	return IRQ_HANDLED;
}

static const struct armada_thermal_data armadaxp_data = {
	.init_sensor = armadaxp_init_sensor,
	.temp_shift = 10,
	.temp_mask = 0x1ff,
	.coef_b = 3153000000ULL,
	.coef_m = 10000000ULL,
	.coef_div = 13825,
	.syscon_status_off = 0xb0,
	.syscon_control1_off = 0xd0,
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
	.syscon_status_off = 0x0,
	.syscon_control1_off = 0x4,
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
	.syscon_status_off = 0x78,
	.syscon_control0_off = 0x7c,
	.syscon_control1_off = 0x80,
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
	.syscon_control0_off = 0x70,
	.syscon_control1_off = 0x74,
	.syscon_status_off = 0x78,
	.dfx_irq_cause_off = 0x110,
	.dfx_irq_mask_off = 0x114,
	.dfx_overheat_irq = BIT(1), /* todo: maybe BIT(2) enables a "low threshold" int (a380)*/
	.dfx_server_irq_mask_off = 0x104,
	.dfx_server_irq_en = BIT(1), /* todo same as above */
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
	.syscon_control0_off = 0x84,
	.syscon_control1_off = 0x88,
	.syscon_status_off = 0x8C,
	.dfx_irq_cause_off = 0x108,
	.dfx_irq_mask_off = 0x10C,
	.dfx_overheat_irq = BIT(22),
	.dfx_server_irq_mask_off = 0x104,
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
	.syscon_control0_off = 0x70,
	.syscon_control1_off = 0x74,
	.syscon_status_off = 0x78,
	.dfx_irq_cause_off = 0x108,
	.dfx_irq_mask_off = 0x10C,
	.dfx_overheat_irq = BIT(20),
	.dfx_server_irq_mask_off = 0x104,
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

static const struct regmap_config armada_thermal_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.fast_io = true,
};

static int armada_thermal_probe_legacy(struct platform_device *pdev,
				       struct armada_thermal_priv *priv)
{
	struct armada_thermal_data *data = priv->data;
	struct resource *res;
	void __iomem *base;

	/* First memory region points towards the status register */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res))
		return PTR_ERR(res);

	/*
	 * Edit the resource start address and length to map over all the
	 * registers, instead of pointing at them one by one.
	 */
	res->start -= data->syscon_status_off;
	res->end = res->start + max(data->syscon_status_off,
				    max(data->syscon_control0_off,
					data->syscon_control1_off)) +
		   sizeof(unsigned int) - 1;

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->syscon = devm_regmap_init_mmio(&pdev->dev, base,
					     &armada_thermal_regmap_config);
	if (IS_ERR(priv->syscon))
		return PTR_ERR(priv->syscon);

	return 0;
}

static int armada_thermal_probe_syscon(struct platform_device *pdev,
				       struct armada_thermal_priv *priv)
{
	int irq, ret;

	priv->syscon = syscon_node_to_regmap(pdev->dev.parent->of_node);
	if (IS_ERR(priv->syscon))
		return PTR_ERR(priv->syscon);

	/*
	 * Do not error out if the IRQ is not found or could not be requested.
	 * CP used to share the overheat interrupt, so only the first CP to
	 * request the IRQ will gain it.
	 */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(&pdev->dev, irq, armada_overheat_isr,
			       IRQF_SHARED, NULL, priv);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&priv->wait_end_overheat_work,
			  armada_wait_end_overheat);

	return 0;
}

static int armada_thermal_probe(struct platform_device *pdev)
{
	struct thermal_zone_device *thermal;
	const struct of_device_id *match;
	struct armada_thermal_priv *priv;
	const char *name;
	int ret;

	match = of_match_device(armada_thermal_id_table, &pdev->dev);
	if (!match)
		return -ENODEV;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Platform device in data structure */
	priv->pdev = pdev;

	priv->data = (struct armada_thermal_data *)match->data;

	/*
	 * Legacy DT bindings only described "control1" register (also referred
	 * as "control MSB" on old documentation). Then, bindings moved to cover
	 * "control0/control LSB" and "control1/control MSB" registers within
	 * the same resource, which was then of size 8 instead of 4.
	 *
	 * The logic of defining sporadic registers is broken. For instance, it
	 * blocked the addition of the overheat interrupt feature that needed
	 * another resource somewhere else in the same memory area. One solution
	 * is to define an overall system controller and put the thermal node
	 * into it, which requires the use of regmaps across all the driver.
	 */
	if (IS_ERR(syscon_node_to_regmap(pdev->dev.parent->of_node))) {
		ret = armada_thermal_probe_legacy(pdev, priv);
	} else {
		ret = armada_thermal_probe_syscon(pdev, priv);
	}

	if (ret)
		return ret;

	priv->data->init_sensor(pdev, priv);

	/* Ensure device name is correct for the thermal core */
	name = dev_name(&pdev->dev);
	if (strlen(name) > THERMAL_NAME_LENGTH) {
		/*
		 * When inside a system controller, the device name has the
		 * form: f06f8000.system-controller:thermal@6f808C so stripping
		 * after the ':' should give us a shorter but meaningful name
		 */
		name = strrchr(name, ':');
		if (!name)
			name = "armada_thermal";
		else
			name++;
	}

	thermal = thermal_zone_device_register(name, 0, 0, priv, &ops, NULL, 0,
					       0);
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
