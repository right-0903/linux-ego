// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 *
 * Initial porting of the BCL (Battery Current Limit) feature based on the
 * SM8350 and SM8650 MSM kernels, targeting platforms earlier than
 * SM8350/SC8280XP.
 */

#define DEBUG

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/spmi.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/nvmem-consumer.h>

#include "../thermal_core.h"
#include "../thermal_hwmon.h"

/*
static inline __maybe_unused int qti_tz_change_mode(struct thermal_zone_device *tz,
						    enum thermal_device_mode mode)
{
	struct thermal_instance *instance;

	if (!tz)
		return 0;

	tz->passive = 0;
	tz->temperature = THERMAL_TEMP_INVALID;
	tz->prev_low_trip = -INT_MAX;
	tz->prev_high_trip = INT_MAX;
	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		instance->initialized = false;
		if (mode == THERMAL_DEVICE_DISABLED) {
			instance->target = THERMAL_NO_TARGET;
			instance->cdev->updated = false;
			thermal_cdev_update(instance->cdev);
		}
	}

	return 0;
}

static inline __maybe_unused int qti_update_tz_ops(struct thermal_zone_device *tz, bool enable)
{
	if (!tz)
		return -EINVAL;

	mutex_lock(&tz->lock);
	if (enable) {
		if (!tz->ops.change_mode)
			tz->ops.change_mode = qti_tz_change_mode;
	} else {
		tz->ops.change_mode = NULL;
	}
	mutex_unlock(&tz->lock);
	return 0;
}
*/

#define BCL_DRIVER_NAME       "bcl_pmic5"
#define BCL_MONITOR_EN        0x46
#define BCL_IRQ_STATUS        0x08

#define BCL_IBAT_HIGH             0x4B
#define BCL_IBAT_TOO_HIGH         0x4C
#define BCL_IBAT_READ             0x86
#define BCL_IBAT_SCALING_UA       78127
#define BCL_IBAT_CCM_SCALING_UA   15625

#define BCL_VBAT_READ         0x76
#define BCL_VBAT_ADC_LOW      0x48
#define BCL_VBAT_COMP_LOW     0x49
#define BCL_VBAT_COMP_TLOW    0x4A
#define BCL_VBAT_CONV_REQ     0x72

#define BCL_IRQ_L0       0x1
#define BCL_IRQ_L1       0x2
#define BCL_IRQ_L2       0x4

/*
* 49827 = 64.879uV (one bit value) * 3 (voltage divider)
*		* 256 (8 bit shift for MSB)
*/
#define BCL_VBAT_SCALING_UV   49827
#define BCL_VBAT_NO_READING   127
#define BCL_VBAT_BASE_MV      2000
#define BCL_VBAT_INC_MV       25
#define BCL_VBAT_MAX_MV       3600
#define BCL_VBAT_THRESH_BASE  2250

#define BCL_IBAT_CCM_OFFSET   800
#define BCL_IBAT_CCM_LSB      100
#define BCL_IBAT_CCM_MAX_VAL  14

#define MAX_PERPH_COUNT       2

enum bcl_dev_type {
	BCL_IBAT_LVL0,
	BCL_IBAT_LVL1,
	BCL_VBAT_LVL0,
	BCL_VBAT_LVL1,
	BCL_VBAT_LVL2,
	BCL_LVL0,
	BCL_LVL1,
	BCL_LVL2,
	BCL_TYPE_MAX,
};

static char bcl_int_names[BCL_TYPE_MAX][25] = {
	"bcl-ibat-lvl0",
	"bcl-ibat-lvl1",
	"bcl-vbat-lvl0",
	"bcl-vbat-lvl1",
	"bcl-vbat-lvl2",
	"bcl-lvl0",
	"bcl-lvl1",
	"bcl-lvl2",
};

enum bcl_ibat_ext_range_type {
	BCL_IBAT_RANGE_LVL0,
	BCL_IBAT_RANGE_LVL1,
	BCL_IBAT_RANGE_LVL2,
	BCL_IBAT_RANGE_MAX,
};

static uint32_t bcl_ibat_ext_ranges[BCL_IBAT_RANGE_MAX] = {
	10,		/* default range factor */
	20,
	25
};

struct bcl_device;

struct bcl_peripheral_data {
	int                     irq_num;
	int                     status_bit_idx;
	long			trip_thresh;
	int                     last_val;
	struct mutex            state_trans_lock;
	bool			irq_enabled;
	enum bcl_dev_type	type;
	struct thermal_zone_device_ops ops;
	struct thermal_zone_device *tz_dev;
	struct thermal_trip	*trips;
	struct bcl_device	*dev;
};

struct bcl_device {
	struct device			*dev;
	struct regmap			*regmap;
	uint16_t			fg_bcl_addr;
	bool				ibat_ccm_enabled;
	bool				ibat_use_qg_adc;
	bool				no_bit_shift;
	uint32_t			ibat_ext_range_factor;
	struct bcl_peripheral_data	param[BCL_TYPE_MAX];
};

static struct bcl_device *bcl_devices[MAX_PERPH_COUNT];
static int bcl_device_ct; /* On sm8350, pm83500b(ibat, lbat), pm8350c(lbat) */

static int bcl_read_register(struct bcl_device *bcl_perph, int16_t reg_offset,
				unsigned int *data)
{
	int ret = 0;

	if (!bcl_perph) {
		pr_err("BCL device not initialized\n");
		return -EINVAL;
	}
	ret = regmap_read(bcl_perph->regmap,
			       (bcl_perph->fg_bcl_addr + reg_offset),
			       data);
	if (ret < 0)
		pr_err("Error reading register 0x%04x err:%d\n",
				bcl_perph->fg_bcl_addr + reg_offset, ret);
	else
		pr_debug("Read register:0x%04x value:0x%02x\n",
				bcl_perph->fg_bcl_addr + reg_offset,
				*data);

	return ret;
}

static int bcl_write_register(struct bcl_device *bcl_perph,
			      int16_t reg_offset, uint8_t data)
{
	int  ret = 0;
	uint8_t *write_buf = &data;
	uint16_t base;

	if (!bcl_perph) {
		pr_err("BCL device not initialized\n");
		return -EINVAL;
	}
	base = bcl_perph->fg_bcl_addr;
	ret = regmap_write(bcl_perph->regmap, (base + reg_offset), *write_buf);
	if (ret < 0) {
		pr_err("Error reading register:0x%04x val:0x%02x err:%d\n",
				base + reg_offset, data, ret);
		return ret;
	}
	pr_debug("wrote 0x%02x to 0x%04x\n", data, base + reg_offset);

	return ret;
}

static void convert_adc_to_vbat_thresh_val(struct bcl_device *bcl_per, int *val)
{
	/*
	 * Threshold register can be bit shifted from ADC MSB.
	 * So the scaling factor is half in those cases.
	 */
	if (bcl_per->no_bit_shift)
		*val = (*val * BCL_VBAT_SCALING_UV) / 1000;
	else
		*val = (*val * BCL_VBAT_SCALING_UV) / 2000;
}

static void convert_adc_to_vbat_val(int *val)
{
	*val = (*val * BCL_VBAT_SCALING_UV) / 1000;
}

static void convert_ibat_to_adc_val(struct bcl_device *bcl_per, int *val, int scaling_factor)
{
	/*
	 * Threshold register can be bit shifted from ADC MSB.
	 * So the scaling factor is half in those cases.
	 */
	if (bcl_per->ibat_use_qg_adc)
		*val = (int)div_s64(*val * 2000 * 2, scaling_factor);
	else if (bcl_per->no_bit_shift)
		*val = (int)div_s64(*val * 1000 * bcl_ibat_ext_ranges[BCL_IBAT_RANGE_LVL0],
				scaling_factor);
	else
		*val = (int)div_s64(*val * 2000 * bcl_ibat_ext_ranges[BCL_IBAT_RANGE_LVL0],
				scaling_factor);

}

static void convert_adc_to_ibat_val(struct bcl_device *bcl_per, int *val, int scaling_factor)
{
	/* Scaling factor will be half if ibat_use_qg_adc is true */
	if (bcl_per->ibat_use_qg_adc)
		*val = (int)div_s64(*val * scaling_factor, 2 * 1000);
	else
		*val = (int)div_s64(*val * scaling_factor,
				1000 * bcl_ibat_ext_ranges[BCL_IBAT_RANGE_LVL0]);
}

static int8_t convert_ibat_to_ccm_val(int ibat)
{
	int8_t val = BCL_IBAT_CCM_MAX_VAL;

	val = (int8_t)((ibat - BCL_IBAT_CCM_OFFSET) / BCL_IBAT_CCM_LSB);

	if (val > BCL_IBAT_CCM_MAX_VAL) {
		pr_err(
		"CCM thresh:%d is invalid, use MAX supported threshold\n",
			ibat);
		val = BCL_IBAT_CCM_MAX_VAL;
	}

	return val;
}

static int bcl_set_ibat(struct thermal_zone_device *tz, int low, int high)
{
	int ret = 0, ibat_ua, thresh_value;
	int8_t val = 0;
	int16_t addr;
	struct bcl_peripheral_data *bat_data = thermal_zone_device_priv(tz);

	mutex_lock(&bat_data->state_trans_lock);
	thresh_value = high;
	if (bat_data->trip_thresh == thresh_value)
		goto set_trip_exit;

	if (bat_data->irq_num && bat_data->irq_enabled) {
		disable_irq_nosync(bat_data->irq_num);
		bat_data->irq_enabled = false;
	}
	if (thresh_value == INT_MAX) {
		bat_data->trip_thresh = thresh_value;
		goto set_trip_exit;
	}

	ibat_ua = thresh_value;
	if (bat_data->dev->ibat_ccm_enabled)
		convert_ibat_to_adc_val(bat_data->dev, &thresh_value,
				BCL_IBAT_CCM_SCALING_UA * bat_data->dev->ibat_ext_range_factor);
	else
		convert_ibat_to_adc_val(bat_data->dev, &thresh_value,
				BCL_IBAT_SCALING_UA * bat_data->dev->ibat_ext_range_factor);
	val = (int8_t)thresh_value;
	switch (bat_data->type) {
	case BCL_IBAT_LVL0:
		addr = BCL_IBAT_HIGH;
		pr_debug("ibat high threshold:%d mA ADC:0x%02x\n",
				ibat_ua, val);
		break;
	case BCL_IBAT_LVL1:
		addr = BCL_IBAT_TOO_HIGH;
		if (bat_data->dev->ibat_ccm_enabled)
			val = convert_ibat_to_ccm_val(ibat_ua);
		pr_debug("ibat too high threshold:%d mA ADC:0x%02x\n",
				ibat_ua, val);
		break;
	default:
		goto set_trip_exit;
	}
	ret = bcl_write_register(bat_data->dev, addr, val);
	if (ret)
		goto set_trip_exit;
	bat_data->trip_thresh = ibat_ua;

	if (bat_data->irq_num && !bat_data->irq_enabled) {
		enable_irq(bat_data->irq_num);
		bat_data->irq_enabled = true;
	}

set_trip_exit:
	mutex_unlock(&bat_data->state_trans_lock);

	return ret;
}

static int bcl_read_ibat(struct thermal_zone_device *tz, int *adc_value)
{
	int ret = 0;
	unsigned int val = 0;
	struct bcl_peripheral_data *bat_data = thermal_zone_device_priv(tz);

	*adc_value = val;
	ret = bcl_read_register(bat_data->dev, BCL_IBAT_READ, &val);
	if (ret)
		goto bcl_read_exit;
	/* IBat ADC reading is in 2's compliment form */
	*adc_value = sign_extend32(val, 7);
	if (val == 0) {
		/*
		 * The sensor sometime can read a value 0 if there is
		 * consequtive reads
		 */
		*adc_value = bat_data->last_val;
	} else {
		if (bat_data->dev->ibat_ccm_enabled)
			convert_adc_to_ibat_val(bat_data->dev, adc_value,
				BCL_IBAT_CCM_SCALING_UA * bat_data->dev->ibat_ext_range_factor);
		else
			convert_adc_to_ibat_val(bat_data->dev, adc_value,
				BCL_IBAT_SCALING_UA * bat_data->dev->ibat_ext_range_factor);
		bat_data->last_val = *adc_value;
	}
	pr_debug("ibat:%d mA ADC:0x%02x\n", bat_data->last_val, val);

bcl_read_exit:
	return ret;
}

static int bcl_read_vbat_tz(struct thermal_zone_device *tzd, int *adc_value)
{
	int ret = 0;
	unsigned int val = 0;
	struct bcl_peripheral_data *bat_data = thermal_zone_device_priv(tzd);

	*adc_value = val;
	ret = bcl_read_register(bat_data->dev, BCL_VBAT_READ, &val);
	if (ret)
		goto bcl_read_exit;
	*adc_value = val;
	if (*adc_value == BCL_VBAT_NO_READING) {
		*adc_value = bat_data->last_val;
	} else {
		convert_adc_to_vbat_val(adc_value);
		bat_data->last_val = *adc_value;
	}
	pr_debug("vbat:%d mv ADC:0x%02x\n", bat_data->last_val, val);

bcl_read_exit:
	return ret;
}

static struct thermal_zone_device_ops vbat_tzd_ops = {
	.get_temp = bcl_read_vbat_tz,
};

static struct thermal_zone_params vbat_tzp = {
	.governor_name = "step_wise",
	.no_hwmon = true,
	.sustainable_power = 0,
	.k_po = 0,
	.k_pu = 0,
	.k_i = 0,
	.k_d = 0,
	.integral_cutoff = 0,
	.slope = 1,
	.offset = 0
};

static int bcl_get_trend(struct thermal_zone_device *tz,
			 const struct thermal_trip *trip,
			 enum thermal_trend *trend)
{
	struct bcl_peripheral_data *bat_data = thermal_zone_device_priv(tz);

	mutex_lock(&bat_data->state_trans_lock);
	if (!bat_data->last_val)
		*trend = THERMAL_TREND_DROPPING;
	else
		*trend = THERMAL_TREND_RAISING;
	mutex_unlock(&bat_data->state_trans_lock);

	return 0;
}

static int bcl_set_lbat(struct thermal_zone_device *tz, int low, int high)
{
	struct bcl_peripheral_data *bat_data = thermal_zone_device_priv(tz);

	mutex_lock(&bat_data->state_trans_lock);

	if (high == INT_MAX &&
		bat_data->irq_num && bat_data->irq_enabled) {
		disable_irq_nosync(bat_data->irq_num);
		disable_irq_wake(bat_data->irq_num);
		bat_data->irq_enabled = false;
		pr_debug("lbat[%d]: disable irq:%d\n",
				bat_data->type,
				bat_data->irq_num);
	} else if (high != INT_MAX &&
		bat_data->irq_num && !bat_data->irq_enabled) {
		enable_irq(bat_data->irq_num);
		enable_irq_wake(bat_data->irq_num);
		bat_data->irq_enabled = true;
		pr_debug("lbat[%d]: enable irq:%d\n",
				bat_data->type,
				bat_data->irq_num);
	}

	mutex_unlock(&bat_data->state_trans_lock);

	return 0;
}

static int bcl_read_lbat(struct thermal_zone_device *tz, int *adc_value)
{
	int ret = 0;
	int ibat = 0, vbat = 0;
	unsigned int val = 0;
	struct bcl_peripheral_data *bat_data = thermal_zone_device_priv(tz);
	struct bcl_device *bcl_perph = bat_data->dev;

	*adc_value = val;
	ret = bcl_read_register(bcl_perph, BCL_IRQ_STATUS, &val);
	if (ret)
		goto bcl_read_exit;
	switch (bat_data->type) {
	case BCL_LVL0:
		*adc_value = val & BCL_IRQ_L0;
		break;
	case BCL_LVL1:
		*adc_value = val & BCL_IRQ_L1;
		break;
	case BCL_LVL2:
		*adc_value = val & BCL_IRQ_L2;
		break;
	default:
		pr_err("Invalid sensor type:%d\n", bat_data->type);
		ret = -ENODEV;
		goto bcl_read_exit;
	}
	bat_data->last_val = *adc_value;
	pr_debug("lbat:%d val:%d\n", bat_data->type,
			bat_data->last_val);
	if (bcl_perph->param[BCL_IBAT_LVL0].tz_dev)
		bcl_read_ibat(bcl_perph->param[BCL_IBAT_LVL0].tz_dev, &ibat);
	if (bcl_perph->param[BCL_VBAT_LVL0].tz_dev)
		bcl_read_vbat_tz(bcl_perph->param[BCL_VBAT_LVL0].tz_dev, &vbat);

bcl_read_exit:
	return ret;
}

static irqreturn_t bcl_handle_irq(int irq, void *data)
{
	struct bcl_peripheral_data *perph_data = thermal_zone_device_priv(data);
	unsigned int irq_status = 0;
	int ibat = 0, vbat = 0;
	struct bcl_device *bcl_perph;

	bcl_perph = perph_data->dev;
	bcl_read_register(bcl_perph, BCL_IRQ_STATUS, &irq_status);
	if (bcl_perph->param[BCL_IBAT_LVL0].tz_dev)
		bcl_read_ibat(bcl_perph->param[BCL_IBAT_LVL0].tz_dev, &ibat);
	if (bcl_perph->param[BCL_VBAT_LVL0].tz_dev)
		bcl_read_vbat_tz(bcl_perph->param[BCL_VBAT_LVL0].tz_dev, &vbat);

	if (irq_status & perph_data->status_bit_idx) {
		pr_debug(
		"Irq:%d triggered for bcl type:%s. status:%u ibat=%d vbat=%d\n",
			irq, bcl_int_names[perph_data->type],
			irq_status, ibat, vbat);

		thermal_zone_device_update(perph_data->tz_dev, THERMAL_TRIP_VIOLATED);
	}

	return IRQ_HANDLED;
}

static int bcl_get_ibat_ext_range_factor(struct platform_device *pdev,
		uint32_t *ibat_range_factor)
{
	int ret = 0;
	const char *name;
	struct nvmem_cell *cell;
	size_t len;
	char *buf;
	uint32_t ext_range_index = 0;

	ret = of_property_read_string(pdev->dev.of_node, "nvmem-cell-names", &name);
	if (ret) {
		*ibat_range_factor = bcl_ibat_ext_ranges[BCL_IBAT_RANGE_LVL0];
		pr_debug("Default ibat range factor enabled %u\n", *ibat_range_factor);
		return 0;
	}

	cell = nvmem_cell_get(&pdev->dev, name);
	if (IS_ERR(cell)) {
		dev_err(&pdev->dev, "failed to get nvmem cell %s\n", name);
		return PTR_ERR(cell);
	}

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR_OR_NULL(buf)) {
		dev_err(&pdev->dev, "failed to read nvmem cell %s\n", name);
		return PTR_ERR(buf);
	}

	if (len <= 0 || len > sizeof(uint32_t)) {
		dev_err(&pdev->dev, "nvmem cell length out of range %d\n", (int)len);
		kfree(buf);
		return -EINVAL;
	}
	memcpy(&ext_range_index, buf, min(len, sizeof(ext_range_index)));
	kfree(buf);

	if (ext_range_index >= BCL_IBAT_RANGE_MAX) {
		dev_err(&pdev->dev, "invalid BCL ibat scaling factor %d\n", ext_range_index);
		return -EINVAL;
	}

	*ibat_range_factor = bcl_ibat_ext_ranges[ext_range_index];
	pr_debug("ext_range_index %u, ibat range factor %u\n",
			ext_range_index, *ibat_range_factor);

	return 0;
}

static int bcl_get_devicetree_data(struct platform_device *pdev,
					struct bcl_device *bcl_perph)
{
	int ret = 0;
	const __be32 *prop = NULL;
	struct device_node *dev_node = pdev->dev.of_node;

	prop = of_get_address(dev_node, 0, NULL, NULL);
	if (prop) {
		bcl_perph->fg_bcl_addr = be32_to_cpu(*prop);
		pr_debug("fg_bcl@%04x\n", bcl_perph->fg_bcl_addr);
	} else {
		dev_err(&pdev->dev, "No fg_bcl registers found\n");
		return -ENODEV;
	}

	bcl_perph->ibat_use_qg_adc = of_property_read_bool(dev_node,
				"qcom,ibat-use-qg-adc-5a");
	bcl_perph->no_bit_shift =  of_property_read_bool(dev_node,
				"qcom,pmic7-threshold");
	bcl_perph->ibat_ccm_enabled =  of_property_read_bool(dev_node,
						"qcom,ibat-ccm-hw-support");

	ret = bcl_get_ibat_ext_range_factor(pdev,
					&bcl_perph->ibat_ext_range_factor);

	return ret;
}

static void bcl_fetch_trip(struct platform_device *pdev, enum bcl_dev_type type,
		struct bcl_peripheral_data *data,
		irqreturn_t (*handle)(int, void *))
{
	int ret = 0, irq_num = 0;
	char *int_name = bcl_int_names[type];

	mutex_lock(&data->state_trans_lock);
	data->irq_num = 0;
	data->irq_enabled = false;
	irq_num = platform_get_irq_byname(pdev, int_name);
	if (irq_num > 0 && handle) {
		ret = devm_request_threaded_irq(&pdev->dev,
				irq_num, NULL, handle,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				int_name, data);
		if (ret) {
			dev_err(&pdev->dev,
				"Error requesting trip irq. err:%d\n",
				ret);
			mutex_unlock(&data->state_trans_lock);
			return;
		}
		disable_irq_nosync(irq_num);
		data->irq_num = irq_num;
	} else if (irq_num > 0 && !handle) {
		disable_irq_nosync(irq_num);
		data->irq_num = irq_num;
	}
	mutex_unlock(&data->state_trans_lock);
}

static struct thermal_trip * bcl_vbat_trips_init(struct device *dev,
						 struct bcl_peripheral_data *bat_data,
						 int num_trips)
{
	struct thermal_trip *trips;
	unsigned int val = 0;
	int16_t addr;
	int ret, i;

	trips = devm_kcalloc(dev, num_trips, sizeof(*trips), GFP_KERNEL);
	if (!trips)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < num_trips; i++) {
		switch (i + BCL_VBAT_LVL0) {
		case BCL_VBAT_LVL0:
			addr = BCL_VBAT_ADC_LOW;
			break;
		case BCL_VBAT_LVL1:
			addr = BCL_VBAT_COMP_LOW;
			break;
		case BCL_VBAT_LVL2:
			addr = BCL_VBAT_COMP_TLOW;
			break;
		}

		ret = bcl_read_register(bat_data->dev, addr, &val);
		if (ret) {
			trips[i].temperature = THERMAL_TEMP_INVALID;
			continue;
		}

		if (addr == BCL_VBAT_ADC_LOW) {
			trips[i].temperature = val;
			convert_adc_to_vbat_thresh_val(bat_data->dev, &trips[i].temperature);
			pr_debug("vbat trip: %d mV ADC:0x%02x\n", trips[i].temperature, val);
		} else {
			trips[i].temperature = BCL_VBAT_THRESH_BASE + val * 25;
			if (trips[i].temperature > BCL_VBAT_MAX_MV)
				trips[i].temperature = BCL_VBAT_MAX_MV;
			pr_debug("vbat-%s-low trip: %d mV ADC:0x%02x\n",
				 (addr == BCL_VBAT_COMP_LOW) ?
				 "too" : "critical", trips[i].temperature, val);
		}

		trips[i].type = THERMAL_TRIP_PASSIVE;
	}

	return trips;
}


static void bcl_vbat_init(struct platform_device *pdev,
		enum bcl_dev_type type, struct bcl_device *bcl_perph)
{
	struct bcl_peripheral_data *vbat = &bcl_perph->param[type];
	unsigned int val = 0;
	int trips_count = 3;
	int ret;

	mutex_init(&vbat->state_trans_lock);
	vbat->dev = bcl_perph;
	vbat->irq_num = 0;
	vbat->irq_enabled = false;
	vbat->tz_dev = NULL;
	ret = bcl_read_register(bcl_perph, BCL_VBAT_CONV_REQ, &val);
	if (ret || !val)
		return;

	vbat->trips = bcl_vbat_trips_init(&pdev->dev, vbat, trips_count);
	if (IS_ERR(vbat->trips)) {
		vbat->trips = NULL;
		return;
	}

	vbat->tz_dev = thermal_zone_device_register_with_trips("vbat", vbat->trips, trips_count, vbat,
			&vbat_tzd_ops, &vbat_tzp, 0, 0);
	if (IS_ERR(vbat->tz_dev)) {
		pr_debug("vbat[%s] register failed. err:%ld\n",
				bcl_int_names[type],
				PTR_ERR(vbat->tz_dev));
		vbat->tz_dev = NULL;
		return;
	}
	// thermal_zone_device_update(vbat->tz_dev, THERMAL_DEVICE_UP);
	ret = thermal_zone_device_enable(vbat->tz_dev);
	if (ret) {
		thermal_zone_device_unregister(vbat->tz_dev);
		vbat->tz_dev = NULL;
	}

	devm_thermal_add_hwmon_sysfs(&pdev->dev, vbat->tz_dev);
}

static void bcl_probe_vbat(struct platform_device *pdev,
					struct bcl_device *bcl_perph)
{
	bcl_vbat_init(pdev, BCL_VBAT_LVL0, bcl_perph);
}

static void bcl_ibat_init(struct platform_device *pdev,
			enum bcl_dev_type type, struct bcl_device *bcl_perph)
{
	struct bcl_peripheral_data *ibat = &bcl_perph->param[type];

	mutex_init(&ibat->state_trans_lock);
	ibat->type = type;
	ibat->dev = bcl_perph;
	ibat->irq_num = 0;
	ibat->irq_enabled = false;
	ibat->ops.get_temp = bcl_read_ibat;
	ibat->ops.set_trips = bcl_set_ibat;
	ibat->tz_dev = devm_thermal_of_zone_register(&pdev->dev,
				type, ibat, &ibat->ops);
	if (IS_ERR(ibat->tz_dev)) {
		pr_debug("ibat:[%s] register failed. err:%ld\n",
				bcl_int_names[type],
				PTR_ERR(ibat->tz_dev));
		ibat->tz_dev = NULL;
		return;
	}
	thermal_zone_device_update(ibat->tz_dev, THERMAL_DEVICE_UP);
}

static void bcl_probe_ibat(struct platform_device *pdev,
					struct bcl_device *bcl_perph)
{
	bcl_ibat_init(pdev, BCL_IBAT_LVL0, bcl_perph);
	bcl_ibat_init(pdev, BCL_IBAT_LVL1, bcl_perph);
}

static void bcl_lvl_init(struct platform_device *pdev,
	enum bcl_dev_type type, int sts_bit_idx, struct bcl_device *bcl_perph)
{
	struct bcl_peripheral_data *lbat = &bcl_perph->param[type];

	mutex_init(&lbat->state_trans_lock);
	lbat->type = type;
	lbat->dev = bcl_perph;
	lbat->status_bit_idx = sts_bit_idx;
	bcl_fetch_trip(pdev, type, lbat, bcl_handle_irq);
	if (lbat->irq_num <= 0)
		return;

	lbat->ops.get_temp = bcl_read_lbat;
	lbat->ops.set_trips = bcl_set_lbat;
	lbat->ops.get_trend = bcl_get_trend;

	lbat->tz_dev = devm_thermal_of_zone_register(&pdev->dev,
				type, lbat, &lbat->ops);
	if (IS_ERR(lbat->tz_dev)) {
		pr_debug("lbat:[%s] register failed. err:%ld\n",
				bcl_int_names[type],
				PTR_ERR(lbat->tz_dev));
		lbat->tz_dev = NULL;
		return;
	}
	thermal_zone_device_update(lbat->tz_dev, THERMAL_DEVICE_UP);

	devm_thermal_add_hwmon_sysfs(&pdev->dev, lbat->tz_dev);
	// qti_update_tz_ops(lbat->tz_dev, true); // ???
}

static void bcl_probe_lvls(struct platform_device *pdev,
					struct bcl_device *bcl_perph)
{
	bcl_lvl_init(pdev, BCL_LVL0, BCL_IRQ_L0, bcl_perph);
	bcl_lvl_init(pdev, BCL_LVL1, BCL_IRQ_L1, bcl_perph);
	bcl_lvl_init(pdev, BCL_LVL2, BCL_IRQ_L2, bcl_perph);
}

static void bcl_configure_bcl_peripheral(struct bcl_device *bcl_perph)
{
	bcl_write_register(bcl_perph, BCL_MONITOR_EN, BIT(7));
}

static void bcl_remove(struct platform_device *pdev)
{
	struct bcl_device *bcl_perph = dev_get_drvdata(&pdev->dev);
/*
	int i = 0;

	for (; i < BCL_TYPE_MAX; i++) {
		if (!bcl_perph->param[i].tz_dev)
			continue;
		// qti_update_tz_ops(bcl_perph->param[i].tz_dev, false);
	}
*/
	if (bcl_perph->param[BCL_VBAT_LVL0].tz_dev)
		thermal_zone_device_unregister(bcl_perph->param[BCL_VBAT_LVL0].tz_dev);
}

static int bcl_probe(struct platform_device *pdev)
{
	struct bcl_device *bcl_perph = NULL;
	int err = 0;

	if (bcl_device_ct >= MAX_PERPH_COUNT) {
		dev_err(&pdev->dev, "Max bcl peripheral supported already.\n");
		return -EINVAL;
	}
	bcl_devices[bcl_device_ct] = devm_kzalloc(&pdev->dev,
					sizeof(*bcl_devices[0]), GFP_KERNEL);
	if (!bcl_devices[bcl_device_ct])
		return -ENOMEM;
	bcl_perph = bcl_devices[bcl_device_ct];
	bcl_perph->dev = &pdev->dev;

	bcl_perph->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!bcl_perph->regmap) {
		dev_err(&pdev->dev, "Couldn't get parent's regmap\n");
		return -EINVAL;
	}

	bcl_device_ct++;
	err = bcl_get_devicetree_data(pdev, bcl_perph);
	if (err) {
		bcl_device_ct--;
		return err;
	}

	bcl_probe_vbat(pdev, bcl_perph);
	bcl_probe_ibat(pdev, bcl_perph);
	bcl_probe_lvls(pdev, bcl_perph);
	bcl_configure_bcl_peripheral(bcl_perph);

	dev_set_drvdata(&pdev->dev, bcl_perph);

	return 0;
}

static const struct of_device_id bcl_match[] = {
	{
		.compatible = "qcom,bcl-v5",
	},
	{},
};
MODULE_DEVICE_TABLE(of, bcl_match);

static struct platform_driver bcl_driver = {
	.probe  = bcl_probe,
	.remove = bcl_remove,
	.driver = {
		.name           = BCL_DRIVER_NAME,
		.of_match_table = bcl_match,
	},
};

module_platform_driver(bcl_driver);
MODULE_LICENSE("GPL v2");
