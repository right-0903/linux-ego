// SPDX-License-Identifier: GPL-2.0-only
/*
 * gaokun-wmi - A WMI driver for HUAWEI Matebook E Go (sc8280xp)
 *
 * The definition of ACPI's invention like GBTT, SBTT, etc from
 * reference: drivers/platform/x86/huawei-wmi.c
 *
 * Copyright (C) 2024 Pengyu Luo <mitltlatltl@gmail.com>
 */

#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/version.h>

#include <linux/platform_data/huawei-gaokun-ec.h>

struct gaokun_wmi {
	struct gaokun_ec *ec;
	struct device *dev;
	struct platform_device *wmi;
};

/* -------------------------------------------------------------------------- */
/* Battery charging threshold */

enum gaokun_wmi_threshold_ind {
	START	= 1,
	END	= 2,
};

static ssize_t charge_control_thresholds_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	int ret;
	u8 start, end;
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);

	ret = gaokun_ec_wmi_get_threshold(ecwmi->ec, &start, START)
		|| gaokun_ec_wmi_get_threshold(ecwmi->ec, &end, END);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d %d\n", start, end);
}

static ssize_t charge_control_thresholds_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t size)
{
	int ret;
	u8 start, end;
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);

	if (sscanf(buf, "%hhd %hhd", &start, &end) != 2)
		return -EINVAL;

	ret = gaokun_ec_wmi_set_threshold(ecwmi->ec, start, end);
	if (ret)
		return ret;

	return size;
}

static DEVICE_ATTR_RW(charge_control_thresholds);

/* -------------------------------------------------------------------------- */
/* Smart charge param */

static ssize_t smart_charge_param_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int ret;
	u8 value;
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);

	ret = gaokun_ec_wmi_get_smart_charge_param(ecwmi->ec, &value);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", value);
}

static ssize_t smart_charge_param_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int ret;
	u8 value;
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);

	if (kstrtou8(buf, 10, &value))
		return -EINVAL;

	ret = gaokun_ec_wmi_set_smart_charge_param(ecwmi->ec, value);
	if (ret)
		return ret;

	return size;
}

static DEVICE_ATTR_RW(smart_charge_param);

/* -------------------------------------------------------------------------- */
/* Smart charge */

static ssize_t smart_charge_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	int ret;
	u8 bf[GAOKUN_SMART_CHARGE_DATA_SIZE];
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);

	ret = gaokun_ec_wmi_get_smart_charge(ecwmi->ec, bf);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d %d %d %d\n",
			  bf[0], bf[1], bf[2], bf[3]);
}

static ssize_t smart_charge_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	int ret;
	u8 bf[GAOKUN_SMART_CHARGE_DATA_SIZE];
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);

	if (sscanf(buf, "%hhd %hhd %hhd %hhd", bf, bf + 1, bf + 2, bf + 3) != 4)
		return -EINVAL;

	ret = gaokun_ec_wmi_set_smart_charge(ecwmi->ec, bf);
	if (ret)
		return ret;

	return size;
}

static DEVICE_ATTR_RW(smart_charge);

/* -------------------------------------------------------------------------- */
/* Fn lock */

static ssize_t fn_lock_state_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	int ret;
	u8 on;
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);

	ret = gaokun_ec_wmi_get_fn_lock(ecwmi->ec, &on);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", on);
}

static ssize_t fn_lock_state_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int ret;
	u8 on;
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);

	if (kstrtou8(buf, 10, &on))
		return -EINVAL;

	ret = gaokun_ec_wmi_set_fn_lock(ecwmi->ec, on);
	if (ret)
		return ret;

	return size;
}

static DEVICE_ATTR_RW(fn_lock_state);

/* -------------------------------------------------------------------------- */
/* Thermal Zone */

static ssize_t temperature_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{

	int ret, len, i;
	char *ptr = buf;
	s16 value;
	s16 temp[GAOKUN_TZ_REG_NUM];
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);

	ret = gaokun_ec_wmi_get_temp(ecwmi->ec, temp);
	if (ret)
		return ret;

	len = i = 0;
	while (i < 20) {
		value = temp[i++];
		if (value < 0) {
			len += sprintf(ptr + len, "-");
			value = -value;
		}
		len += sprintf(ptr + len, "%d.%d ", value / 10, value % 10);
	}
	len += sprintf(ptr + len, "\n");

	return len;
}

static DEVICE_ATTR_RO(temperature);

static struct attribute *gaokun_wmi_features_attrs[] = {
	&dev_attr_charge_control_thresholds.attr,
	&dev_attr_smart_charge_param.attr,
	&dev_attr_smart_charge.attr,
	&dev_attr_fn_lock_state.attr,
	&dev_attr_temperature.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gaokun_wmi_features);

static int gaokun_wmi_probe(struct auxiliary_device *adev,
			    const struct auxiliary_device_id *id)
{
	struct gaokun_ec *ec = adev->dev.platform_data;
	struct device *dev = &adev->dev;
	struct gaokun_wmi *ecwmi;

	ecwmi = devm_kzalloc(&adev->dev, sizeof(*ecwmi), GFP_KERNEL);
	if (!ecwmi)
		return -ENOMEM;

	ecwmi->ec = ec;
	ecwmi->dev = dev;

	auxiliary_set_drvdata(adev, ecwmi);

	/* make it under /sys/devices/platform, convenient for sysfs I/O,
	 * while adev is under
	 * /sys/devices/platform/soc@0/ac0000.geniqup/a9c000.i2c/i2c-15/15-0038/
	 */
	ecwmi->wmi = platform_device_register_simple("gaokun-wmi", -1, NULL, 0);
	if (IS_ERR(ecwmi->wmi))
    	return dev_err_probe(dev, PTR_ERR(ecwmi->wmi),
			     "Failed to register wmi platform device\n");

	platform_set_drvdata(ecwmi->wmi, ecwmi);

	return device_add_groups(&ecwmi->wmi->dev, gaokun_wmi_features_groups);
}

static void gaokun_wmi_remove(struct auxiliary_device *adev)
{
	struct gaokun_wmi *ecwmi = auxiliary_get_drvdata(adev);
	struct platform_device *wmi = ecwmi->wmi;

	device_remove_groups(&wmi->dev, gaokun_wmi_features_groups);
	platform_device_unregister(ecwmi->wmi);
}

static const struct auxiliary_device_id gaokun_wmi_id_table[] = {
	{ .name = GAOKUN_MOD_NAME "." GAOKUN_DEV_WMI, },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, gaokun_wmi_id_table);

static struct auxiliary_driver gaokun_wmi_driver = {
	.name = GAOKUN_DEV_WMI,
	.id_table = gaokun_wmi_id_table,
	.probe = gaokun_wmi_probe,
	.remove = gaokun_wmi_remove,
};

module_auxiliary_driver(gaokun_wmi_driver);

MODULE_DESCRIPTION("HUAWEI Matebook E Go WMI driver");
MODULE_LICENSE("GPL");
