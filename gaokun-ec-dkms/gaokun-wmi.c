// SPDX-License-Identifier: GPL-2.0-only
/*
 * gaokun-wmi - A WMI driver for HUAWEI Matebook E Go (sc8280xp)
 *
 * The definition of ACPI's invention like GBTT, SBTT, etc from
 * drivers/platform/x86/huawei-wmi.c
 *
 * Copyright (C) 2024 nuvole <mitltlatltl@gmail.com>
 */

#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/version.h>

#include "ec.h"

#define CHARGE_CONTROL_SET  0X68
#define CHARGE_CONTROL_GET  0X69
#define FN_LOCK_STATE_GET   0X6B
#define FN_LOCK_STATE_SET   0X6C

struct gaokun_wmi {
	struct gaokun_ec *ec;
    struct device *dev;
    struct platform_device *wmi;
};

/* Battery charging threshold */
enum Indicator {
	START 	= 1,
	END	  	= 2,
	UNKNOWN = 3,
};

static int gaokun_get_threshold(struct device *dev, int *data, enum Indicator ind)
{
	/* GBTT */
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);
	u8 *obuf;

	obuf = ec_command_data(ecwmi->ec, 0x02, CHARGE_CONTROL_GET, 1, (u8 []){ind}, 3);
	if (data)
		*data = obuf[2];

	return obuf[0];
}

static int gaokun_set_threshold(struct device *dev, int start, int end)
{
	/* SBTT */
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);

	int err;
	u8 *obuf;

	obuf = ec_command_data(ecwmi->ec, 0x02, CHARGE_CONTROL_SET, 2, (u8 []){UNKNOWN, 0x5a}, 1);
	err = *obuf;

	if (err)
		return err;

	if(start == 0 && end == 0) /* 0 0 actually works, but means? */
		return -EINVAL;

	if(start >= 0 && start <= end && end <= 100) { // start and end valid
		obuf = ec_command_data(ecwmi->ec, 0x02, CHARGE_CONTROL_SET, 2, (u8 []){START, start}, 1);
		err = *obuf;
		obuf = ec_command_data(ecwmi->ec, 0x02, CHARGE_CONTROL_SET, 2, (u8 []){END, end}, 1);
		err += *obuf;
	}else{
		return -EINVAL;
	}

	return err;
}

static ssize_t charge_control_start_threshold_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, start;

	err = gaokun_get_threshold(dev, &start, START);
	if (err)
		return err;

	return sysfs_emit(buf, "%d\n", start);
}

static ssize_t charge_control_end_threshold_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, end;

	err = gaokun_get_threshold(dev, &end, END);
	if (err)
		return err;

	return sysfs_emit(buf, "%d\n", end);
}

static ssize_t charge_control_thresholds_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, start, end;

	err = gaokun_get_threshold(dev, &start, START)
		|| gaokun_get_threshold(dev, &end, END);
	if (err)
		return err;

	return sysfs_emit(buf, "%d %d\n", start, end);
}

static ssize_t charge_control_start_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err, start, end;

	err = gaokun_get_threshold(dev, &end, END);
	if (err)
		return err;

	if (sscanf(buf, "%d", &start) != 1)
		return -EINVAL;

	err = gaokun_set_threshold(dev, start, end);
	if (err)
		return err;

	return size;
}

static ssize_t charge_control_end_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err, start, end;

	err = gaokun_get_threshold(dev, &start, START);
	if (err)
		return err;

	if (sscanf(buf, "%d", &end) != 1)
		return -EINVAL;

	err = gaokun_set_threshold(dev, start, end);
	if (err)
		return err;

	return size;
}

static ssize_t charge_control_thresholds_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err, start, end;

	if (sscanf(buf, "%d %d", &start, &end) != 2)
		return -EINVAL;

	err = gaokun_set_threshold(dev, start, end);
	if (err)
		return err;

	return size;
}

static DEVICE_ATTR_RW(charge_control_start_threshold);
static DEVICE_ATTR_RW(charge_control_end_threshold);
static DEVICE_ATTR_RW(charge_control_thresholds);

/* Smart charge param*/
static int gaokun_get_smart_charge_param(struct device *dev, int *value)
{
	// GBAC
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);
	u8 *obuf;

	obuf = ec_command_data(ecwmi->ec, 0x02, 0XE6, 0, NULL, 3);

	if (value)
		*value = obuf[2];

	return obuf[0];
}

static int gaokun_set_smart_charge_param(struct device *dev, int value)
{
	// SBAC
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);
	u8 *obuf;

	if (value < 0 || value > 2)
		return -EINVAL;

	obuf = ec_command_data(ecwmi->ec, 0x02, 0XE5, 1, (u8 []){value}, 1);

	return *obuf;
}

static ssize_t smart_charge_param_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, value;

	err = gaokun_get_smart_charge_param(dev, &value);
	if (err)
		return err;

	return sysfs_emit(buf, "%d\n", value);
}

static ssize_t smart_charge_param_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err, value;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	err = gaokun_set_smart_charge_param(dev, value);
	if (err)
		return err;

	return size;
}

static DEVICE_ATTR_RW(smart_charge_param);

/* Smart charge */
static int gaokun_get_smart_charge(struct device *dev, int *mode, int *delay, int *start, int *end)
{
	// GBCM
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);
	u8 *obuf;

	obuf = ec_command_data(ecwmi->ec, 0x02, 0XE4, 0, NULL, 6);

	if (mode)
		*mode = obuf[2];
	if (delay)
		*delay = obuf[3];
	if (start)
		*start = obuf[4];
	if (end)
		*end = obuf[5];

	return 0;
}

static int gaokun_set_smart_charge(struct device *dev, int mode, int delay, int start, int end)
{
	// SBCM
	struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);
	u8 *obuf;

	if (start < 0 || end < 0 || start > 100 || end > 100)
		return -EINVAL;

	obuf = ec_command_data(ecwmi->ec, 0x02, 0XE3, 4, (u8 []){mode, delay, start, end}, 2);
	
	return obuf[0];
}

static ssize_t smart_charge_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, start, end, mode, delay;

	err = gaokun_get_smart_charge(dev, &mode, &delay, &start, &end);
	if (err)
		return err;

	return sysfs_emit(buf, "%d %d %d %d\n", mode, delay, start, end);
}

static ssize_t smart_charge_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err, start, end, mode, delay;

	if (sscanf(buf, "%d %d %d %d", &mode, &delay, &start, &end) != 4)
		return -EINVAL;

	err = gaokun_set_smart_charge(dev, mode, delay, start, end);
	if (err)
		return err;

	return size;
}

static DEVICE_ATTR_RW(smart_charge);

/* Fn lock */
static int gaokun_get_fn_lock(struct device *dev, int *on)
{
    /* GFRS */
    struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);
	u8 *obuf;
	int ret;

	obuf = ec_command_data(ecwmi->ec, 0x02, FN_LOCK_STATE_GET, 0, NULL, 3);
	ret = obuf[2];
	if (ret == 0x55) {
		*on = 1;
	}else if (ret == 0x5A) {
		*on = 2;
	}else {
		return -ERANGE;
	}

	return *obuf;
}

static int gaokun_set_fn_lock(struct device *dev, int on)
{
    /* SFRS */
    struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);
	u8 *obuf;
	/* 1 off/unlock, 2 on/lock, you can also press fn key(light on) to reverse */

	if (on == 1) {
		obuf = ec_command_data(ecwmi->ec, 0x02, FN_LOCK_STATE_SET, 1, (u8 []){0x55}, 1);
	}else if (on == 2) {
		obuf = ec_command_data(ecwmi->ec, 0x02, FN_LOCK_STATE_SET, 1, (u8 []){0x5A}, 1);
	}else {
		return -EINVAL;
	}

	return *obuf;
}

static ssize_t fn_lock_state_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err, on;

	err = gaokun_get_fn_lock(dev, &on);
	if (err)
		return err;

	return sysfs_emit(buf, "%d\n", on - 1);
}

static ssize_t fn_lock_state_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int on, err;

	if (kstrtoint(buf, 10, &on) ||
			on < 0 || on > 1)
		return -EINVAL;

	err = gaokun_set_fn_lock(dev, on + 1);
	if (err)
		return err;

	return size;
}

static DEVICE_ATTR_RW(fn_lock_state);

/* Thermal Zone */
/* Range from 0 to 0x2C, partial valid */
static u8 temp_reg[] = {0x05, 0x07, 0x08, 0x0E, 0x0F, 0x12, 0x15,
						0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24,
						0x25, 0x26, 0x27, 0x28, 0x29, 0x2A};
static int gaokun_get_temp(struct device *dev, s16 temp[sizeof(temp_reg)])
{
    /* GTMP */
    struct gaokun_wmi *ecwmi = dev_get_drvdata(dev);
	u8 *obuf;
	int i = 0;

	while (i < sizeof(temp_reg)) {
		obuf = ec_command_data(ecwmi->ec, 0x02, 0x61, 1, (u8 []){temp_reg[i]}, 4);
		if(*obuf)
			return -EIO;
		temp[i++] = *(s16 *)(obuf + 2);
		usleep_range(1200, 1500);
	}

	return 0;
}

static ssize_t temperature_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int err;
	s16 temp[sizeof(temp_reg)];
	int len, i;
	char *ptr = buf;

	err = gaokun_get_temp(dev, temp);
	if (err)
		return err;

	len = i = 0;
	while (i < sizeof(temp_reg)) {
		// sysfs_emit does not work, why?
        s16 value = temp[i++];
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
	&dev_attr_charge_control_start_threshold.attr,
	&dev_attr_charge_control_end_threshold.attr,
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

    /* make it under /sys/devices/platform, convenient for sysfs I/O, while adev
     * is under /sys/devices/platform/soc@0/ac0000.geniqup/a9c000.i2c/i2c-5/5-0038/
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
