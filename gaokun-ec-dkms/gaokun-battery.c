// SPDX-License-Identifier: GPL-2.0-only
/*
 * gaokun-battery - A power supply driver for HUAWEI Matebook E Go (sc8280xp)
 *
 * Copyright (C) 2024 nuvole <mitltlatltl@gmail.com>
 */

#include <linux/auxiliary_bus.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/sprintf.h>

#include "ec.h"

#define EC_BAT_VENDOR 0x01 /* from 0x01 to 0x16, SUNWODA */

/* _L: lower EC reg address, _H: higer EC reg address
 *
 * The following abbv.s are ACPI's invention
 * BCLP: last full charge, abbv. of Battery Capacity Last Point(by GPT) ?
 * DSCP: design capacity
 * DSVO: design voltage
 * SRNM: serial numbers
 * BCCL: battery charging cycle
 *
 * The following abbv.s are defined by me.
 * VOLT: voltage now
 * CPCT: capacity now
 * CRNT: current now
 * BCAP: percentage now
 */
#define EC_BAT_BCAP_L		0x90
#define EC_BAT_BCAP_H		0x91
#define EC_BAT_VOLT_L		0x92
#define EC_BAT_VOLT_H		0x93
#define EC_BAT_CPCT_L		0x94
#define EC_BAT_CPCT_H		0x95
#define EC_BAT_BCLP_L		0x96
#define EC_BAT_BCLP_H		0x97
#define EC_BAT_CRNT_L		0x9A
#define EC_BAT_CRNT_H		0x9B
#define EC_BAT_DSCP_L		0xA2
#define EC_BAT_DSCP_H		0xA3
#define EC_BAT_DSVO_L		0xA4
#define EC_BAT_DSVO_H		0xA5
#define EC_BAT_SRNM_L		0xA6
#define EC_BAT_SRNM_H		0xA7
#define EC_BAT_BCCL_L		0xAA
#define EC_BAT_BCCL_H		0xAB

#define EC_EVENT_BAT_A0		0xA0
#define EC_EVENT_BAT_A1		0xA1
#define EC_EVENT_BAT_A2		0xA2
#define EC_EVENT_BAT_A3		0xA3
#define EC_EVENT_BAT_B1		0xB1

#define MILLI_TO_MICRO		1000

/* possible value: 1: discharge 2: charge, other, maybe 8:full */
#define EC_BAT_STATE		0x82
#define EC_BAT_DISCHARGING	BIT(0)
#define EC_BAT_CHARGING		BIT(1)

/* possible value: 2: detach, 3: attach */
#define EC_ADP_STATE		0x81
#define EC_AC_STATUS 		BIT(0)
#define EC_BAT_PRESENT		BIT(1)


struct gaokun_psy {
	struct gaokun_ec *ec;
	struct device *dev;
	struct notifier_block nb;

	struct power_supply *bat_psy;
	struct power_supply *adp_psy;
};

static char gaokun_battery_vendor[0x10]; // SUNWODA, should be safe without tail '\0'
static char gaokun_battery_serial[0x10];

struct gaokun_ec_bat_psy_static_data {
	__le16 design_voltage;
	__le16 design_capacity;
	const char * const serial_number;
	const char * const vendor;
	int present;
};

static struct gaokun_ec_bat_psy_static_data static_data = {
	.vendor = gaokun_battery_vendor,
	.serial_number = gaokun_battery_serial,
	.present = 0,
};

static inline void gaokun_get_static_battery_data(struct gaokun_psy *ecbat)
{
	struct gaokun_ec *ec = ecbat->ec;
	int cnt, input;
	u8 *obuf;
	u16 val;
	cnt = 0;
	input = EC_BAT_VENDOR;
	while (cnt < 0x10) {
		obuf = ec_command_data(ec, 0x02, EC_READ, 1, (u8 []){input}, 4);
		gaokun_battery_vendor[cnt] = (char)obuf[2];
		cnt = input++;
	}

	/* design capacity */
	ec_read_word_data(ec, EC_BAT_DSCP_L, EC_BAT_DSCP_H, &static_data.design_capacity);

	/* design voltage */
	ec_read_word_data(ec, EC_BAT_DSVO_L, EC_BAT_DSVO_H, &static_data.design_voltage);

	/* serial numbers */
	ec_read_word_data(ec, EC_BAT_SRNM_L, EC_BAT_SRNM_H, &val);
	snprintf(gaokun_battery_serial, sizeof(gaokun_battery_serial), "%d", val);

	/* battery present */
	ec_command_data(ec, 0x02, 0xB2, 1, (u8 []){0x90}, 2);
	usleep_range(1200, 1500);
	obuf = ec_command_data(ec, 0x02, EC_READ, 1, (u8 []){EC_ADP_STATE}, 3);
	val = obuf[2] & EC_BAT_PRESENT;
	static_data.present = val >> 1;

}

struct gaokun_ec_bat_psy_dynamic_data {
	__le16 capacity_now;
	__le16 voltage_now;
	__le16 current_now;
	__le16 percentage_now;
	__le16 BCLP; // last_full_capacity
	__le16 BCCL; // battery charging cycle
};

static int gaokun_ec_bat_psy_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);
	struct gaokun_ec *ec = ecbat->ec;
	struct gaokun_ec_bat_psy_dynamic_data dynamic_data;
	int offset;
	u16 _val;
	u8 *obuf;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		obuf = ec_command_data(ec, 0x2, EC_READ, 1, (u8 []){EC_BAT_STATE}, 4);
		offset = 2;
		_val = obuf[offset];
		if (_val & EC_BAT_DISCHARGING)
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (_val & EC_BAT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_FULL;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ec_read_word_data(ec, EC_BAT_BCCL_L, EC_BAT_BCCL_H, &dynamic_data.BCCL);
		val->intval = le16_to_cpu(dynamic_data.BCCL);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ec_read_word_data(ec, EC_BAT_VOLT_L, EC_BAT_VOLT_H, &dynamic_data.voltage_now);
		val->intval = le16_to_cpu(dynamic_data.voltage_now) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = le16_to_cpu(static_data.design_voltage) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ec_read_word_data(ec, EC_BAT_CPCT_L, EC_BAT_CPCT_H, &dynamic_data.capacity_now);
		val->intval = le16_to_cpu(dynamic_data.capacity_now) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ec_read_word_data(ec, EC_BAT_BCLP_L, EC_BAT_BCLP_H, &dynamic_data.BCLP);
		val->intval = le16_to_cpu(dynamic_data.BCLP) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = le16_to_cpu(static_data.design_capacity) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		ec_read_word_data(ec, EC_BAT_BCAP_L, EC_BAT_BCAP_H, &dynamic_data.percentage_now);
		val->intval = le16_to_cpu(dynamic_data.percentage_now);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ec_read_word_data(ec, EC_BAT_CRNT_L, EC_BAT_CRNT_H, &dynamic_data.current_now);
		val->intval = (s16)le16_to_cpu(dynamic_data.current_now) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = static_data.present;
		break;

	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "HB30C4J7ECW-21";
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = static_data.vendor;
		break;

	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = static_data.serial_number;
		if (!val->strval)
			return -ENOMEM;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property gaokun_ec_bat_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc gaokun_ec_bat_psy_desc = {
	.name			= "gaokun-ec-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= gaokun_ec_bat_psy_get_property,
	.properties		= gaokun_ec_bat_psy_props,
	.num_properties	= ARRAY_SIZE(gaokun_ec_bat_psy_props),
};

static int gaokun_ec_adp_psy_get_property(struct power_supply *psy,
						enum power_supply_property psp,
						union power_supply_propval *val)
{
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);
	u8 *obuf;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		obuf = ec_command_data(ecbat->ec, 0x02, EC_READ, 1, (u8 []){EC_ADP_STATE}, 3);
		val->intval = obuf[2] & EC_AC_STATUS;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property gaokun_ec_adp_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc gaokun_ec_adp_psy_desc = {
	.name			= "gaokun-ec-adapter",
	.type			= POWER_SUPPLY_TYPE_USB_TYPE_C,
	.get_property	= gaokun_ec_adp_psy_get_property,
	.properties		= gaokun_ec_adp_psy_props,
	.num_properties	= ARRAY_SIZE(gaokun_ec_adp_psy_props),
};


static int gaokun_psy_notify(struct notifier_block *nb,
					unsigned long action, void *data)
{
	struct gaokun_psy *ecbat = container_of(nb, struct gaokun_psy, nb);

	switch (action) {
	case EC_EVENT_BAT_A0: /* plug in or plug out */
		power_supply_changed(ecbat->adp_psy);
		usleep_range(12000, 15000);
		power_supply_changed(ecbat->bat_psy);
		dev_info(ecbat->dev, "EC_EVENT_BAT_A0 event triggered\n");
		break;

	case EC_EVENT_BAT_A1:
		power_supply_changed(ecbat->bat_psy);
		dev_info(ecbat->dev, "EC_EVENT_BAT_A1 event triggered\n");
		break;

	case EC_EVENT_BAT_A2:
		/* Notify (\_SB.BATC, 0x81) // Information Change
		 * From ACPI specification version 6.3, section 10.2.1
		 * The static battery information has changed.
		 * For systems that have battery slots in a docking station
		 */
		dev_info(ecbat->dev, "EC_EVENT_BAT_A2 event triggered\n");
		break;

	case EC_EVENT_BAT_A3:
		power_supply_changed(ecbat->bat_psy);
		dev_info(ecbat->dev, "EC_EVENT_BAT_A3 event triggered\n");
		break;

	case EC_EVENT_BAT_B1:
		// Notify (\_SB.BATC, 0x81) // Information Change
		dev_info(ecbat->dev, "EC_EVENT_BAT_B1 event triggered\n");
		break;
	}

	return NOTIFY_OK;
}

static int gaokun_psy_probe(struct auxiliary_device *adev,
					const struct auxiliary_device_id *id)
{
	struct gaokun_ec *ec = adev->dev.platform_data;
	struct power_supply_config psy_cfg = {};
	struct device *dev = &adev->dev;
	struct gaokun_psy *ecbat;
	int ret;

	ecbat = devm_kzalloc(&adev->dev, sizeof(*ecbat), GFP_KERNEL);
	if (!ecbat)
		return -ENOMEM;

	ecbat->ec = ec;
	ecbat->dev = dev;
	ecbat->nb.notifier_call = gaokun_psy_notify;

	auxiliary_set_drvdata(adev, ecbat);

	psy_cfg.drv_data = ecbat;
	psy_cfg.supplied_to = (char **)&gaokun_ec_bat_psy_desc.name;
	psy_cfg.num_supplicants = 1;

	// ecbat->bat_psy = devm_power_supply_register(dev, &gaokun_ec_bat_psy_desc, &psy_cfg);
	ecbat->bat_psy = power_supply_register_no_ws(dev, &gaokun_ec_bat_psy_desc, &psy_cfg);
	if (IS_ERR(ecbat->bat_psy))
		return dev_err_probe(dev, PTR_ERR(ecbat->bat_psy),
					"Failed to register battery power supply\n");

	gaokun_get_static_battery_data(ecbat); // get static data, only once

	ecbat->adp_psy = power_supply_register_no_ws(dev, &gaokun_ec_adp_psy_desc, &psy_cfg);
	if (IS_ERR(ecbat->adp_psy))
		return dev_err_probe(dev, PTR_ERR(ecbat->adp_psy),
					"Failed to register AC power supply\n");

	ret = gaokun_ec_register_notify(ec, &ecbat->nb);
	if (ret)
		goto err_unreg_bat;

	dev_info(ecbat->dev, "EC battery init\n");

	return 0;

err_unreg_bat:
	power_supply_unregister(ecbat->bat_psy);
	power_supply_unregister(ecbat->adp_psy);
	return ret;
}

static void gaokun_psy_remove(struct auxiliary_device *adev)
{
	struct gaokun_psy *ecbat = auxiliary_get_drvdata(adev);

	gaokun_ec_unregister_notify(ecbat->ec, &ecbat->nb);
	power_supply_unregister(ecbat->bat_psy);
	power_supply_unregister(ecbat->adp_psy);

	dev_info(ecbat->dev, "EC battery exit\n");
}
 
static const struct auxiliary_device_id gaokun_psy_id_table[] = {
	{ .name = GAOKUN_MOD_NAME "." GAOKUN_DEV_PSY, },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, gaokun_psy_id_table);

static struct auxiliary_driver gaokun_psy_driver = {
	.name = GAOKUN_DEV_PSY,
	.id_table = gaokun_psy_id_table,
	.probe = gaokun_psy_probe,
	.remove = gaokun_psy_remove,
};

module_auxiliary_driver(gaokun_psy_driver);

MODULE_DESCRIPTION("HUAWEI Matebook E Go psy driver");
MODULE_LICENSE("GPL");
