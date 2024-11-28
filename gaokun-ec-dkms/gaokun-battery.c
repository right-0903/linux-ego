// SPDX-License-Identifier: GPL-2.0-only
/*
 * gaokun-battery - A power supply driver for HUAWEI Matebook E Go (sc8280xp)
 *
 * reference: drivers/power/supply/lenovo_yoga_c630_battery.c
 *            drivers/platform/arm64/acer-aspire1-ec.c
 *            drivers/acpi/battery.c
 *            drivers/acpi/ac.c
 *
 * Copyright (C) 2024 nuvole <mitltlatltl@gmail.com>
 */

#include <linux/auxiliary_bus.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/sprintf.h>

#include "ec.h"


/* -------------------------------------------------------------------------- */
/* String Data Reg */

#define EC_BAT_VENDOR			0x01 /* from 0x01 to 0x0F, SUNWODA */
#define EC_BAT_MODEL			0x11 /* from 0x11 to 0x1F, HB30A8P9ECW-22T */

#define EC_ADP_STATUS			0x81
#define EC_AC_STATUS			BIT(0)
#define EC_BAT_PRESENT			BIT(1) /* BATC._STA */

#define EC_BAT_STATUS			0x82 /* _BST */
#define EC_BAT_DISCHARGING		BIT(0)
#define EC_BAT_CHARGING			BIT(1)
#define EC_BAT_CRITICAL			BIT(2) /* Low Battery Level */
#define EC_BAT_FULL				BIT(3)

/* -------------------------------------------------------------------------- */
/* Word Data Reg */

/* 0x5A: ?
 * 0x5C: ?
 * 0x5E: ?
 * 0X60: ?
 * 0x84: ?
 */

#define EC_BAT_STATUS_START		0x90
#define EC_BAT_PERCENTAGE		0x90
#define EC_BAT_VOLTAGE			0x92
#define EC_BAT_CAPACITY			0x94
#define EC_BAT_FULL_CAPACITY	0x96
/* 0x98: ? */
#define EC_BAT_CURRENT			0x9A
/* 0x9C: ? */

#define EC_BAT_INFO_START		0xA0
/* 0xA0: POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT? */
#define EC_BAT_DESIGN_CAPACITY	0xA2
#define EC_BAT_DESIGN_VOLTAGE	0xA4
#define EC_BAT_SERIAL_NUMBER	0xA6
#define EC_BAT_CYCLE_COUNT		0xAA

/* -------------------------------------------------------------------------- */
/* Battery Event ID */

#define EC_EVENT_BAT_A0			0xA0
#define EC_EVENT_BAT_A1			0xA1
#define EC_EVENT_BAT_A2			0xA2
#define EC_EVENT_BAT_A3			0xA3
#define EC_EVENT_BAT_B1			0xB1
/* EVENT B1 A0 A1 repeat about every 1s 2s 3s respectively */

#define MILLI_TO_MICRO			1000
#define CACHE_TIME				1000 /* cache time in milliseconds */


struct gaokun_psy_bat_status {
	__le16 percentage_now;	/* 0x90 */
	__le16 voltage_now;
	__le16 capacity_now;
	__le16 full_capacity;
	__le16 unknown1;
	__le16 rate_now;
	__le16 unknown2;		/* 0x9C */
} __packed;

struct gaokun_psy_bat_info {
	__le16 unknown3;		/* 0xA0 */
	__le16 design_capacity;
	__le16 design_voltage;
	__le16 serial_number;
	__le16 padding2;
	__le16 cycle_count;		/* 0xAA */
} __packed;

struct gaokun_psy {
	struct gaokun_ec *ec;
	struct device *dev;
	struct notifier_block nb;

	struct power_supply *bat_psy;
	struct power_supply *adp_psy;

	unsigned long update_time;
	struct gaokun_psy_bat_status status;
	struct gaokun_psy_bat_info info;

	char battery_model[0x10]; /* HB30A8P9ECW-22T, the real one is XXX-22A */
	char battery_serial[0x10];
	char battery_vendor[0x10];

	int charge_now;
	int online;
	int bat_present;
};

/* -------------------------------------------------------------------------- */
/* Adapter */

static int gaokun_psy_get_adp_status(struct gaokun_psy *ecbat)
{
	/* _PSR */
	int ret;
	u8 online = 0;

	ret = gaokun_ec_read_byte(ecbat->ec, EC_ADP_STATUS, &online);
	ecbat->online = !!(online & EC_AC_STATUS);

	return ret;
}

static int gaokun_psy_get_adp_property(struct power_supply *psy,
									   enum power_supply_property psp,
									   union power_supply_propval *val)
{
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);

	if (gaokun_psy_get_adp_status(ecbat))
		return -EIO;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = ecbat->online;
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static enum power_supply_property gaokun_psy_adp_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc gaokun_psy_adp_desc = {
	.name			= "gaokun-ec-adapter",
	.type			= POWER_SUPPLY_TYPE_USB_TYPE_C,
	.get_property	= gaokun_psy_get_adp_property,
	.properties		= gaokun_psy_adp_props,
	.num_properties	= ARRAY_SIZE(gaokun_psy_adp_props),
};

/* -------------------------------------------------------------------------- */
/* Battery */

static inline void gaokun_psy_get_bat_present(struct gaokun_psy *ecbat)
{
	int ret;
	u8 present;

	gaokun_ec_write(ecbat->ec, (u8 []){0x02, 0xB2, 1, 0x90});
	ret = gaokun_ec_read_byte(ecbat->ec, EC_ADP_STATUS, &present);

	ecbat->bat_present = ret ? false: !!(present & EC_BAT_PRESENT);
}

static inline int gaokun_psy_bat_present(struct gaokun_psy *ecbat)
{
	return ecbat->bat_present;
}

static int gaokun_psy_get_bat_info(struct gaokun_psy *ecbat)
{
	/* _BIX */
	if (!gaokun_psy_bat_present(ecbat))
		return 0;

	return gaokun_ec_multi_read(ecbat->ec, EC_BAT_INFO_START,
								sizeof(ecbat->info), (u8 *)&ecbat->info);
}

static inline void gaokun_psy_update_bat_charge(struct gaokun_psy *ecbat)
{
	u8 charge = 0;

	gaokun_ec_read_byte(ecbat->ec, EC_BAT_STATUS, &charge);

	 charge = (charge == EC_BAT_CHARGING) * POWER_SUPPLY_STATUS_CHARGING
			+ (charge == EC_BAT_DISCHARGING) * POWER_SUPPLY_STATUS_DISCHARGING
			+ (charge == EC_BAT_FULL) * POWER_SUPPLY_STATUS_FULL;

	ecbat->charge_now = charge;
}

static int gaokun_psy_get_bat_status(struct gaokun_psy *ecbat)
{
	/* _BST */
	int ret;

	if (time_before(jiffies, ecbat->update_time +
		msecs_to_jiffies(CACHE_TIME)))
		return 0;

	gaokun_psy_update_bat_charge(ecbat);
	ret = gaokun_ec_multi_read(ecbat->ec, EC_BAT_STATUS_START,
							   sizeof(ecbat->status), (u8 *)&ecbat->status);

	ecbat->update_time = jiffies;

	return ret;
}

static inline void gaokun_psy_init(struct gaokun_psy *ecbat)
{
	gaokun_psy_get_bat_present(ecbat);
	if (!gaokun_psy_bat_present(ecbat))
		return ;

	gaokun_psy_get_bat_info(ecbat);

	snprintf(ecbat->battery_serial, sizeof(ecbat->battery_serial),
			 "%d", le16_to_cpu(ecbat->info.serial_number));

	gaokun_ec_multi_read(ecbat->ec, EC_BAT_VENDOR,
						 sizeof(ecbat->battery_vendor) - 1,
						 ecbat->battery_vendor);

	gaokun_ec_multi_read(ecbat->ec, EC_BAT_MODEL,
						 sizeof(ecbat->battery_model) - 1,
						 ecbat->battery_model);

	ecbat->battery_model[14] = 'A'; /* FIX UP */
}

static int gaokun_psy_get_bat_property(struct power_supply *psy,
									   enum power_supply_property psp,
									   union power_supply_propval *val)
{
	struct gaokun_psy *ecbat = power_supply_get_drvdata(psy);

	if (gaokun_psy_bat_present(ecbat))
		gaokun_psy_get_bat_status(ecbat);
	else if (psp != POWER_SUPPLY_PROP_PRESENT)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = ecbat->charge_now;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = ecbat->bat_present;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = le16_to_cpu(ecbat->info.cycle_count);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = le16_to_cpu(ecbat->info.design_voltage) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = le16_to_cpu(ecbat->status.voltage_now) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = (s16)le16_to_cpu(ecbat->status.rate_now) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = le16_to_cpu(ecbat->info.design_capacity) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = le16_to_cpu(ecbat->status.full_capacity) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = le16_to_cpu(ecbat->status.capacity_now) * MILLI_TO_MICRO;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = le16_to_cpu(ecbat->status.percentage_now);
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = ecbat->battery_model;
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = ecbat->battery_vendor;
		break;

	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = ecbat->battery_serial;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property gaokun_psy_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc gaokun_psy_bat_desc = {
	.name			= "gaokun-ec-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= gaokun_psy_get_bat_property,
	.properties		= gaokun_psy_bat_props,
	.num_properties	= ARRAY_SIZE(gaokun_psy_bat_props),
};

static int gaokun_psy_notify(struct notifier_block *nb,
							 unsigned long action, void *data)
{
	struct gaokun_psy *ecbat = container_of(nb, struct gaokun_psy, nb);

	switch (action) {
	case EC_EVENT_BAT_A2:
	case EC_EVENT_BAT_B1:
		gaokun_psy_get_bat_info(ecbat);
		break;

	case EC_EVENT_BAT_A0: /* plug in or plug out */
		gaokun_psy_get_adp_status(ecbat);
		power_supply_changed(ecbat->adp_psy);
		msleep(10);
		fallthrough;

	case EC_EVENT_BAT_A1:
	case EC_EVENT_BAT_A3:
		if (action == EC_EVENT_BAT_A3) {
			gaokun_psy_get_bat_info(ecbat);
			msleep(100);
		}
		gaokun_psy_get_bat_status(ecbat);
		power_supply_changed(ecbat->bat_psy);
		break;
	}

	dev_info(ecbat->dev, "EC_EVENT_BAT_%02lX event triggered\n", action);
	return NOTIFY_OK;
}

static int gaokun_psy_probe(struct auxiliary_device *adev,
							const struct auxiliary_device_id *id)
{
	struct gaokun_ec *ec = adev->dev.platform_data;
	struct power_supply_config psy_cfg = {};
	struct device *dev = &adev->dev;
	struct gaokun_psy *ecbat;

	ecbat = devm_kzalloc(&adev->dev, sizeof(*ecbat), GFP_KERNEL);
	if (!ecbat)
		return -ENOMEM;

	ecbat->ec = ec;
	ecbat->dev = dev;
	ecbat->nb.notifier_call = gaokun_psy_notify;

	auxiliary_set_drvdata(adev, ecbat);

	psy_cfg.drv_data = ecbat;
	psy_cfg.supplied_to = (char **)&gaokun_psy_bat_desc.name;
	psy_cfg.num_supplicants = 1;

	ecbat->adp_psy = devm_power_supply_register(dev, &gaokun_psy_adp_desc,
												&psy_cfg);
	if (IS_ERR(ecbat->adp_psy))
		return dev_err_probe(dev, PTR_ERR(ecbat->adp_psy),
							 "Failed to register AC power supply\n");

	ecbat->bat_psy = devm_power_supply_register(dev, &gaokun_psy_bat_desc,
												&psy_cfg);
	if (IS_ERR(ecbat->bat_psy))
		return dev_err_probe(dev, PTR_ERR(ecbat->bat_psy),
					"Failed to register battery power supply\n");

	gaokun_psy_init(ecbat);
	dev_info(ecbat->dev, "EC battery init\n");

	return gaokun_ec_register_notify(ec, &ecbat->nb);
}

static void gaokun_psy_remove(struct auxiliary_device *adev)
{
	struct gaokun_psy *ecbat = auxiliary_get_drvdata(adev);

	gaokun_ec_unregister_notify(ecbat->ec, &ecbat->nb);
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
