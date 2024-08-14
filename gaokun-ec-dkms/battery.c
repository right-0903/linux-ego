// SPDX-License-Identifier: GPL-2.0
#include <linux/power_supply.h>
#include <linux/sprintf.h>
#include "ec.h"
#include "battery.h"


static char gaokun_battery_vendor[0x10]; // SUNWODA, should be safe without tail '\0'
static char gaokun_battery_serial[0x10];

struct gaokun_ec_bat_psy_static_data {
	__le16 design_voltage;
	__le16 design_capacity;
	const char * const serial_number;
	const char * const vendor;
} __packed;

static struct gaokun_ec_bat_psy_static_data static_data = {
	.vendor = gaokun_battery_vendor,
	.serial_number = gaokun_battery_serial
};

static inline void gaokun_get_static_battery_data(struct gaokun_ec *ec)
{
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
}

struct gaokun_ec_bat_psy_dynamic_data {
	__le16 capacity_now;
	__le16 voltage_now;
	__le16 current_now;
	__le16 percentage_now;
	__le16 BCLP; // last_full_capacity
	__le16 BCCL; // battery charging cycle
} __packed;

static int gaokun_ec_bat_psy_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct gaokun_ec *ec = power_supply_get_drvdata(psy);
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
		//dev_info(&ec->client->dev, "battery_status_now: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ec_read_word_data(ec, EC_BAT_BCCL_L, EC_BAT_BCCL_H, &dynamic_data.BCCL);
		val->intval = le16_to_cpu(dynamic_data.BCCL);
		dev_info(&ec->client->dev, "battery_cycle: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ec_read_word_data(ec, EC_BAT_VOLT_L, EC_BAT_VOLT_H, &dynamic_data.voltage_now);
		val->intval = le16_to_cpu(dynamic_data.voltage_now) * MILLI_TO_MICRO;
		dev_info(&ec->client->dev, "voltage_now: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = le16_to_cpu(static_data.design_voltage) * MILLI_TO_MICRO;
		dev_info(&ec->client->dev, "design_voltage: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ec_read_word_data(ec, EC_BAT_CPCT_L, EC_BAT_CPCT_H, &dynamic_data.capacity_now);
		val->intval = le16_to_cpu(dynamic_data.capacity_now) * MILLI_TO_MICRO;
		dev_info(&ec->client->dev, "capacity_now: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ec_read_word_data(ec, EC_BAT_BCLP_L, EC_BAT_BCLP_H, &dynamic_data.BCLP);
		val->intval = le16_to_cpu(dynamic_data.BCLP) * MILLI_TO_MICRO;
		dev_info(&ec->client->dev, "last_full_capacity: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = le16_to_cpu(static_data.design_capacity) * MILLI_TO_MICRO;
		dev_info(&ec->client->dev, "design_capacity: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		ec_read_word_data(ec, EC_BAT_BCAP_L, EC_BAT_BCAP_H, &dynamic_data.percentage_now);
		val->intval = le16_to_cpu(dynamic_data.percentage_now);
		dev_info(&ec->client->dev, "battery percentage: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ec_read_word_data(ec, EC_BAT_CRNT_L, EC_BAT_CRNT_H, &dynamic_data.current_now);
		val->intval = (s16)le16_to_cpu(dynamic_data.current_now) * MILLI_TO_MICRO;
		dev_info(&ec->client->dev, "current_now: %d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
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
		dev_info(&ec->client->dev, "serial_number: %s\n", static_data.serial_number);
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
	.name		= "gaokun-ec-bat",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= gaokun_ec_bat_psy_get_property,
	.properties	= gaokun_ec_bat_psy_props,
	.num_properties	= ARRAY_SIZE(gaokun_ec_bat_psy_props),
};

static int gaokun_ec_adp_psy_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct gaokun_ec *ec = power_supply_get_drvdata(psy);
	u8 *obuf;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		obuf = ec_command_data(ec, 0x02, EC_READ, 1, (u8 []){EC_ADP_STATE}, 3);
		val->intval = obuf[3] & EC_AC_STATUS;
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
	.name		= "gaokun-ec-adp",
	.type		= POWER_SUPPLY_TYPE_USB_TYPE_C,
	.get_property	= gaokun_ec_adp_psy_get_property,
	.properties	= gaokun_ec_adp_psy_props,
	.num_properties	= ARRAY_SIZE(gaokun_ec_adp_psy_props),
};

void battery_event_handler(struct gaokun_ec *ec, int event_id)
{
    switch (event_id) {
    case EC_EVENT_BAT_A0:
		power_supply_changed(ec->adp_psy);
        usleep_range(12000, 15000);
		power_supply_changed(ec->bat_psy);
		dev_info(&ec->client->dev, "EC_EVENT_BAT_A0 event triggered\n");
		break;

	case EC_EVENT_BAT_A1:
		power_supply_changed(ec->bat_psy);
		dev_info(&ec->client->dev, "EC_EVENT_BAT_A1 event triggered\n");
		break;

	case EC_EVENT_BAT_A2:
        // Notify (\_SB.BATC, 0x81) // Information Change
		dev_info(&ec->client->dev, "EC_EVENT_BAT_A2 event triggered\n");
		fallthrough;

	case EC_EVENT_BAT_A3:
        // usleep_range(120000, 150000);
		power_supply_changed(ec->bat_psy);
		dev_info(&ec->client->dev, "EC_EVENT_BAT_A3 event triggered\n");
		break;

	case EC_EVENT_BAT_B1: // detach
		dev_info(&ec->client->dev, "EC_EVENT_BAT_B1 event triggered\n");
		break;
    }
}

int gaokun_battery_setup(struct gaokun_ec *ec)
{
	struct power_supply_config psy_cfg = {0};
    struct device *dev = &ec->client->dev;

	/* Battery status reports */
	psy_cfg.drv_data = ec;
	ec->bat_psy = devm_power_supply_register(dev, &gaokun_ec_bat_psy_desc, &psy_cfg);
	if (IS_ERR(ec->bat_psy))
		return dev_err_probe(dev, PTR_ERR(ec->bat_psy),
				     "Failed to register battery power supply\n");

	gaokun_get_static_battery_data(ec); // get static data, only once

	ec->adp_psy = devm_power_supply_register(dev, &gaokun_ec_adp_psy_desc, &psy_cfg);
	if (IS_ERR(ec->adp_psy))
		return dev_err_probe(dev, PTR_ERR(ec->adp_psy),
				     "Failed to register AC power supply\n");

    pr_info("%s: EC battery init\n", __func__);

	return 0;
}

void gaokun_battery_cleanup(void)
{
    pr_info("%s: EC battery exit\n", __func__);
}
