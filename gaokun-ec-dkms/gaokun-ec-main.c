// SPDX-License-Identifier: GPL-2.0
/*
 * gaokun-ec - An EC driver for HUAWEI Matebook E Go (sc8280xp)
 *
 * based on acer-aspire1-ec.c : https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/platform/arm64/acer-aspire1-ec.c
 *
 * Copyright (C) 2024 nuvole <mitltlatltl@gmail.com>
 */

#include <asm-generic/unaligned.h>
#include <linux/bits.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue_types.h>

#include "ec.h"
#include "battery.h"


static inline int gaokun_get_event(struct gaokun_ec *ec)
{
	u8 *obuf = ec_command_data(ec, EC_EVENT, EC_QUERY_EVENT, 0, NULL, 4);
	return obuf[2];
}

static irqreturn_t gaokun_ec_irq_handler(int irq, void *data)
{
	struct gaokun_ec *ec = data;
	int id;
	u8 *obuf, status;

	id = gaokun_get_event(ec);

	switch (id) {
	case 0x0: /* No event */
		break;

	case EC_EVENT_LID:
		obuf = ec_command_data(ec, 0x2, EC_READ, 1, (u8 []){EC_LID_STATE}, 4);
		status = obuf[2];
		status &= EC_LID_CLOSE;
		status >>= 1;
		input_report_switch(ec->idev, SW_LID, status); // \_SB.IC16.LID0.LIDB = INL3
		input_sync(ec->idev);
		dev_info(&ec->client->dev, "lid status change event triggered, data: %*ph\n", 4, obuf);
		break;

	case 0xC0:
		obuf = ec_command_data(ec, 0x02, 0xE7, 0, NULL, 15);
		dev_info(&ec->client->dev, "ThermalZone event triggered, data: %*ph\n", 15, obuf);
		break;

	case 0x0A: case 0x0B: case 0x0C: case 0x0D: case 0x0E: case 0x0F:
	case 0x10: case 0x12: case 0x13: case 0x17: case 0x19: case 0x20:
	case 0x23: case 0xBF: case 0x18: // case 0x21: case 0x22:
		// Windows Management Instrumentation Device
		dev_info(&ec->client->dev, "WMI event triggered\n");
		break;

	// do not handle for now
	case EC_EVENT_USB:
		obuf = ec_command_data(ec, 0x03, 0xD3, 0, NULL, 9);
		// 0x2: free, 0x4: charging, 0x8: display connected
		if (obuf[4] == 0x8 || obuf[6] == 0x8){
			dev_info(&ec->client->dev, "USB event triggered, connected, data is %*ph\n", 9, obuf);
		}else{
			dev_info(&ec->client->dev, "USB event triggered, disconnected, data is %*ph\n", 9, obuf);
		}
		break;
/*
	case EC_EVENT_UCSI:
		u32 cci;
		ec->ucsi->ops->read_cci(ec->ucsi, &cci);
		ucsi_notify_common(ec->ucsi, cci);
		dev_info(&ec->client->dev, "UCSI event triggered, cci is %*ph\n", 4, (u8 *)&cci);
		break;
*/

	case EC_EVENT_BAT_A0: case EC_EVENT_BAT_A1: case EC_EVENT_BAT_A2: case EC_EVENT_BAT_A3: case EC_EVENT_BAT_B1:
		battery_event_handler(ec, id);
		break;

	default:
		dev_warn(&ec->client->dev, "Unknown event id: 0x%x\n", id);
	}

	return IRQ_HANDLED;
}

/* Modern Standby */
static int gaokun_ec_suspend(struct device *dev)
{
	struct gaokun_ec *ec = dev_get_drvdata(dev);
	u8 *obuf;

	if (ec->suspended)
		return 0;

	obuf = ec_command_data(ec, 0x02, 0xB2, 1, (u8 []){0xDB}, 2);

	if (obuf[0])
		return obuf[0];

	ec->suspended = true;

	return 0;
}

static int gaokun_ec_resume(struct device *dev)
{
	struct gaokun_ec *ec = dev_get_drvdata(dev);
	u8 *obuf;

	if (!ec->suspended)
		return 0;

	obuf = ec_command_data(ec, 0x02, 0x2, 1, (u8 []){0xEB}, 2);
	if (obuf[0])
		return obuf[0];

	ec->suspended = false;

	return 0;
}

static int gaokun_ec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gaokun_ec *ec;
	int ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	mutex_init(&ec->lock);
	ec->client = client;
	i2c_set_clientdata(client, ec);

	/* Battery and Adapter */
	gaokun_battery_setup(ec);

	/* Lid switch */
	ec->idev = devm_input_allocate_device(dev);
	if (!ec->idev)
		return -ENOMEM;

	ec->idev->name = "LID";
	ec->idev->phys = "gaokun-ec/input0";
	input_set_capability(ec->idev, EV_SW, SW_LID);

	ret = input_register_device(ec->idev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register input device\n");

	/* irq */
	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					gaokun_ec_irq_handler, IRQF_ONESHOT,
					dev_name(dev), ec);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request irq\n");

	dev_warn(&ec->client->dev, "module init, this is an experimental EC driver, at your risks");

	return 0;
}

static const struct i2c_device_id gaokun_ec_id[] = {
	{ "gaokun-ec", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gaokun_ec_id);

static const struct of_device_id gaokun_ec_of_match[] = {
	{ .compatible = "huawei,gaokun-ec", },
	{ }
};
MODULE_DEVICE_TABLE(of, gaokun_ec_of_match);

static const struct dev_pm_ops gaokun_ec_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(gaokun_ec_suspend, gaokun_ec_resume)
};

static struct i2c_driver gaokun_ec_driver = {
	.driver = {
		.name = "gaokun-ec",
		.of_match_table = gaokun_ec_of_match,
		.pm = &gaokun_ec_pm_ops,
	},
	.probe = gaokun_ec_probe,
	.id_table = gaokun_ec_id,
};
module_i2c_driver(gaokun_ec_driver);

MODULE_DESCRIPTION("EC driver for HUAWEI Matebook E Go");
MODULE_AUTHOR("nuvole <mitltlatltl@gmail.com>");
MODULE_LICENSE("GPL");
