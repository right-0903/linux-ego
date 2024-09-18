// SPDX-License-Identifier: GPL-2.0-only
/*
 * gaokun-ec - An EC driver for HUAWEI Matebook E Go (sc8280xp)
 *
 * reference: drivers/platform/arm64/acer-aspire1-ec.c
 *            drivers/platform/arm64/lenovo-yoga-c630.c
 *
 * Copyright (C) 2024 nuvole <mitltlatltl@gmail.com>
 */

#include <asm-generic/unaligned.h>
#include <linux/auxiliary_bus.h>
#include <linux/bits.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/notifier.h>
#include <drm/drm_bridge.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
#include <linux/workqueue_types.h>
#else
#include <linux/workqueue.h>
#endif

#include "ec.h"


struct gaokun_ec {
	struct i2c_client *client;
    struct mutex lock;
	struct blocking_notifier_head notifier_list;

	struct input_dev *idev;

	bool suspended;

};

u8 usb_data[9];
EXPORT_SYMBOL_GPL(usb_data);

u8 *ec_command_data(struct gaokun_ec *ec, u8 mcmd, u8 scmd, u8 ilen, const u8 *buf, u8 olen)
{
	/* The C implementation of ACPI method ECCD
	 * FIXME: ECCD share the OBUF as a field DOBF,
	 * but it has an impact on thread safty
	 */
	u8 ibuf[EC_INPUT_BUFFER_LENGTH];
	static u8 obuf[EC_OUTPUT_BUFFER_LENGTH];
	int ret;

	mutex_lock(&ec->lock);

	ibuf[0] = scmd;
	ibuf[1] = ilen;

	if (ilen > EC_INPUT_BUFFER_LENGTH || olen > EC_OUTPUT_BUFFER_LENGTH) { /* overflow */
		obuf[0] = 0x02; // ACPI do this, intention?
		goto err;
	}

	if(ilen > 0){
		// copy the data block
		memcpy(ibuf + 2, buf, ilen);
	}
	else{
		ibuf[2] = 0; // ACPI use ibuf[3] = buf, may be unnecessary.
	}

	switch(ilen) {
		case 0: case 1: case 2: case 3: case 4: case 5:
		case 0x18:
			ret = i2c_smbus_write_i2c_block_data(ec->client, mcmd, ilen + 2, ibuf);
			if (ret) {
				dev_err(&ec->client->dev, "I2C EC write failed with error code: %d\n", ret);
				goto err;
			}
			break;
		default:
			// ACPI allow this, so we don't goto err.
			dev_warn(&ec->client->dev, "Unsupported input data buffer length: %d\n", ilen);
	}

	usleep_range(2500, 3000); // Sleep (0x02)


	switch(olen) {
		case 1: case 2: case 3: case 4: case 5: case 6:
		case 7: case 8: case 0xF: case 0x16: case 0x20:
		case 0x23: case 0x40: case 0xFE: case 0xFF: case 0x9:
			ret = i2c_smbus_read_i2c_block_data(ec->client, mcmd, olen, obuf);
			if (ret < 0) {
				dev_err(&ec->client->dev, "I2C EC read failed with error code: %d\n", ret);
				goto err;
			}else{
				dev_info(&ec->client->dev, "I2C EC read successful, data: %*ph\n", olen, obuf);
			}
			break;
		default:
			dev_warn(&ec->client->dev, "Unsupported output data buffer length: %d\n", olen);
	}
	// set to zero, to indicate status for the convinient of transplanting ACPI method
	obuf[0] = 0;

	mutex_unlock(&ec->lock);
	return obuf;

err:
	obuf[0] = 1;
	mutex_unlock(&ec->lock);
	return obuf;
}
EXPORT_SYMBOL_GPL(ec_command_data);

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
        //  \_SB.GPU0.LIDB = INL3 ?
		break;

	case 0xC0:
		obuf = ec_command_data(ec, 0x02, 0xE7, 0, NULL, 15);
		dev_info(&ec->client->dev, "ThermalZone event triggered, data: %*ph\n", 15, obuf);
		break;

	case 0x0A: case 0x0B: case 0x0C: case 0x0D: case 0x0E: case 0x0F:
	case 0x10: case 0x12: case 0x13: case 0x17: case 0x19: case 0x20:
	case 0x23: case 0xBF: case 0x18: // case 0x21: case 0x22:
		dev_info(&ec->client->dev, "WMI event triggered\n");
		break;

	case EC_EVENT_USB:
		obuf = ec_command_data(ec, 0x03, 0xD3, 0, NULL, 9);
		pr_info_ratelimited("%s: USB EVENT, DATA: %*ph\n", __func__, 9, obuf);
		memcpy(usb_data, obuf, 9);
		blocking_notifier_call_chain(&ec->notifier_list, id, ec);
		break;

	default:
		blocking_notifier_call_chain(&ec->notifier_list, id, ec);
	}

	return IRQ_HANDLED;
}

int gaokun_ec_register_notify(struct gaokun_ec *ec, struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ec->notifier_list, nb);
}
EXPORT_SYMBOL_GPL(gaokun_ec_register_notify);

void gaokun_ec_unregister_notify(struct gaokun_ec *ec, struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&ec->notifier_list, nb);
}
EXPORT_SYMBOL_GPL(gaokun_ec_unregister_notify);

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
	int i;

	if (!ec->suspended)
		return 0;

	/* The EC or I2C host can be grumpy when waking up */
	for (i = 0; i < 3; i++) {
		obuf = ec_command_data(ec, 0x02, 0x2, 1, (u8 []){0xEB}, 2);
		if (obuf[0] == 0) break;

		msleep(10);
	}

	ec->suspended = false;

	return 0;
}

static void gaokun_aux_release(struct device *dev)
{
	struct auxiliary_device *adev = to_auxiliary_dev(dev);

	kfree(adev);
}

static void gaokun_aux_remove(void *data)
{
	struct auxiliary_device *adev = data;

	auxiliary_device_delete(adev);
	auxiliary_device_uninit(adev);
}

static int gaokun_aux_init(struct device *parent, const char *name,
			      struct gaokun_ec *ec)
{
	struct auxiliary_device *adev;
	int ret;

	adev = kzalloc(sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->name = name;
	adev->id = 0;
	adev->dev.parent = parent;
	adev->dev.release = gaokun_aux_release;
	adev->dev.platform_data = ec;

	ret = auxiliary_device_init(adev);
	if (ret) {
		kfree(adev);
		return ret;
	}

	ret = auxiliary_device_add(adev);
	if (ret) {
		auxiliary_device_uninit(adev);
		return ret;
	}

	return devm_add_action_or_reset(parent, gaokun_aux_remove, adev);
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
	BLOCKING_INIT_NOTIFIER_HEAD(&ec->notifier_list);

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

	/* Battery and Adapter */
	ret = gaokun_aux_init(dev, "psy", ec);
	if (ret)
		return ret;

	/* UCSI */
	ret = gaokun_aux_init(dev, "ucsi", ec);
	if (ret)
		return ret;

	/* Altmode */
	ret = gaokun_aux_init(dev, "altmode", ec);
	if (ret)
		return ret;

	/* WMI */
	ret = gaokun_aux_init(dev, "wmi", ec);
	if (ret)
		return ret;

	/* irq */
	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					gaokun_ec_irq_handler, IRQF_ONESHOT,
					dev_name(dev), ec);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request irq\n");

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
