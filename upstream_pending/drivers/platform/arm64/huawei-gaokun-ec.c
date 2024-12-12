// SPDX-License-Identifier: GPL-2.0-only
/*
 * gaokun-ec - An EC driver for HUAWEI Matebook E Go (sc8280xp)
 *
 * reference: drivers/platform/arm64/acer-aspire1-ec.c
 *            drivers/platform/arm64/lenovo-yoga-c630.c
 *
 * Copyright (C) 2024 Pengyu Luo <mitltlatltl@gmail.com>
 */

#include <linux/auxiliary_bus.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/notifier.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/version.h>

#include <linux/platform_data/huawei-gaokun-ec.h>

#define EC_EVENT		0x06

/* Also can be found in ACPI specification 12.3 */
#define EC_READ			0x80
#define EC_WRITE		0x81
#define EC_BURST		0x82
#define EC_QUERY		0x84


#define EC_EVENT_LID		0x81

#define EC_LID_STATE		0x80
#define EC_LID_OPEN		BIT(1)

#define UCSI_REG_SIZE		7

/* for tx, command sequences are arranged as
 * {master_cmd, slave_cmd, data_len, data_seq}
 */
#define REQ_HDR_SIZE		3
#define INPUT_SIZE_OFFSET	2
#define INPUT_DATA_OFFSET	3

/* for rx, data sequences are arranged as
 * {status, data_len(unreliable), data_seq}
 */
#define RESP_HDR_SIZE		2
#define DATA_OFFSET		2


struct gaokun_ec {
	struct i2c_client *client;
	struct mutex lock;
	struct blocking_notifier_head notifier_list;
	struct input_dev *idev;
	bool suspended;
};

static int gaokun_ec_request(struct gaokun_ec *ec, const u8 *req,
			     size_t resp_len, u8 *resp)
{
	struct i2c_client *client = ec->client;
	/* It looks like a I2C_SMBUS_BLOCK_PROC_CALL, but not */
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = req[INPUT_SIZE_OFFSET] + REQ_HDR_SIZE,
			.buf = req,
		}, {
			.addr = client->addr,
			.flags = client->flags | I2C_M_RD,
			.len = resp_len,
			.buf = resp,
		},
	};

	mutex_lock(&ec->lock);

	i2c_transfer(client->adapter, msgs, 2);
	usleep_range(2000, 2500); /* have a break */

	mutex_unlock(&ec->lock);

	return *resp;
}

/* -------------------------------------------------------------------------- */
/* Common API */

/**
 * gaokun_ec_read - read from EC
 * @ec: The gaokun_ec
 * @req: The sequence to request
 * @resp_len: The size to read
 * @resp: Where the data are read to
 *
 * This function is used to read data after writing a magic sequence to EC.
 * All EC operations dependent on this functions.
 *
 * Huawei uses magic sequences everywhere to complete various functions, all
 * these sequences are passed to ECCD(a ACPI method which is quiet similiar
 * to gaokun_ec_request), there is no good abstraction to generalize these
 * sequences, so just wrap it for now. Almost all magic sequences are kept
 * in this file.
 */
int gaokun_ec_read(struct gaokun_ec *ec, const u8 *req,
		   size_t resp_len, u8 *resp)
{
	return gaokun_ec_request(ec, req, resp_len, resp);
}
EXPORT_SYMBOL_GPL(gaokun_ec_read);

/**
 * gaokun_ec_write - write to EC
 * @ec: The gaokun_ec
 * @req: The sequence to request
 *
 * This function has no big difference from gaokun_ec_read. When caller care
 * only write status and no actual data are returnd, then use it.
 */
int gaokun_ec_write(struct gaokun_ec *ec, u8 *req)
{
	u8 resp[RESP_HDR_SIZE];

	return gaokun_ec_request(ec, req, sizeof(resp), resp);
}
EXPORT_SYMBOL_GPL(gaokun_ec_write);

int gaokun_ec_read_byte(struct gaokun_ec *ec, u8 *req, u8 *byte)
{
	int ret;
	u8 resp[RESP_HDR_SIZE + sizeof(*byte)];

	ret = gaokun_ec_read(ec, req, sizeof(resp), resp);
	*byte = resp[DATA_OFFSET];

	return ret;
}
EXPORT_SYMBOL_GPL(gaokun_ec_read_byte);

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

/* -------------------------------------------------------------------------- */
/* API For PSY */

int gaokun_ec_psy_multi_read(struct gaokun_ec *ec, u8 reg,
			     size_t resp_len, u8 *resp)
{
	int i, ret;
	u8 _resp[RESP_HDR_SIZE + 1];
	u8 req[REQ_HDR_SIZE + 1] = {0x02, EC_READ, 1, };

	for (i = 0; i < resp_len; ++i) {
		req[INPUT_DATA_OFFSET] = reg++;
		ret = gaokun_ec_read(ec, req, sizeof(_resp), _resp);
		if (ret)
			return -EIO;
		resp[i] = _resp[DATA_OFFSET];
	}

	return 0;
}
EXPORT_SYMBOL_GPL(gaokun_ec_psy_multi_read);

/* -------------------------------------------------------------------------- */
/* API For WMI */

/* Battery charging threshold */
int gaokun_ec_wmi_get_threshold(struct gaokun_ec *ec, u8 *value, int ind)
{
	/* GBTT */
	return gaokun_ec_read_byte(ec, (u8 []){0x02, 0x69, 1, ind}, value);
}
EXPORT_SYMBOL_GPL(gaokun_ec_wmi_get_threshold);

int gaokun_ec_wmi_set_threshold(struct gaokun_ec *ec, u8 start, u8 end)
{
	/* SBTT */
	int ret;
	u8 req[REQ_HDR_SIZE + 2] = {0x02, 0x68, 2, 3, 0x5a};

	ret = gaokun_ec_write(ec, req);
	if (ret)
		return -EIO;

	if (start == 0 && end == 0)
		return -EINVAL;

	if (start >= 0 && start <= end && end <= 100) {
		req[INPUT_DATA_OFFSET] = 1;
		req[INPUT_DATA_OFFSET + 1] = start;
		ret = gaokun_ec_write(ec, req);
		if (ret)
			return -EIO;

		req[INPUT_DATA_OFFSET] = 2;
		req[INPUT_DATA_OFFSET + 1] = end;
		ret = gaokun_ec_write(ec, req);
	} else {
		return -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(gaokun_ec_wmi_set_threshold);

/* Smart charge param */
int gaokun_ec_wmi_get_smart_charge_param(struct gaokun_ec *ec, u8 *value)
{
	/* GBAC */
	return gaokun_ec_read_byte(ec, (u8 []){0x02, 0xE6, 0}, value);
}
EXPORT_SYMBOL_GPL(gaokun_ec_wmi_get_smart_charge_param);

int gaokun_ec_wmi_set_smart_charge_param(struct gaokun_ec *ec, u8 value)
{
	/* SBAC */
	if (value < 0 || value > 2)
		return -EINVAL;

	return gaokun_ec_write(ec, (u8 []){0x02, 0xE5, 1, value});
}
EXPORT_SYMBOL_GPL(gaokun_ec_wmi_set_smart_charge_param);

/* Smart charge */
int gaokun_ec_wmi_get_smart_charge(struct gaokun_ec *ec,
				   u8 data[GAOKUN_SMART_CHARGE_DATA_SIZE])
{
	/* GBCM */
	u8 req[REQ_HDR_SIZE] = {0x02, 0xE4, 0};
	u8 resp[RESP_HDR_SIZE + 4];
	int ret;

	ret = gaokun_ec_read(ec, req, sizeof(resp), resp);
	if (ret)
		return -EIO;

	data[0] = resp[DATA_OFFSET];
	data[1] = resp[DATA_OFFSET + 1];
	data[2] = resp[DATA_OFFSET + 2];
	data[3] = resp[DATA_OFFSET + 3];

	return 0;
}
EXPORT_SYMBOL_GPL(gaokun_ec_wmi_get_smart_charge);

int gaokun_ec_wmi_set_smart_charge(struct gaokun_ec *ec,
				   u8 data[GAOKUN_SMART_CHARGE_DATA_SIZE])
{
	/* SBCM */
	u8 req[REQ_HDR_SIZE + GAOKUN_SMART_CHARGE_DATA_SIZE] = {0x02, 0XE3, 4,};

	if (!(data[2] >= 0 && data[2] <= data[3] && data[3] <= 100))
		return -EINVAL;

	memcpy(req + INPUT_DATA_OFFSET, data, GAOKUN_SMART_CHARGE_DATA_SIZE);

	return gaokun_ec_write(ec, req);
}
EXPORT_SYMBOL_GPL(gaokun_ec_wmi_set_smart_charge);

/* Fn lock */
int gaokun_ec_wmi_get_fn_lock(struct gaokun_ec *ec, u8 *on)
{
	/* GFRS */
	int ret;
	u8 val;
	u8 req[REQ_HDR_SIZE] = {0x02, 0x6B, 0};

	ret = gaokun_ec_read_byte(ec, req, &val);
	if (val == 0x55) {
		*on = 0;
	}else if (val == 0x5A) {
		*on = 1;
	}else {
		return -EIO;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(gaokun_ec_wmi_get_fn_lock);

int gaokun_ec_wmi_set_fn_lock(struct gaokun_ec *ec, u8 on)
{
	/* SFRS */
	u8 req[REQ_HDR_SIZE + 1] = {0x02, 0x6C, 1,};

	if (on == 0) {
		req[INPUT_DATA_OFFSET] = 0x55;
	}else if (on == 1) {
		req[INPUT_DATA_OFFSET] = 0x5A;
	}else {
		return -EINVAL;
	}

	return gaokun_ec_write(ec, req);
}
EXPORT_SYMBOL_GPL(gaokun_ec_wmi_set_fn_lock);

/* Thermal Zone */
/* Range from 0 to 0x2C, partial valid */
static const u8 temp_reg[] = {0x05, 0x07, 0x08, 0x0E, 0x0F, 0x12, 0x15, 0x1E,
			      0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26,
			      0x27, 0x28, 0x29, 0x2A};

int gaokun_ec_wmi_get_temp(struct gaokun_ec *ec, s16 temp[GAOKUN_TZ_REG_NUM])
{
	/* GTMP */
	u8 req[REQ_HDR_SIZE] = {0x02, 0x61, 1,};
	u8 resp[RESP_HDR_SIZE + sizeof(s16)];
	int ret, i = 0;

	while (i < GAOKUN_TZ_REG_NUM) {
		req[INPUT_DATA_OFFSET] = temp_reg[i];
		ret = gaokun_ec_read(ec, req, sizeof(resp), resp);
		if(ret)
			return -EIO;
		temp[i++] = *(s16 *)(resp + DATA_OFFSET);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(gaokun_ec_wmi_get_temp);

/* -------------------------------------------------------------------------- */
/* API For UCSI */

int gaokun_ec_ucsi_read(struct gaokun_ec *ec,
			u8 resp[GAOKUN_UCSI_READ_SIZE])
{
	u8 req[REQ_HDR_SIZE] = {0x3, 0xD5, 0};
	u8 _resp[RESP_HDR_SIZE + GAOKUN_UCSI_READ_SIZE];
	int ret;

	ret = gaokun_ec_read(ec, req, sizeof(_resp), _resp);
	if (ret)
		return ret;

	memcpy(resp, _resp + DATA_OFFSET, GAOKUN_UCSI_READ_SIZE);
	return 0;
}
EXPORT_SYMBOL_GPL(gaokun_ec_ucsi_read);

int gaokun_ec_ucsi_write(struct gaokun_ec *ec,
			 const u8 req[GAOKUN_UCSI_WRITE_SIZE])
{
	u8 _req[REQ_HDR_SIZE + GAOKUN_UCSI_WRITE_SIZE];

	_req[0] = 0x03;
	_req[1] = 0xD4;
	_req[INPUT_SIZE_OFFSET] = GAOKUN_UCSI_WRITE_SIZE;
	memcpy(_req + INPUT_DATA_OFFSET, req, GAOKUN_UCSI_WRITE_SIZE);

	return gaokun_ec_write(ec, _req);
}
EXPORT_SYMBOL_GPL(gaokun_ec_ucsi_write);

int gaokun_ec_ucsi_get_reg(struct gaokun_ec *ec, u8 *ureg)
{
	u8 req[REQ_HDR_SIZE] = {0x03, 0xD3, 0};
	u8 _resp[RESP_HDR_SIZE + UCSI_REG_SIZE];
	int ret;

	ret = gaokun_ec_read(ec, req, sizeof(_resp), _resp);
	if (ret)
		return ret;

	memcpy(ureg, _resp + DATA_OFFSET, UCSI_REG_SIZE);

	return 0;
}
EXPORT_SYMBOL_GPL(gaokun_ec_ucsi_get_reg);

int gaokun_ec_ucsi_pan_ack(struct gaokun_ec *ec, int port_id)
{
	u8 req[REQ_HDR_SIZE + 1] = {0x03, 0xD2, 1, 0};

	if (port_id >= 0)
		req[INPUT_DATA_OFFSET] = 1 << port_id;

	return gaokun_ec_write(ec, req);
}
EXPORT_SYMBOL_GPL(gaokun_ec_ucsi_pan_ack);

/* -------------------------------------------------------------------------- */
/* Modern Standby */

static int gaokun_ec_suspend(struct device *dev)
{
	struct gaokun_ec *ec = dev_get_drvdata(dev);
	u8 req[REQ_HDR_SIZE + 1] = {0x02, 0xB2, 1, 0xDB};
	int ret;

	if (ec->suspended)
		return 0;

	ret = gaokun_ec_write(ec, req);

	if (ret)
		return ret;

	ec->suspended = true;

	return 0;
}

static int gaokun_ec_resume(struct device *dev)
{
	struct gaokun_ec *ec = dev_get_drvdata(dev);
	u8 req[REQ_HDR_SIZE + 1] = {0x02, 0xB2, 1, 0xEB};
	int ret;
	int i;

	if (!ec->suspended)
		return 0;

	for (i = 0; i < 3; i++) {
		ret = gaokun_ec_write(ec, req);
		if (ret == 0)
			break;

		msleep(100);
	};

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
	/* Allow aux devices to access parent's DT nodes directly */
	device_set_of_node_from_dev(&adev->dev, parent);

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

/* -------------------------------------------------------------------------- */
/* EC */

static irqreturn_t gaokun_ec_irq_handler(int irq, void *data)
{
	struct gaokun_ec *ec = data;
	u8 req[REQ_HDR_SIZE] = {EC_EVENT, EC_QUERY, 0};
	u8 status, id;
	int ret;

	ret = gaokun_ec_read_byte(ec, req, &id);
	if (ret)
		return IRQ_HANDLED;

	switch (id) {
	case 0x0: /* No event */
		break;

	case EC_EVENT_LID:
		gaokun_ec_psy_read_byte(ec, EC_LID_STATE, &status);
		status = EC_LID_OPEN & status;
		input_report_switch(ec->idev, SW_LID, !status);
		input_sync(ec->idev);
		break;

	default:
		blocking_notifier_call_chain(&ec->notifier_list, id, ec);
	}

	return IRQ_HANDLED;
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

	ret = gaokun_aux_init(dev, "psy", ec);
	if (ret)
		return ret;

	ret = gaokun_aux_init(dev, "wmi", ec);
	if (ret)
		return ret;

	ret = gaokun_aux_init(dev, "ucsi", ec);
	if (ret)
		return ret;

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
	/* use gaokun3 for now, it may be reused to gaokun2 */
	{ .compatible = "huawei,gaokun3-ec", },
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

MODULE_DESCRIPTION("HUAWEI Matebook E Go EC driver");
MODULE_AUTHOR("Pengyu Luo <mitltlatltl@gmail.com>");
MODULE_LICENSE("GPL");
