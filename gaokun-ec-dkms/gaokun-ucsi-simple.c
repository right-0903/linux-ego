// SPDX-License-Identifier: GPL-2.0-only
/*
 * gaokun-ucsi-simple - A simple UCSI driver for HUAWEI Matebook E Go (sc8280xp)
 *
 * reference: drivers/usb/typec/ucsi/ucsi_yoga_c630.c
 *            drivers/usb/typec/ucsi/ucsi_glink.c
 *            drivers/soc/qcom/pmic_glink_altmode.c
 *
 * Copyright (C) 2024 nuvole <mitltlatltl@gmail.com>
 */

#include <linux/auxiliary_bus.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/container_of.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/workqueue_types.h>

#include <linux/usb/pd_vdo.h>
#include <drm/bridge/aux-bridge.h>

// location: driver/usb/typec/ucsi/gaokun-ucsi.c
#include "ucsi.h" /* from kernel */
#include "ec.h"


#define EC_EVENT_UCSI	0x21
#define EC_EVENT_USB	0x22

#define GAOKUN_UCSI_CCI_SIZE	4
#define GAOKUN_UCSI_DATA_SIZE	16
#define GAOKUN_UCSI_READ_SIZE	(GAOKUN_UCSI_CCI_SIZE + GAOKUN_UCSI_DATA_SIZE)
#define GAOKUN_UCSI_WRITE_SIZE	0x18

#define SC8280XP_HPD_STATE_MASK		BIT(4)

enum gaokun_ucsi_mux {
	USBC_MUX_NONE,
	USBC_MUX_USB_2L,
	USBC_MUX_DP_4L,
	USBC_MUX_USB_DP,
};

struct gaokun_ucsi_port {
	spinlock_t lock;
	struct completion usb_ack;
	struct work_struct altmode_work;
	struct gaokun_ucsi *ucsi;
	struct auxiliary_device *bridge;
	int idx;
	u16 svid;
	u8 hpd_state;
};

struct gaokun_ucsi {
	struct gaokun_ec *ec;
	struct ucsi *ucsi;
	struct gaokun_ucsi_port *ports;
	struct device *dev;
	struct notifier_block nb;
	unsigned int port_num;
	u16 updt;
	u16 version;
};

/* -------------------------------------------------------------------------- */
/* For UCSI */

static int gaokun_ec_ucsi_read(struct gaokun_ec *ec,
			       u8 resp[GAOKUN_UCSI_READ_SIZE])
{
	u8 _resp[GAOKUN_UCSI_READ_SIZE + 2];
	int ret;

	ret = gaokun_ec_request(ec, (u8 []){0x3, 0xD5, 0},
				sizeof(_resp), _resp);
	if (ret)
		return ret;

	memcpy(resp, _resp + 2, GAOKUN_UCSI_READ_SIZE);
	return 0;
}

static int gaokun_ec_ucsi_write(struct gaokun_ec *ec,
				const u8 req[GAOKUN_UCSI_WRITE_SIZE])
{
	u8 _req[GAOKUN_UCSI_WRITE_SIZE + 3];
	u8 resp[3];

	_req[0] = 0x03;
	_req[1] = 0xD4;
	_req[2] = GAOKUN_UCSI_WRITE_SIZE;
	memcpy(_req + 3, req, GAOKUN_UCSI_WRITE_SIZE);

	return gaokun_ec_request(ec, _req, sizeof(resp), resp);
}

static int gaokun_ucsi_read_version(struct ucsi *ucsi, u16 *version)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);

	*version = uec->version;

	return 0;
}

static int gaokun_ucsi_read_cci(struct ucsi *ucsi, u32 *cci)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);
	u8 buf[GAOKUN_UCSI_READ_SIZE];
	int ret;

	ret = gaokun_ec_ucsi_read(uec->ec, buf);
	if (ret)
		return ret;

	memcpy(cci, buf, sizeof(*cci));

	return 0;
}

static int gaokun_ucsi_read_message_in(struct ucsi *ucsi,
				       void *val, size_t val_len)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);
	u8 buf[GAOKUN_UCSI_READ_SIZE];
	int ret;

	ret = gaokun_ec_ucsi_read(uec->ec, buf);
	if (ret)
		return ret;

	memcpy(val, buf + GAOKUN_UCSI_CCI_SIZE,
	       min(val_len, GAOKUN_UCSI_DATA_SIZE));

	return 0;
}

static int gaokun_ucsi_async_control(struct ucsi *ucsi, u64 command)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);
	u8 buf[GAOKUN_UCSI_WRITE_SIZE] = {};

	memcpy(buf, &command, sizeof(command));
	return gaokun_ec_ucsi_write(uec->ec, buf);
}

const struct ucsi_operations gaokun_ucsi_ops = {
	.read_version = gaokun_ucsi_read_version,
	.read_cci = gaokun_ucsi_read_cci,
	.read_message_in = gaokun_ucsi_read_message_in,
	.sync_control = ucsi_sync_control_common,
	.async_control = gaokun_ucsi_async_control,
};

/* -------------------------------------------------------------------------- */
/* For Altmode */
static void gaokun_ucsi_port_refresh(struct gaokun_ucsi_port *port,
				     const u8 *buf)
{
	unsigned long flags;
	u8 mux;
	int index = (port->idx) * 2;

	spin_lock_irqsave(&port->lock, flags);
	mux = (buf[index] & 0xC) >> 2;

	if (mux == USBC_MUX_NONE)
		port->svid = 0;
	else if (mux == USBC_MUX_USB_2L)
		port->svid = USB_SID_PD;
	else if (mux == USBC_MUX_DP_4L || mux == USBC_MUX_USB_DP)
		port->svid = USB_SID_DISPLAYPORT;

	port->hpd_state = FIELD_GET(SC8280XP_HPD_STATE_MASK, buf[index + 1]);

	spin_unlock_irqrestore(&port->lock, flags);
}

static int gaokun_ucsi_refresh(struct gaokun_ucsi *uec)
{
	u8 resp[9];
	int ret, updt;

	ret = gaokun_ec_request(uec->ec, (u8 []){0x03, 0xD3, 0},
				sizeof(resp), resp);
	if (ret)
		return -EIO;

	pr_info("%s: USB event triggered, data: %*ph\n",
		__func__, (int)sizeof(resp), resp);

	uec->port_num = resp[2];
	updt = resp[3];
	uec->updt = updt;

	gaokun_ucsi_port_refresh(&uec->ports[updt - 1], resp + 4);

	return updt;
}

static inline int gaokun_ucsi_port_write(struct gaokun_ucsi_port *port)
{
	u8 resp[4];
	return gaokun_ec_request(port->ucsi->ec,
				 (u8 []){0x03, 0xD2, 1, port?
					 1 << port->idx: 0},
				 sizeof(resp), resp);
}

static void pmic_glink_ucsi_handle_altmode(struct gaokun_ucsi_port *port)
{
	struct gaokun_ucsi *uec = port->ucsi;
	unsigned long flags;
	int idx = port->idx;

	if (!completion_done(&port->usb_ack))
		complete(&port->usb_ack); /* indicate EC sent the USB EVENT */

	if (idx >= uec->ucsi->cap.num_connectors) {
		dev_warn(uec->ucsi->dev, "altmode port out of range: %d\n", idx);
		return;
	}

	spin_lock_irqsave(&port->lock, flags);
	if (port->svid == USB_SID_DISPLAYPORT && port->bridge)
		drm_aux_hpd_bridge_notify(&port->bridge->dev,
					  port->hpd_state ?
					  connector_status_connected :
					  connector_status_disconnected);
	spin_unlock_irqrestore(&port->lock, flags);

	gaokun_ucsi_port_write(port);
}

static void pmic_glink_ucsi_altmode_notify_ind(struct gaokun_ucsi *uec)
{
	int updt;

	updt = gaokun_ucsi_refresh(uec);
	if (updt == 0)
		gaokun_ucsi_port_write(NULL);
	else if (updt == 1 || updt == 2)
		pmic_glink_ucsi_handle_altmode(&uec->ports[updt - 1]);
}

static void gaokun_handle_no_usb_event(struct gaokun_ucsi *uec, int idx)
{
	struct gaokun_ucsi_port *port;
	if (!idx)
		return;

	port = &uec->ports[idx - 1];
	if (!wait_for_completion_timeout(&port->usb_ack, 2 * HZ)) {
		dev_warn(uec->dev, "No USB EVENT, enable DP manually");
		pmic_glink_ucsi_altmode_notify_ind(uec); /* Manual if timeout */
	}
}

static int gaokun_ucsi_notify(struct notifier_block *nb,
			      unsigned long action, void *data)
{
	struct gaokun_ucsi *uec = container_of(nb, struct gaokun_ucsi, nb);
	u32 cci;

	switch (action) {
	case EC_EVENT_USB:
		pmic_glink_ucsi_altmode_notify_ind(uec);
		return NOTIFY_OK;

	case EC_EVENT_UCSI:
		uec->ucsi->ops->read_cci(uec->ucsi, &cci);
		pr_info("%s: UCSI event triggered, cci is %*ph\n",
			__func__, 4, (u8 *)&cci);
		ucsi_notify_common(uec->ucsi, cci);
		gaokun_handle_no_usb_event(uec, UCSI_CCI_CONNECTOR(cci));
		return NOTIFY_OK;

	default:
		return NOTIFY_DONE;
	}
}

static int gaokun_ucsi_ports_init(struct gaokun_ucsi *uec)
{
	u8 resp[9];
	u32 port;
	int i, ret, port_num;
	struct device *dev = uec->dev;
	struct fwnode_handle *fwnode;

	ret = gaokun_ec_request(uec->ec, (u8 []){0x03, 0xD3, 0},
				sizeof(resp), resp);
	if (ret)
		return -EIO;

	port_num = resp[2];
	uec->port_num = port_num;
	uec->ports = devm_kzalloc(dev, port_num * sizeof(*(uec->ports)),
				  GFP_KERNEL);
	if (!uec->ports)
		return -ENOMEM;

	for (i = 0; i < port_num; ++i) {
		uec->ports[i].idx = i;
		uec->ports[i].ucsi = uec;
		spin_lock_init(&uec->ports[i].lock);
		init_completion(&uec->ports[i].usb_ack);
	}

	device_for_each_child_node(dev, fwnode) {
		ret = fwnode_property_read_u32(fwnode, "reg", &port);
		if (ret < 0) {
			dev_err(dev, "missing reg property of %pOFn\n", fwnode);
			fwnode_handle_put(fwnode);
			return ret;
		}

		if (port >= port_num) {
			dev_warn(dev, "invalid connector number %d, ignoring\n", port);
			continue;
		}
		uec->ports[port].bridge = devm_drm_dp_hpd_bridge_alloc(dev, to_of_node(fwnode));
		if (IS_ERR(uec->ports[port].bridge)) {
			fwnode_handle_put(fwnode);
			return PTR_ERR(uec->ports[port].bridge);
		}
	}

	for (i = 0; i < port_num; i++) {
		if (!uec->ports[i].bridge)
			continue;

		ret = devm_drm_dp_hpd_bridge_add(dev, uec->ports[i].bridge);
		if (ret)
			return ret;
 	}

	return 0;
}

static int gaokun_ucsi_probe(struct auxiliary_device *adev,
			     const struct auxiliary_device_id *id)
{
	struct gaokun_ec *ec = adev->dev.platform_data;
	struct device *dev = &adev->dev;
	struct gaokun_ucsi *uec;
	int ret;

	uec = devm_kzalloc(dev, sizeof(*uec), GFP_KERNEL);
	if (!uec)
		return -ENOMEM;

	uec->ec = ec;
	uec->dev = dev;
	uec->nb.notifier_call = gaokun_ucsi_notify;

	ret = gaokun_ucsi_ports_init(uec);
	if (ret)
		return ret;

	uec->ucsi = ucsi_create(dev, &gaokun_ucsi_ops);
	if (IS_ERR(uec->ucsi))
		return PTR_ERR(uec->ucsi);

	uec->ucsi->quirks = UCSI_NO_PARTNER_PDOS | UCSI_DELAY_DEVICE_PDOS;

	ucsi_set_drvdata(uec->ucsi, uec);

	uec->version = 0x0100; // from ACPI and verified the register on Windows

	auxiliary_set_drvdata(adev, uec);

	ssleep(3); /* EC can't handle UCSI properly in the early stage */

	ret = ucsi_register(uec->ucsi);
	if (ret)
		return ret;

	return gaokun_ec_register_notify(ec, &uec->nb);
}

static void gaokun_ucsi_remove(struct auxiliary_device *adev)
{
	struct gaokun_ucsi *uec = auxiliary_get_drvdata(adev);

	gaokun_ec_unregister_notify(uec->ec, &uec->nb);
	ucsi_unregister(uec->ucsi);
	ucsi_destroy(uec->ucsi);
}

static const struct auxiliary_device_id gaokun_ucsi_id_table[] = {
	{ .name = GAOKUN_MOD_NAME "." GAOKUN_DEV_UCSI, },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, gaokun_ucsi_id_table);

static struct auxiliary_driver gaokun_ucsi_driver = {
	.name = GAOKUN_DEV_UCSI,
	.id_table = gaokun_ucsi_id_table,
	.probe = gaokun_ucsi_probe,
	.remove = gaokun_ucsi_remove,
};

module_auxiliary_driver(gaokun_ucsi_driver);

MODULE_DESCRIPTION("HUAWEI Matebook E Go UCSI driver");
MODULE_LICENSE("GPL");
