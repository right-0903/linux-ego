// SPDX-License-Identifier: GPL-2.0-only
/*
 * ucsi-huawei-gaokun - A UCSI driver for HUAWEI Matebook E Go
 *
 * reference: drivers/usb/typec/ucsi/ucsi_yoga_c630.c
 *            drivers/usb/typec/ucsi/ucsi_glink.c
 *            drivers/soc/qcom/pmic_glink_altmode.c
 *
 * Copyright (C) 2024 Pengyu Luo <mitltlatltl@gmail.com>
 */

#include <drm/bridge/aux-bridge.h>
#include <linux/auxiliary_bus.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/container_of.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
// #include <linux/platform_data/huawei-gaokun-ec.h>
#include <linux/string.h>
#include <linux/usb/pd_vdo.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/workqueue_types.h>

#include "huawei-gaokun-ec.h"
#include "ucsi.h"

#define EC_EVENT_UCSI	0x21
#define EC_EVENT_USB	0x22

#define GAOKUN_CCX_MASK		GENMASK(1, 0)
#define GAOKUN_MUX_MASK		GENMASK(3, 2)

#define GAOKUN_DPAM_MASK	GENMASK(3, 0)
#define GAOKUN_HPD_STATE_MASK	BIT(4)
#define GAOKUN_HPD_IRQ_MASK	BIT(5)

#define GET_IDX(updt) (ffs(updt) - 1)

#define CCX_TO_ORI(ccx) (++ccx % 3) /* convert ccx to enum typec_orientation */

/* Configuration Channel Extension */
enum gaokun_ucsi_ccx {
	USBC_CCX_NORMAL,
	USBC_CCX_REVERSE,
	USBC_CCX_NONE,
};

enum gaokun_ucsi_mux {
	USBC_MUX_NONE,
	USBC_MUX_USB_2L,
	USBC_MUX_DP_4L,
	USBC_MUX_USB_DP,
};
/* based on pmic_glink_altmode_pin_assignment */
enum gaokun_ucsi_dpam_pan {	/* DP Alt Mode Pin Assignments */
	USBC_DPAM_PAN_NONE,
	USBC_DPAM_PAN_A,	/* Not supported after USB Type-C Standard v1.0b */
	USBC_DPAM_PAN_B,	/* Not supported after USB Type-C Standard v1.0b */
	USBC_DPAM_PAN_C,	/* USBC_DPAM_PAN_C_REVERSE - 6 */
	USBC_DPAM_PAN_D,
	USBC_DPAM_PAN_E,
	USBC_DPAM_PAN_F,	/* Not supported after USB Type-C Standard v1.0b */
	USBC_DPAM_PAN_A_REVERSE,/* Not supported after USB Type-C Standard v1.0b */
	USBC_DPAM_PAN_B_REVERSE,/* Not supported after USB Type-C Standard v1.0b */
	USBC_DPAM_PAN_C_REVERSE,
	USBC_DPAM_PAN_D_REVERSE,
	USBC_DPAM_PAN_E_REVERSE,
	USBC_DPAM_PAN_F_REVERSE,/* Not supported after USB Type-C Standard v1.0b */
};

struct gaokun_ucsi_reg {
	u8 port_num;
	u8 port_updt;
	u8 port_data[4];
	u8 checksum;
	u8 reserved;
} __packed;

struct gaokun_ucsi_port {
	struct completion usb_ack;
	spinlock_t lock;

	struct gaokun_ucsi *ucsi;
	struct auxiliary_device *bridge;

	int idx;
	enum gaokun_ucsi_ccx ccx;
	enum gaokun_ucsi_mux mux;
	u8 mode;
	u16 svid;
	u8 hpd_state;
	u8 hpd_irq;
};

struct gaokun_ucsi {
	struct gaokun_ec *ec;
	struct ucsi *ucsi;
	struct gaokun_ucsi_port *ports;
	struct device *dev;
	struct delayed_work work;
	struct notifier_block nb;
	u16 version;
	u8 port_num;
};

/* -------------------------------------------------------------------------- */
/* For UCSI */

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

static void gaokun_ucsi_update_connector(struct ucsi_connector *con)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(con->ucsi);

	if (con->num > uec->port_num)
		return;

	con->typec_cap.orientation_aware = true;
}

static void gaokun_set_orientation(struct ucsi_connector *con,
				   struct gaokun_ucsi_port *port)
{
	enum gaokun_ucsi_ccx ccx;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	ccx = port->ccx;
	spin_unlock_irqrestore(&port->lock, flags);

	typec_set_orientation(con->port, CCX_TO_ORI(ccx));
}

static void gaokun_ucsi_connector_status(struct ucsi_connector *con)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(con->ucsi);
	int idx;

	idx = con->num - 1;
	if (con->num > uec->port_num) {
		dev_warn(uec->ucsi->dev, "set orientation out of range: con%d\n", idx);
		return;
	}

	gaokun_set_orientation(con, &uec->ports[idx]);
}

const struct ucsi_operations gaokun_ucsi_ops = {
	.read_version = gaokun_ucsi_read_version,
	.read_cci = gaokun_ucsi_read_cci,
	.read_message_in = gaokun_ucsi_read_message_in,
	.sync_control = ucsi_sync_control_common,
	.async_control = gaokun_ucsi_async_control,
	.update_connector = gaokun_ucsi_update_connector,
	.connector_status = gaokun_ucsi_connector_status,
};

/* -------------------------------------------------------------------------- */
/* For Altmode */

static void gaokun_ucsi_port_update(struct gaokun_ucsi_port *port,
				    const u8 *port_data)
{
	struct ucsi *ucsi = port->ucsi->ucsi;
	int offset = port->idx * 2; /* every port has 2 Bytes data */
	unsigned long flags;
	u8 dcc, ddi;

	dcc = port_data[offset];
	ddi = port_data[offset + 1];

	spin_lock_irqsave(&port->lock, flags);

	port->ccx = FIELD_GET(GAOKUN_CCX_MASK, dcc);
	port->mux = FIELD_GET(GAOKUN_MUX_MASK, dcc);
	port->mode = FIELD_GET(GAOKUN_DPAM_MASK, ddi);
	port->hpd_state = FIELD_GET(GAOKUN_HPD_STATE_MASK, ddi);
	port->hpd_irq = FIELD_GET(GAOKUN_HPD_IRQ_MASK, ddi);

	/* Mode and SVID are unused; keeping them to make things clearer */
	switch (port->mode) {
	case USBC_DPAM_PAN_C:
	case USBC_DPAM_PAN_C_REVERSE:
		port->mode = DP_PIN_ASSIGN_C; /* correct it for usb later */
		break;
	case USBC_DPAM_PAN_D:
	case USBC_DPAM_PAN_D_REVERSE:
		port->mode = DP_PIN_ASSIGN_D;
		break;
	case USBC_DPAM_PAN_E:
	case USBC_DPAM_PAN_E_REVERSE:
		port->mode = DP_PIN_ASSIGN_E;
		break;
	case USBC_DPAM_PAN_NONE:
		port->mode = TYPEC_STATE_SAFE;
		break;
	default:
		dev_warn(ucsi->dev, "unknow mode %d\n", port->mode);
		break;
	}

	switch (port->mux) {
	case USBC_MUX_NONE:
		port->svid = 0;
		break;
	case USBC_MUX_USB_2L:
		port->svid = USB_SID_PD;
		port->mode = TYPEC_STATE_USB; /* same as PAN_C, correct it */
		break;
	case USBC_MUX_DP_4L:
	case USBC_MUX_USB_DP:
		port->svid = USB_SID_DISPLAYPORT;
		break;
	default:
		dev_warn(ucsi->dev, "unknow mux state %d\n", port->mux);
		break;
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

static int gaokun_ucsi_refresh(struct gaokun_ucsi *uec)
{
	struct gaokun_ucsi_reg ureg;
	int ret, idx;

	ret = gaokun_ec_ucsi_get_reg(uec->ec, (u8 *)&ureg);
	if (ret)
		return GAOKUN_UCSI_NO_PORT_UPDATE;

	uec->port_num = ureg.port_num;
	idx = GET_IDX(ureg.port_updt);

	if (idx < 0 || idx >= ureg.port_num)
		return GAOKUN_UCSI_NO_PORT_UPDATE;

	gaokun_ucsi_port_update(&uec->ports[idx], ureg.port_data);
	return idx;
}

static void gaokun_ucsi_handle_altmode(struct gaokun_ucsi_port *port)
{
	struct gaokun_ucsi *uec = port->ucsi;
	int idx = port->idx;

	if (idx >= uec->ucsi->cap.num_connectors || !uec->ucsi->connector) {
		dev_warn(uec->ucsi->dev, "altmode port out of range: %d\n", idx);
		return;
	}

	/* UCSI callback .connector_status() have set orientation */
	if (port->bridge)
		drm_aux_hpd_bridge_notify(&port->bridge->dev,
					  port->hpd_state ?
					  connector_status_connected :
					  connector_status_disconnected);

	gaokun_ec_ucsi_pan_ack(uec->ec, port->idx);
}

static void gaokun_ucsi_altmode_notify_ind(struct gaokun_ucsi *uec)
{
	int idx;

	idx = gaokun_ucsi_refresh(uec);
	if (idx == GAOKUN_UCSI_NO_PORT_UPDATE)
		gaokun_ec_ucsi_pan_ack(uec->ec, idx); /* ack directly if no update */
	else
		gaokun_ucsi_handle_altmode(&uec->ports[idx]);
}

/*
 * USB event is necessary for enabling altmode, the event should follow
 * UCSI event, if not after timeout(this notify may be disabled somehow),
 * then force to enable altmode.
 */
static void gaokun_ucsi_handle_no_usb_event(struct gaokun_ucsi *uec, int idx)
{
	struct gaokun_ucsi_port *port;

	port = &uec->ports[idx];
	if (!wait_for_completion_timeout(&port->usb_ack, 2 * HZ)) {
		dev_warn(uec->dev, "No USB EVENT, triggered by UCSI EVENT");
		gaokun_ucsi_altmode_notify_ind(uec);
	}
}

static int gaokun_ucsi_notify(struct notifier_block *nb,
			      unsigned long action, void *data)
{
	u32 cci;
	struct gaokun_ucsi *uec = container_of(nb, struct gaokun_ucsi, nb);

	switch (action) {
	case EC_EVENT_USB:
		gaokun_ucsi_altmode_notify_ind(uec);
		return NOTIFY_OK;

	case EC_EVENT_UCSI:
		uec->ucsi->ops->read_cci(uec->ucsi, &cci);
		ucsi_notify_common(uec->ucsi, cci);
		if (UCSI_CCI_CONNECTOR(cci))
			gaokun_ucsi_handle_no_usb_event(uec, UCSI_CCI_CONNECTOR(cci) - 1);

		return NOTIFY_OK;

	default:
		return NOTIFY_DONE;
	}
}

static int gaokun_ucsi_get_port_num(struct gaokun_ucsi *uec)
{
	struct gaokun_ucsi_reg ureg;
	int ret;

	ret = gaokun_ec_ucsi_get_reg(uec->ec, (u8 *)&ureg);

	return ret ? 0 : ureg.port_num;
}

static int gaokun_ucsi_ports_init(struct gaokun_ucsi *uec)
{
	u32 port;
	int i, ret, port_num;
	struct device *dev = uec->dev;
	struct gaokun_ucsi_port *ucsi_port;
	struct fwnode_handle *fwnode;

	port_num = gaokun_ucsi_get_port_num(uec);
	uec->port_num = port_num;

	uec->ports = devm_kzalloc(dev, port_num * sizeof(*(uec->ports)),
				  GFP_KERNEL);
	if (!uec->ports)
		return -ENOMEM;

	for (i = 0; i < port_num; ++i) {
		ucsi_port = &uec->ports[i];
		ucsi_port->ccx = USBC_CCX_NONE;
		ucsi_port->idx = i;
		ucsi_port->ucsi = uec;
		init_completion(&ucsi_port->usb_ack);
		spin_lock_init(&ucsi_port->lock);
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

		ucsi_port = &uec->ports[port];
		ucsi_port->bridge = devm_drm_dp_hpd_bridge_alloc(dev, to_of_node(fwnode));
		if (IS_ERR(ucsi_port->bridge)) {
			fwnode_handle_put(fwnode);
			return PTR_ERR(ucsi_port->bridge);
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

static void gaokun_ucsi_register_worker(struct work_struct *work)
{
	struct gaokun_ucsi *uec;
	struct ucsi *ucsi;
	int ret;

	uec = container_of(work, struct gaokun_ucsi, work.work);
	ucsi = uec->ucsi;
	/* This may be a problem specific to sc8280xp-based machines */
	ucsi->quirks = UCSI_NO_PARTNER_PDOS | UCSI_DELAY_DEVICE_PDOS;

	ret = gaokun_ec_register_notify(uec->ec, &uec->nb);
	if (ret) {
		dev_err_probe(ucsi->dev, ret, "notifier register failed\n");
		return;
	}

	ret = ucsi_register(ucsi);
	if (ret)
		dev_err_probe(ucsi->dev, ret, "ucsi register failed\n");
}

static inline int gaokun_ucsi_register(struct gaokun_ucsi *uec)
{
	/* EC can't handle UCSI properly in the early stage */
	schedule_delayed_work(&uec->work, 3 * HZ);

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
	uec->version = 0x0100;
	uec->nb.notifier_call = gaokun_ucsi_notify;

	INIT_DELAYED_WORK(&uec->work, gaokun_ucsi_register_worker);

	ret = gaokun_ucsi_ports_init(uec);
	if (ret)
		return ret;

	uec->ucsi = ucsi_create(dev, &gaokun_ucsi_ops);
	if (IS_ERR(uec->ucsi))
		return PTR_ERR(uec->ucsi);

	ucsi_set_drvdata(uec->ucsi, uec);
	auxiliary_set_drvdata(adev, uec);

	return gaokun_ucsi_register(uec);
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
