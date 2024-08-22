// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Linaro Ltd
 */
#include <linux/auxiliary_bus.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/soc/qcom/pdr.h>
#include <drm/bridge/aux-bridge.h>
#include <drm/drm_bridge.h>

#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_retimer.h>

#include "ec.h"

struct pmic_glink_altmode;

#define work_to_altmode_port(w) container_of((w), struct pmic_glink_altmode_port, work)

struct pmic_glink_altmode_port {
	struct pmic_glink_altmode *altmode;
	unsigned int index;

	struct typec_switch *typec_switch;
	struct typec_mux *typec_mux;
	struct typec_mux_state state;
	struct typec_retimer *typec_retimer;
	struct typec_retimer_state retimer_state;
	struct typec_altmode dp_alt;

	struct work_struct work;

	bool bridge_configured;
	struct drm_bridge bridge;

	bool connected;

	enum typec_orientation orientation;
	u16 svid;
	u8 dp_data;
	u8 mode;
	u8 hpd_state;
	u8 hpd_irq;
};

struct pmic_glink_altmode {
	struct gaokun_ec *ec;
	struct notifier_block nb;

	struct device *dev;

	struct pmic_glink_altmode_port ports[PMIC_GLINK_MAX_PORTS];
};

/* ========================================================================== */

static void pmic_glink_altmode_enable_dp(struct pmic_glink_altmode *altmode,
					 struct pmic_glink_altmode_port *port,
					 u8 mode, bool hpd_state,
					 bool hpd_irq)
{
	struct typec_displayport_data dp_data = {};
	int ret;

	dp_data.status = DP_STATUS_ENABLED;
	if (hpd_state)
		dp_data.status |= DP_STATUS_HPD_STATE;
	if (hpd_irq)
		dp_data.status |= DP_STATUS_IRQ_HPD;
	dp_data.conf = DP_CONF_SET_PIN_ASSIGN(mode);

	port->state.alt = &port->dp_alt;
	port->state.data = &dp_data;
	port->state.mode = TYPEC_MODAL_STATE(mode);

	ret = typec_mux_set(port->typec_mux, &port->state);
	if (ret)
		dev_err(altmode->dev, "failed to switch mux to DP: %d\n", ret);

	port->retimer_state.alt = &port->dp_alt;
	port->retimer_state.data = &dp_data;
	port->retimer_state.mode = TYPEC_MODAL_STATE(mode);

	ret = typec_retimer_set(port->typec_retimer, &port->retimer_state);
	if (ret)
		dev_err(altmode->dev, "failed to setup retimer to DP: %d\n", ret);
}

static void pmic_glink_altmode_enable_usb(struct pmic_glink_altmode *altmode,
					  struct pmic_glink_altmode_port *port)
{
	int ret;

	port->state.alt = NULL;
	port->state.data = NULL;
	port->state.mode = TYPEC_STATE_USB;

	ret = typec_mux_set(port->typec_mux, &port->state);
	if (ret)
		dev_err(altmode->dev, "failed to switch mux to USB: %d\n", ret);

	port->retimer_state.alt = NULL;
	port->retimer_state.data = NULL;
	port->retimer_state.mode = TYPEC_STATE_USB;

	ret = typec_retimer_set(port->typec_retimer, &port->retimer_state);
	if (ret)
		dev_err(altmode->dev, "failed to setup retimer to USB: %d\n", ret);
}

static void pmic_glink_altmode_safe(struct pmic_glink_altmode *altmode,
				    struct pmic_glink_altmode_port *port)
{
	int ret;

	port->state.alt = NULL;
	port->state.data = NULL;
	port->state.mode = TYPEC_STATE_SAFE;

	ret = typec_mux_set(port->typec_mux, &port->state);
	if (ret)
		dev_err(altmode->dev, "failed to switch mux to safe mode: %d\n", ret);

	port->retimer_state.alt = NULL;
	port->retimer_state.data = NULL;
	port->retimer_state.mode = TYPEC_STATE_SAFE;

	ret = typec_retimer_set(port->typec_retimer, &port->retimer_state);
	if (ret)
		dev_err(altmode->dev, "failed to setup retimer to USB: %d\n", ret);
}

/* ========================================================================== */

static enum typec_orientation pmic_glink_altmode_orientation(unsigned int orientation)
{
	if (orientation == 0)
		return TYPEC_ORIENTATION_NORMAL;
	else if (orientation == 1)
		return TYPEC_ORIENTATION_REVERSE;
	else
		return TYPEC_ORIENTATION_NONE;
}

static int pmic_glink_altmode_sc8280xp_notify(struct notifier_block *nb,
                unsigned long action, void *data)
{
	struct pmic_glink_altmode *altmode = container_of(nb, struct pmic_glink_altmode, nb);
	struct pmic_glink_altmode_port *alt_port = altmode->ports;

	switch (action) {
	case EC_EVENT_USB:
		for (int i = 0; i < PMIC_GLINK_MAX_PORTS; ++i)
			schedule_work(&alt_port[i].work);
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

static void update(struct pmic_glink_altmode_port *alt_port, const u8 *obuf)
{
	struct pmic_glink_altmode *altmode = alt_port->altmode;
	u8 orientation;
	u8 hpd_state;
	u8 mode;
	u8 mux;
	u8 dpot;
	u16 bsid;
	u16 bssd;
	u16 bvid = 0x12d1; // huawei vendor id, little endian, from acpi
	u8 svs[8] = {};

	orientation = obuf[0] & EC_CON_REVERSE;

	hpd_state = obuf[0] & BIT(3);
	mux = obuf[0] & 0xC;
	mux >>= 2; // 0 / 1 / 2 / 3, free / charge or usb(2 lanes) / dp(4 lanes) / dp + usb(3 + 1?)
	mode = obuf[1] & 0xF;
	if(mode >= 7)
		mode -= 6;
	dpot = obuf[1] & 0x30;
	dpot <<= 2;
	mode += dpot;

	switch(mux){
	case 3:
	case 2:
		bsid = 0xFF01;
		svs[0] = mode;
		break;
	default:
		bsid = 0xFF00;
		svs[0] = mode = 0;
		break;
	}

	alt_port->orientation = pmic_glink_altmode_orientation(orientation);
	alt_port->svid = bsid;
	alt_port->mode = mode;
	alt_port->hpd_state = hpd_state;
}

static void mode_set(struct pmic_glink_altmode_port *alt_port)
{
	struct pmic_glink_altmode *altmode = alt_port->altmode;
	typec_switch_set(alt_port->typec_switch, alt_port->orientation);

	if (alt_port->svid == USB_TYPEC_DP_SID && alt_port->mode == 0xff)
		pmic_glink_altmode_safe(altmode, alt_port);
	else if (alt_port->svid == USB_TYPEC_DP_SID)
		pmic_glink_altmode_enable_dp(altmode, alt_port, alt_port->mode,
					     alt_port->hpd_state, alt_port->hpd_irq);
	else
		pmic_glink_altmode_enable_usb(altmode, alt_port);

	drm_bridge_hpd_notify(&alt_port->bridge,
				  alt_port->hpd_state ?
				  connector_status_connected :
				  connector_status_disconnected);
	// req, USBW
	ec_command_data(altmode->ec, 0x03, 0xD2, 1, (u8 []){alt_port->index + 1}, 4);
}

static void aspire_ec_bridge_update_hpd_work(struct work_struct *work)
{
	// UsbcPinAssignmentNotify(UPAN)
	struct pmic_glink_altmode_port *alt_port = work_to_altmode_port(work);
	// struct pmic_glink_altmode *altmode = alt_port->altmode;

	// triggered when status update
	u8 *obuf = usb_data;
	int index = (alt_port->index + 2) * 2; // obuf[4] -> con0, obuf[6] -> con1
	bool flag = (obuf[index] & BIT(3)) == BIT(3);
	if (flag != alt_port->connected) {
		alt_port->connected = flag;
		alt_port->hpd_irq = 1;
		update(alt_port, obuf + index);
		mode_set(alt_port);
		pr_info_ratelimited("%s: STATUS CHANGED, USB EVENT DATA: %*ph\n", __func__, 9, obuf);
	}else {
		alt_port->hpd_irq = 0;
	}
}

static void aspire_ec_bridge_hpd_enable(struct drm_bridge *bridge)
{
	struct pmic_glink_altmode_port *altmode_port = container_of(bridge, struct pmic_glink_altmode_port, bridge);
	schedule_work(&altmode_port->work);
}

static int aspire_ec_bridge_attach(struct drm_bridge *bridge, enum drm_bridge_attach_flags flags)
{
	return flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR ? 0 : -EINVAL;
}

static const struct drm_bridge_funcs aspire_ec_bridge_funcs = {
	.hpd_enable = aspire_ec_bridge_hpd_enable,
	.attach = aspire_ec_bridge_attach,
};

static void pmic_glink_altmode_put_retimer(void *data)
{
	typec_retimer_put(data);
}

static void pmic_glink_altmode_put_mux(void *data)
{
	typec_mux_put(data);
}

static void pmic_glink_altmode_put_switch(void *data)
{
	typec_switch_put(data);
}

static int pmic_glink_altmode_probe(struct auxiliary_device *adev,
				    const struct auxiliary_device_id *id)
{
	struct gaokun_ec *ec = adev->dev.platform_data;
	struct pmic_glink_altmode_port *alt_port;
	struct pmic_glink_altmode *altmode;
	struct fwnode_handle *fwnode;
	struct device *dev = &adev->dev;
	u32 port;
	int ret;

	altmode = devm_kzalloc(&adev->dev, sizeof(*altmode), GFP_KERNEL);
	if (!altmode)
		return -ENOMEM;

	altmode->dev = dev;
	altmode->ec = ec;

	altmode->nb.notifier_call = pmic_glink_altmode_sc8280xp_notify;
	auxiliary_set_drvdata(adev, altmode);

	device_for_each_child_node(dev->parent, fwnode) {
		ret = fwnode_property_read_u32(fwnode, "reg", &port);
		if (ret < 0) {
			dev_err(dev, "missing reg property of %pOFn\n", fwnode);
			fwnode_handle_put(fwnode);
			return ret;
		}

		if (port >= ARRAY_SIZE(altmode->ports)) {
			dev_warn(dev, "invalid connector number, ignoring\n");
			continue;
		}

		if (altmode->ports[port].altmode) {
			dev_err(dev, "multiple connector definition for port %u\n", port);
			fwnode_handle_put(fwnode);
			return -EINVAL;
		}

		alt_port = &altmode->ports[port];
		alt_port->altmode = altmode;
		alt_port->index = port;
		INIT_WORK(&alt_port->work, aspire_ec_bridge_update_hpd_work);

	// drm bridge method
		alt_port->connected = false;
		alt_port->bridge_configured = false;
		alt_port->bridge.funcs = &aspire_ec_bridge_funcs;
		alt_port->bridge.of_node = to_of_node(fwnode);
		alt_port->bridge.ops = DRM_BRIDGE_OP_HPD;
		alt_port->bridge.type = DRM_MODE_CONNECTOR_USB;
		ret = devm_drm_bridge_add(altmode->dev, &alt_port->bridge);
		if (ret) {
			fwnode_handle_put(fwnode);
			return dev_err_probe(dev, ret, "Failed to register drm bridge for connector%d\n", port);
		}else {
			alt_port->bridge_configured = true;
			dev_info(dev, "drm bridge for connector%d registered\n", port);
		}

		alt_port->dp_alt.svid = USB_TYPEC_DP_SID;
		alt_port->dp_alt.mode = USB_TYPEC_DP_MODE;
		alt_port->dp_alt.active = 1;

		// FIXME
		alt_port->typec_mux = fwnode_typec_mux_get(fwnode);
		if (IS_ERR(alt_port->typec_mux)) {
			fwnode_handle_put(fwnode);
			return dev_err_probe(dev, PTR_ERR(alt_port->typec_mux),
					     "failed to acquire mode-switch for port: %d\n",
					     port);
		}

		ret = devm_add_action_or_reset(dev, pmic_glink_altmode_put_mux,
					       alt_port->typec_mux);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}

		alt_port->typec_retimer = fwnode_typec_retimer_get(fwnode);
		if (IS_ERR(alt_port->typec_retimer)) {
			fwnode_handle_put(fwnode);
			return dev_err_probe(dev, PTR_ERR(alt_port->typec_retimer),
					     "failed to acquire retimer-switch for port: %d\n",
					     port);
		}

		ret = devm_add_action_or_reset(dev, pmic_glink_altmode_put_retimer,
					       alt_port->typec_retimer);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}

		alt_port->typec_switch = fwnode_typec_switch_get(fwnode);
		if (IS_ERR(alt_port->typec_switch)) {
			fwnode_handle_put(fwnode);
			return dev_err_probe(dev, PTR_ERR(alt_port->typec_switch),
					     "failed to acquire orientation-switch for port: %d\n",
					     port);
		}

		ret = devm_add_action_or_reset(dev, pmic_glink_altmode_put_switch,
					       alt_port->typec_switch);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}
	}

	ret = gaokun_ec_register_notify(ec, &altmode->nb);

	return ret;
}

static void pmic_glink_altmode_remove(struct auxiliary_device *adev)
{
	struct pmic_glink_altmode *altmode = auxiliary_get_drvdata(adev);

	gaokun_ec_unregister_notify(altmode->ec, &altmode->nb);
}

static const struct auxiliary_device_id pmic_glink_altmode_id_table[] = {
	{ .name = GAOKUN_MOD_NAME "." GAOKUN_DEV_USBC, },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, pmic_glink_altmode_id_table);

static struct auxiliary_driver pmic_glink_altmode_driver = {
	.name = GAOKUN_DEV_USBC,
	.id_table = pmic_glink_altmode_id_table,
	.probe = pmic_glink_altmode_probe,
	.remove = pmic_glink_altmode_remove,
};

module_auxiliary_driver(pmic_glink_altmode_driver);

MODULE_DESCRIPTION("HUAWEI Matebook E Go Type-C Altmode driver");
MODULE_LICENSE("GPL");
