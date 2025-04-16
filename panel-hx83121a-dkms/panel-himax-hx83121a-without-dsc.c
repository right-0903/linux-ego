// SPDX-License-Identifier: GPL-2.0-only
/*
 * Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree.
 *
 * Matebook E Go (sc8180x) doesn't enable dsc, but they have the same panel.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

/* command < 0xB0, Display Command Set (DCS)
 * command >= 0xB0, Manufacturer Command Set (MCS)
 * commands can be sent in LP or HS ?
 */

/*
 * 05: short write with 0 parameter, 1 payload(command)
 * e.g. 05 11 / 05 10, exit/enter sleep
 *
 * 06: read request, no parameter, 1 payload(command)
 * 06 0A 9D: DSIStatusSequence, GET_POWER_MODE, 0x9D should be returned as the expected value
 *
 * 39: long write
 *
 *
 * ##################### Brightness ##########################
 * 15 53 24: WRITE_CONTROL_DISPLAY
 *
 * G bit 0 selects gamma curve: 0 = Manual, 1 = Automatic
 * DB bit 1 selects display brightness: 0 = Manual, 1 = Automatic
 * BL bit 2 controls backlight control: 0 = Off, 1 = On
 * DD bit 3 controls display dimming: 0 = Off, 1 = On
 * A bit 4 controls LABC block: 0 = Off, 1 = On
 * BCTRL bit 5 controls brightness block: 0 = Off, 1 = On
 *
 * ##################### CABC ##########################
 * Content Adaptive Backlight Control
 * 00:CABC disable; 01:CABC enable(UI mode)
 * 39 55 01: WRITE_POWER_SAVE
 *
 * 39 29 00: SET_DISPLAY_ON
 *
 * some controllers do not support sending
 * commands in non-LPM mode. Thus the panel driver should send all commands
 * before the DSI host switches from LPM to VIDEO mode.
 */

struct panel_info {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	const struct panel_desc *desc;
	// enum drm_panel_orientation orientation;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *tp_reset_gpio;
	struct backlight_device *backlight;
	struct regulator_bulk_data supplies[3];
};

struct panel_desc {
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned int bpc;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct mipi_dsi_device_info dsi_info;
	int (*init_sequence)(struct panel_info *pinfo);
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, panel);
}

static int gaokun_csot_init_sequence(struct panel_info *pinfo)
{
	struct mipi_dsi_device *dsi = pinfo->dsi[0];
	struct mipi_dsi_multi_context ctx = { .dsi = dsi };
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	if (pinfo->dsi[1])
		pinfo->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb9, 0x83, 0x12, 0x1a, 0x55, 0x00);

	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0x0008); // 39 51 08 00
	if (ret < 0) {
		dev_err(dev, "Failed to set display brightness: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq_multi(&ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb1,
			       0x1c, 0x6b, 0x6b, 0x27, 0xe7, 0x00, 0x1b, 0x11,
			       0x21, 0x21, 0x2d, 0x2d, 0x17, 0x33, 0x31, 0x40,
			       0xcd, 0xff, 0x1a, 0x05, 0x15, 0x98, 0x00, 0x88,
			       0x7f, 0xff, 0xff, 0xcf, 0x1a, 0xcc, 0x02, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xd1, 0x37, 0x03, 0x0c, 0xfd);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe2, 0x20);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb2,
			       0x00, 0x6a, 0x40, 0x00, 0x00, 0x14, 0x98, 0x60,
			       0x3c, 0x02, 0x80, 0x21, 0x21, 0x00, 0x00, 0x10,
			       0x27);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x03);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe1, 0x00, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe9, 0xe2);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe7, 0x49);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe9, 0x3f);

	mipi_dsi_dcs_write_seq_multi(&ctx, 0xd3,
			       0x00, 0xc0, 0x08, 0x08, 0x08, 0x04, 0x04, 0x04,
			       0x16, 0x02, 0x07, 0x07, 0x07, 0x31, 0x13, 0x16,
			       0x12, 0x12, 0x03, 0x03, 0x03, 0x32, 0x10, 0x15,
			       0x00, 0x11, 0x32, 0x10, 0x03, 0x00, 0x03, 0x32,
			       0x10, 0x03, 0x00, 0x03, 0x00, 0x00, 0xff, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x02);

	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe2, 0x80, 0x05, 0x1c, 0xbe, 0x09, 0x8d, 0x0f,
			       0x57, 0x03, 0x87, 0x06, 0x10, 0x32, 0x06, 0x15, 0x00,
			       0x00, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00, 0x00, 0x01,
			       0x10, 0x10, 0x16, 0x28, 0x3c, 0x03, 0x23, 0x5d, 0x02,
			       0x02, 0x00, 0x00, 0x48, 0x01, 0xac, 0x0f, 0xab, 0x10,
			       0x00, 0x32, 0x87, 0x00, 0xa1, 0x00, 0x0a, 0xcb, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe7,
			       0x02, 0x00, 0xb2, 0x01, 0x56, 0x07, 0x56, 0x08,
			       0x48, 0x14, 0x00, 0x26);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x02);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe7,
			       0x05, 0x05, 0x01, 0x05, 0x01, 0x05, 0x04, 0x04,
			       0x04, 0x24, 0x00, 0x24, 0x81, 0x02, 0x40, 0x00,
			       0x32, 0x87, 0x03, 0x02, 0x01, 0x00, 0x00, 0x00,
			       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			       0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe9, 0xd0);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb2, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe9, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbf,
			       0xfd, 0x00, 0x80, 0x9c, 0x10, 0x00, 0x81, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe4,
			       0xe1, 0xe1, 0xe1, 0xe1, 0xe1, 0xe1, 0xe1, 0xe1,
			0xc7, 0xb2, 0xa0, 0x90, 0x81, 0x75, 0x69, 0x5f,
			0x55, 0x4c, 0x44, 0x3d, 0x36, 0x2f, 0x2a, 0x24,
			0x1e, 0x19, 0x14, 0x10, 0x09, 0x08, 0x07, 0x54,
			0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x03);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe4,
			       0xaa, 0xd4, 0xff, 0x2a, 0x55, 0x7f, 0xaa, 0xd4,
			0xff, 0xea, 0xff, 0x03);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbd, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe9, 0xc8);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb1, 0x25);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe9, 0x3f);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbe, 0x01, 0x35, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xd9, 0x5f);

	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb9, 0x00, 0x00, 0x00);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}

	msleep(140);

	mipi_dsi_dcs_set_display_on_multi(&ctx);

	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0xff0f); // 39 51 0F FF
	if (ret < 0) {
		dev_err(dev, "Failed to set display brightness: %d\n", ret);
		return ret;
	}

	return ctx.accum_err;
}

static const struct drm_display_mode gaokun_csot_modes[] = {
	{
		.clock = (1600 + 60 + 40 + 40) * (2560 + 154 + 4 + 18) * 60 / 1000,
		.hdisplay = 1600,
		.hsync_start = 1600 + 60,
		.hsync_end = 1600 + 60 + 20,
		.htotal = 1600 + 60 + 20 + 40,
		.vdisplay = 2560,
		.vsync_start = 2560 + 168,
		.vsync_end = 2560 + 168 + 4,
		.vtotal = 2560 + 168 + 4 + 18,
	},
};

static const struct panel_desc gaokun_csot_desc = {
	.modes = gaokun_csot_modes,
	.num_modes = ARRAY_SIZE(gaokun_csot_modes),
	.dsi_info = {
		.type = "CSOT-gaokun",
		.channel = 0, // DSI virtual channel assigned to peripheral
		.node = NULL,
	},
	.width_mm = 166,
	.height_mm = 266,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = gaokun_csot_init_sequence,
};

static void hx83121a_reset(struct panel_info *pinfo)
{
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0); // high
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1); // low
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0); // high

	msleep(110);
}

static int hx83121a_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);
	if (ret < 0) {
		dev_err(&pinfo->dsi[0]->dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	hx83121a_reset(pinfo);

	ret = pinfo->desc->init_sequence(pinfo);
	if (ret < 0) {
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		regulator_bulk_disable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);
		gpiod_set_value_cansleep(pinfo->reset_gpio, 0);
		return ret;
	}
	msleep(120);

	return 0;
}

static int hx83121a_panel_off(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(pinfo->dsi[0]); // 05 10 00
	if (ret < 0)
		dev_err(&pinfo->dsi[0]->dev, "failed to enter sleep mode: %d\n", ret);

	msleep(120);

	return 0;
}

static int hx83121a_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	hx83121a_panel_off(panel);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);

	return 0;
}

static int hx83121a_get_modes(struct drm_panel *panel,
			      struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int i;

	for (i = 0; i < pinfo->desc->num_modes; i++) {
		const struct drm_display_mode *m = &pinfo->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode) {
			dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
			return -ENOMEM;
		}

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = pinfo->desc->width_mm;
	connector->display_info.height_mm = pinfo->desc->height_mm;
	connector->display_info.bpc = pinfo->desc->bpc;

	return pinfo->desc->num_modes;
}

// static enum drm_panel_orientation hx83121a_get_orientation(struct drm_panel *panel)
// {
// 	struct panel_info *pinfo = to_panel_info(panel);
//
// 	return pinfo->orientation;
// }

static const struct drm_panel_funcs hx83121a_panel_funcs = {
	.prepare = hx83121a_prepare,
	.unprepare = hx83121a_unprepare,
	.get_modes = hx83121a_get_modes,
	// .get_orientation = hx83121a_get_orientation,
};

static int hx83121a_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	struct mipi_dsi_multi_context ctx = { .dsi = dsi };
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;
	// mipi_dsi_dcs_write_seq_multi(&ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24); // 39 53 24

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int hx83121a_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops hx83121a_bl_ops = {
	.update_status = hx83121a_bl_update_status,
	.get_brightness = hx83121a_bl_get_brightness,
};

static struct backlight_device *hx83121a_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 0x800,
		.max_brightness = 4095,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &hx83121a_bl_ops, &props);
}

static void hx83121a_remove(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(pinfo->dsi[0]);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI0 host: %d\n", ret);

	ret = mipi_dsi_detach(pinfo->dsi[1]);
	if (ret < 0)
		dev_err(&pinfo->dsi[1]->dev, "failed to detach from DSI1 host: %d\n", ret);
	mipi_dsi_device_unregister(pinfo->dsi[1]);

	drm_panel_remove(&pinfo->panel);
}

static int hx83121a_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct panel_info *pinfo;
	const struct mipi_dsi_device_info *info;
	int i, ret;

	pinfo = devm_kzalloc(dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	pinfo->supplies[0].supply = "vddio";
	pinfo->supplies[1].supply = "vsp";
	pinfo->supplies[2].supply = "vsn";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(pinfo->supplies),
				      pinfo->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(pinfo->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pinfo->reset_gpio), "failed to get display panel reset gpio\n");

	pinfo->desc = of_device_get_match_data(dev);
	if (!pinfo->desc)
		return -ENODEV;

	info = &pinfo->desc->dsi_info;

	dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
	if (!dsi1) {
		dev_err(dev, "cannot get secondary DSI node.\n");
		return -ENODEV;
	} else {
		dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
		of_node_put(dsi1);
		if (!dsi1_host)
			return dev_err_probe(dev, -EPROBE_DEFER, "cannot get secondary DSI host\n");

		pinfo->dsi[1] = mipi_dsi_device_register_full(dsi1_host, info);
		if (IS_ERR(pinfo->dsi[1])) {
			dev_err(dev, "cannot get secondary DSI device\n");
			return PTR_ERR(pinfo->dsi[1]);
		}

		mipi_dsi_set_drvdata(pinfo->dsi[1], pinfo);
	}

	pinfo->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	drm_panel_init(&pinfo->panel, dev, &hx83121a_panel_funcs, DRM_MODE_CONNECTOR_DSI);

	pinfo->panel.prepare_prev_first = true;

	// ret = of_drm_get_panel_orientation(dev->of_node, &pinfo->orientation);
	// if (ret < 0) {
	// 	dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
	// 	return ret;
	// }

	pinfo->panel.backlight = hx83121a_create_backlight(dsi);
	if (IS_ERR(pinfo->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(pinfo->panel.backlight),
					"Failed to create backlight\n");

	drm_panel_add(&pinfo->panel);

	for (i = 0; i < ARRAY_SIZE(pinfo->dsi); i++) {
		pinfo->dsi[i]->lanes = pinfo->desc->lanes;
		pinfo->dsi[i]->format = pinfo->desc->format;
		pinfo->dsi[i]->mode_flags = pinfo->desc->mode_flags;

		ret = devm_mipi_dsi_attach(dev, pinfo->dsi[i]);
		if (ret < 0){
			drm_panel_remove(&pinfo->panel);
			return dev_err_probe(dev, ret, "cannot attach to DSI%d host.\n", i);
		}
	}

	return 0;
}

static const struct of_device_id hx83121a_of_match[] = {
	{
		.compatible = "himax,hx83121a",
		.data = &gaokun_csot_desc,
	},
	{},
};
MODULE_DEVICE_TABLE(of, hx83121a_of_match);

static struct mipi_dsi_driver hx83121a_driver = {
	.probe = hx83121a_probe,
	.remove = hx83121a_remove,
	.driver = {
		.name = "panel-himax-hx83121a",
		.of_match_table = hx83121a_of_match,
	},
};
module_mipi_dsi_driver(hx83121a_driver);

MODULE_DESCRIPTION("DRM driver for Himax HX83121a based MIPI DSI panels");
MODULE_LICENSE("GPL");
