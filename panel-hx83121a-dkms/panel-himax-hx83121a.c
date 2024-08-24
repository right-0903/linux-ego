// SPDX-License-Identifier: GPL-2.0-only
/*
 * Himax HX83121a DriverIC panels driver
 * Copyright (c) 2024 nuvole <mitltlatltl@gmail.com>
 *
 * based on
 * Novatek NT36523 DriverIC panels driver
 * Copyright (c) 2022, 2023 Jianhua Lu <lujianhua000@gmail.com>
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
#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

// #define XBL /* use initial sequence code from XBL rather than one from the DSDT table */

#define REFRESHRATE 60

#define DSI_NUM_MIN 1

#define mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, cmd, seq...)	\
	do {														\
		mipi_dsi_dcs_write_seq(dsi0, cmd, seq);					\
		mipi_dsi_dcs_write_seq(dsi1, cmd, seq);					\
	} while (0)

struct panel_info {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	const struct panel_desc *desc;
	struct drm_dsc_config dsc;
	// enum drm_panel_orientation orientation;
	struct gpio_desc *reset_gpio;
	struct backlight_device *backlight;
	struct regulator *vddio;
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

	bool is_dual_dsi;
	bool has_dcs_backlight;
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, panel);
}

static int gaokun_csot_init_sequence(struct panel_info *pinfo)
{

	struct mipi_dsi_device *dsi0 = pinfo->dsi[0];
	struct mipi_dsi_device *dsi1 = pinfo->dsi[1];
	int ret, i;

	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xb9, 0x83, 0x12, 0x1a, 0x55, 0x00);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbd, 0x00);

#ifdef XBL
	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		/* from xbl sequence */
		ret = mipi_dsi_dcs_set_display_brightness(pinfo->dsi[i], 0x0008); // 39 51 08 00
		if (ret < 0) {
			dev_err(&pinfo->dsi[i]->dev, "Failed to set display brightness: %d\n", ret);
			return ret;
		}
	}
#endif

	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24); // 39 53 24
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xb1,
			       0x1c, 0x6b, 0x6b, 0x27, 0xe7, 0x00, 0x1b, 0x25,
			       0x21, 0x21, 0x2d, 0x2d, 0x17, 0x33, 0x31, 0x40,
			       0xcd, 0xff, 0x1a, 0x05, 0x15, 0x98, 0x00, 0x88,
			       0x7f, 0xff, 0xff, 0xcf, 0x1a, 0xcc, 0x02, 0x00);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xd1, 0x37, 0x03, 0x0c, 0xfd);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xb2,
			       0x00, 0x6a, 0x40, 0x00, 0x00, 0x14, 0x98, 0x60,
			       0x3c, 0x02, 0x80, 0x21, 0x21, 0x00, 0x00, 0xf0,
			       0x27);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe2, 0x10);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xc0, 0x23, 0x23, 0xcc, 0x22, 0x99, 0xd8);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xb4,
			       0x46, 0x06, 0x0c, 0xbe, 0x0c, 0xbe, 0x09, 0x46,
			       0x0f, 0x57, 0x0f, 0x57, 0x03, 0x4a, 0x00, 0x00,
			       0x04, 0x0c, 0x00, 0x18, 0x01, 0x06, 0x08, 0x00,
			       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			       0x00, 0x00, 0xff, 0x00, 0xff, 0x10, 0x00, 0x02,
			       0x14, 0x14, 0x14, 0x14);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbd, 0x03);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe1, 0x01, 0x3f);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbd, 0x00);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe9, 0xe2);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe7, 0x49);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe9, 0x3f);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xd3,
			       0x00, 0xc0, 0x08, 0x08, 0x08, 0x04, 0x04, 0x04,
			       0x16, 0x02, 0x07, 0x07, 0x07, 0x31, 0x13, 0x19,
			       0x12, 0x12, 0x03, 0x03, 0x03, 0x32, 0x10, 0x18,
			       0x00, 0x11, 0x32, 0x10, 0x03, 0x00, 0x03, 0x32,
			       0x10, 0x03, 0x00, 0x03, 0x00, 0x00, 0xff, 0x00);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe1,
			       0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0a, 0x00,
			       0x03, 0x20, 0x00, 0x14, 0x03, 0x20, 0x03, 0x20,
			       0x02, 0x00, 0x02, 0x91, 0x00, 0x20, 0x02, 0x47,
			       0x00, 0x0b, 0x00, 0x0c, 0x05, 0x0e, 0x03, 0x68,
			       0x18, 0x00, 0x10, 0xe0, 0x03, 0x0c, 0x20, 0x00,
			       0x06, 0x0b, 0x0b, 0x33, 0x0e, 0x1c, 0x2a, 0x38,
			       0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b,
			       0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00, 0x09);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe7,
			       0x17, 0x08, 0x08, 0x2c, 0x46, 0x1e, 0x02, 0x23,
			       0x5d, 0x02, 0xc9, 0x00, 0x00, 0x00, 0x00, 0x12,
			       0x05, 0x02, 0x02, 0x07, 0x10, 0x10, 0x00, 0x1d,
			       0xb9, 0x23, 0xb9, 0x00, 0x33, 0x02, 0x88);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbd, 0x01);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe7,
			       0x02, 0x00, 0xb2, 0x01, 0x56, 0x07, 0x56, 0x08,
			       0x48, 0x14, 0xfd, 0x26);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbd, 0x02);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe7,
			       0x08, 0x08, 0x01, 0x03, 0x01, 0x03, 0x07, 0x02,
			       0x02, 0x47, 0x00, 0x47, 0x81, 0x02, 0x40, 0x00,
			       0x18, 0x4a, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
			       0x00, 0x00, 0x03, 0x02, 0x01, 0x00, 0x00, 0x00,
			       0x00, 0x00);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbd, 0x00);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbf,
			       0xfd, 0x00, 0x80, 0x9c, 0x36, 0x00, 0x81, 0x0c);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xcd,
			       0x81, 0x00, 0x80, 0x77, 0x00, 0x01, 0x00);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbd, 0x01);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe4,
			       0xe1, 0xe1, 0xe1, 0xe1, 0xe1, 0xe1, 0xe1, 0xe1,
			       0xc7, 0xb2, 0xa0, 0x90, 0x81, 0x75, 0x69, 0x5f,
			       0x55, 0x4c, 0x44, 0x3d, 0x36, 0x2f, 0x2a, 0x24,
			       0x1e, 0x19, 0x14, 0x10, 0x09, 0x08, 0x07, 0x54,
			       0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbd, 0x03);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xe4,
			       0xaa, 0xd4, 0xff, 0x2a, 0x55, 0x7f, 0xaa, 0xd4,
			       0xff, 0xea, 0xff, 0x03);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbd, 0x00);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xbe, 0x01, 0x35, 0x00);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xd9, 0x5f);
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, 0xb9, 0x00, 0x00, 0x00);


	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		ret = mipi_dsi_dcs_exit_sleep_mode(pinfo->dsi[i]); // 05 11 00
		if (ret < 0) {
			dev_err(&pinfo->dsi[i]->dev, "Failed to exit sleep mode: %d\n", ret);
			return ret;
		}
	}
	msleep(140); // ff 8c


#ifdef XBL
	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		ret = mipi_dsi_dcs_enter_sleep_mode(pinfo->dsi[i]); // 05 10 00
		if (ret < 0) {
			dev_err(&pinfo->dsi[i]->dev, "Failed to enter sleep mode: %d\n", ret);
			return ret;
		}
	}
	msleep(120); // ff 78

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		ret = mipi_dsi_dcs_exit_sleep_mode(pinfo->dsi[i]); // 05 11 00
		if (ret < 0) {
			dev_err(&pinfo->dsi[i]->dev, "Failed to exit sleep mode: %d\n", ret);
			return ret;
		}
	}
	msleep(150); // ff 96
#endif

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		ret = mipi_dsi_dcs_set_display_on(pinfo->dsi[i]); // 05 29 00
		if (ret < 0) {
			dev_err(&pinfo->dsi[i]->dev, "Failed to set display on: %d\n", ret);
			return ret;
		}
	}

#ifdef XBL
	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		ret = mipi_dsi_dcs_set_display_brightness(pinfo->dsi[i], 0xff04); // 39 51 04 ff
		if (ret < 0) {
			dev_err(&pinfo->dsi[i]->dev, "Failed to set display brightness: %d\n", ret);
			return ret;
		}
	}
#endif

	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, MIPI_DCS_WRITE_POWER_SAVE, 0x01); // 39 55 01
	mipi_dsi_dual_dcs_write_seq(dsi0, dsi1, MIPI_DCS_WRITE_POWER_SAVE, 0x24); // 39 55 24
	msleep(20);


	return 0;
}

static const struct drm_display_mode gaokun_csot_modes[] = {
#ifdef XBL
	{
		.clock = (1600 + 60 + 40 + 60) * (2560 + 2890 + 4 + 18) * 60 / 1000,
		.hdisplay = 1600,
		.hsync_start = 1600 + 60,
		.hsync_end = 1600 + 60 + 40,
		.htotal = 1600 + 60 + 40 + 60,
		.vdisplay = 2560,
		.vsync_start = 2560 + 2890,
		.vsync_end = 2560 + 2890 + 4,
		.vtotal = 2560 + 2890 + 4 + 18,
	},
#else
	{
		.clock = (1600 + 60 + 40 + 40) * (2560 + 154 + 4 + 18) * REFRESHRATE / 1000,
		.hdisplay = 1600,
		.hsync_start = 1600 + 60,
		.hsync_end = 1600 + 60 + 40,
		.htotal = 1600 + 60 + 40 + 40,
		.vdisplay = 2560,
		.vsync_start = 2560 + 154,
		.vsync_end = 2560 + 154 + 4,
		.vtotal = 2560 + 154 + 4 + 18,
	},
#endif
/*	// windows mode timings
	{
        .clock = (1600 + 60 + 40 + 40) * (2560 + 146 + 4 + 26) * REFRESHRATE / 1000,
        .hdisplay = 1600,
        .hsync_start = 1600 + 60,
        .hsync_end = 1600 + 60 + 40,
        .htotal = 1600 + 60 + 40 + 40,
        .vdisplay = 2560,
        .vsync_start = 2560 + 146,
        .vsync_end = 2560 + 146 + 4,
        .vtotal = 2560 + 146 + 4 + 26,
	},
*/
};

static const struct panel_desc gaokun_csot_desc = {
	.modes = gaokun_csot_modes,
	.num_modes = ARRAY_SIZE(gaokun_csot_modes),
	.dsi_info = {
		.type = "CSOT-gaokun",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 166,
	.height_mm = 266,
	.bpc = 8,
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.init_sequence = gaokun_csot_init_sequence,
	.is_dual_dsi = true,
	.has_dcs_backlight = true,
};

static void hx83121a_reset(struct panel_info *pinfo)
{

	gpiod_set_value_cansleep(pinfo->reset_gpio, 1); // low
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0); // high
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 1); // low
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(pinfo->reset_gpio, 0); // high

	msleep(110);

}

static int hx83121a_prepare(struct drm_panel *panel)
{

	struct panel_info *pinfo = to_panel_info(panel);
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = regulator_enable(pinfo->vddio);
	if (ret) {
		dev_err(panel->dev, "failed to enable vddio regulator: %d\n", ret);
		return ret;
	}

	hx83121a_reset(pinfo);

	ret = pinfo->desc->init_sequence(pinfo);
	if (ret < 0) {
		regulator_disable(pinfo->vddio);
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		return ret;
	}
	msleep(120);

	/* disable sending pps packets when MIPI DSC enabled according to XML  */

	drm_dsc_pps_payload_pack(&pps, &pinfo->dsc);

	print_hex_dump(KERN_INFO, "DSC:", DUMP_PREFIX_NONE, 16,
	       1, (void *)&pps, sizeof(pps), false);

	ret = mipi_dsi_picture_parameter_set(pinfo->dsi[0], &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_compression_mode(pinfo->dsi[0], true);
	if (ret < 0) {
		dev_err(&pinfo->dsi[0]->dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}

	msleep(120);


	return 0;
}

static int hx83121a_disable(struct drm_panel *panel)
{

	struct panel_info *pinfo = to_panel_info(panel);
	int i, ret;

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		ret = mipi_dsi_dcs_enter_sleep_mode(pinfo->dsi[i]); // 05 10 00
		if (ret < 0)
			dev_err(&pinfo->dsi[i]->dev, "failed to enter sleep mode: %d\n", ret);
	}

	msleep(120);

	return 0;
}

static int hx83121a_unprepare(struct drm_panel *panel)
{

	struct panel_info *pinfo = to_panel_info(panel);

	gpiod_set_value_cansleep(pinfo->reset_gpio, 1);
	regulator_disable(pinfo->vddio);

	return 0;
}

static void hx83121a_remove(struct mipi_dsi_device *dsi)
{

	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(pinfo->dsi[0]);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI0 host: %d\n", ret);

	if (pinfo->desc->is_dual_dsi) {
		ret = mipi_dsi_detach(pinfo->dsi[1]);
		if (ret < 0)
			dev_err(&pinfo->dsi[1]->dev, "failed to detach from DSI1 host: %d\n", ret);
		mipi_dsi_device_unregister(pinfo->dsi[1]);
	}

	drm_panel_remove(&pinfo->panel);

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
	.disable = hx83121a_disable,
	.prepare = hx83121a_prepare,
	.unprepare = hx83121a_unprepare,
	.get_modes = hx83121a_get_modes,
	// .get_orientation = hx83121a_get_orientation,
};

static int hx83121a_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

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
		.brightness = 512,
		.max_brightness = 4095,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
										  &hx83121a_bl_ops, &props);
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

	pinfo->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(pinfo->vddio))
		return dev_err_probe(dev, PTR_ERR(pinfo->vddio), "failed to get vddio regulator\n");

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pinfo->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pinfo->reset_gpio), "failed to get reset gpio\n");


	pinfo->desc = of_device_get_match_data(dev);
	if (!pinfo->desc)
		return -ENODEV;

	/* If the panel is dual dsi, register DSI1 */
	if (pinfo->desc->is_dual_dsi) {
		info = &pinfo->desc->dsi_info;

		dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
		if (!dsi1) {
			dev_err(dev, "cannot get secondary DSI node.\n");
			return -ENODEV;
		}

		dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
		of_node_put(dsi1);
		if (!dsi1_host)
			return dev_err_probe(dev, -EPROBE_DEFER, "cannot get secondary DSI host\n");

		pinfo->dsi[1] = mipi_dsi_device_register_full(dsi1_host, info);
		if (IS_ERR(pinfo->dsi[1])) {
			dev_err(dev, "cannot get secondary DSI device\n");
			return PTR_ERR(pinfo->dsi[1]);
		}
	}

	pinfo->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		pinfo->dsi[i]->dsc = &pinfo->dsc;
		pinfo->dsi[i]->lanes = pinfo->desc->lanes;
		pinfo->dsi[i]->format = pinfo->desc->format;
		pinfo->dsi[i]->mode_flags = pinfo->desc->mode_flags;
	}

	drm_panel_init(&pinfo->panel, dev, &hx83121a_panel_funcs, DRM_MODE_CONNECTOR_DSI);

	// ret = of_drm_get_panel_orientation(dev->of_node, &pinfo->orientation);
	// if (ret < 0) {
	// 	dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
	// 	return ret;
	// }

	pinfo->panel.prepare_prev_first = true;

	if (pinfo->desc->has_dcs_backlight) {
		pinfo->panel.backlight = hx83121a_create_backlight(dsi);
		if (IS_ERR(pinfo->panel.backlight))
			return dev_err_probe(dev, PTR_ERR(pinfo->panel.backlight),
								 "Failed to create backlight\n");
	} else {
		ret = drm_panel_of_backlight(&pinfo->panel);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to get backlight\n");
	}

	drm_panel_add(&pinfo->panel);
    pr_err("%s: prepare and enable should be done\n", __func__);

	pinfo->dsc.dsc_version_major = 1;
	pinfo->dsc.dsc_version_minor = 1;

	pinfo->dsc.slice_height = 20;
	pinfo->dsc.slice_width = 800;

	pinfo->dsc.slice_count = 1;
	pinfo->dsc.bits_per_component = 8;
	pinfo->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	pinfo->dsc.block_pred_enable = true;

	for (i = 0; i < DSI_NUM_MIN + pinfo->desc->is_dual_dsi; i++) {
		ret = mipi_dsi_attach(pinfo->dsi[i]);
		if (ret < 0)
			return dev_err_probe(dev, ret, "cannot attach to DSI%d host.\n", i);
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

MODULE_AUTHOR("nuvole <mitltlatltl@gmail.com>");
MODULE_DESCRIPTION("DRM driver for Himax HX83121a based MIPI DSI panels");
MODULE_LICENSE("GPL");
