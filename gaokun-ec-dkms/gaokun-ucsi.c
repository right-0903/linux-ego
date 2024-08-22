// SPDX-License-Identifier: GPL-2.0-only

#include <linux/auxiliary_bus.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/container_of.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/string.h>

// location: driver/usb/typec/ucsi/gaokun-ucsi.c
#include "ucsi.h" /* from 6.11-rc2 */
#include "ec.h"

#define EC_EVENT_USB 	0x22
#define EC_EVENT_UCSI 	0x21
#define YOGA_C630_UCSI_CCI_SIZE     4
#define YOGA_C630_UCSI_DATA_SIZE    16
#define YOGA_C630_UCSI_READ_SIZE	(YOGA_C630_UCSI_CCI_SIZE + YOGA_C630_UCSI_DATA_SIZE)
#define YOGA_C630_UCSI_WRITE_SIZE   0x18

/* The implementation of functions from lenovo-yoga-c630.c
 * keep origianl function name for the convenience, final release will be replaced.
 */
static int yoga_c630_ec_ucsi_read(struct gaokun_ec *ec,
			   u8 resp[YOGA_C630_UCSI_READ_SIZE])
{
    u8 *obuf;
    obuf = ec_command_data(ec, 0x3, 0xD5, 0, NULL, YOGA_C630_UCSI_READ_SIZE + 2);
    memcpy(resp, obuf + 2, YOGA_C630_UCSI_READ_SIZE);

    return *obuf;
}

static int yoga_c630_ec_ucsi_write(struct gaokun_ec *ec,
			    const u8 req[YOGA_C630_UCSI_WRITE_SIZE])
{
    u8 *obuf;
    obuf = ec_command_data(ec, 0x3, 0xD4, YOGA_C630_UCSI_WRITE_SIZE, req, 3);
    pr_info_ratelimited("%s: ec_ucsi_write_ctl_8b, data: %*ph\n", __func__, 3, obuf);
    return *obuf;
}

struct gaokun_ucsi {
	struct gaokun_ec *ec;
	struct ucsi *ucsi;
	struct notifier_block nb;
	u16 version;
};

static int yoga_c630_ucsi_read_version(struct ucsi *ucsi, u16 *version)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);

	*version = uec->version;

	return 0;
}

static int yoga_c630_ucsi_read_cci(struct ucsi *ucsi, u32 *cci)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);
	u8 buf[YOGA_C630_UCSI_READ_SIZE];
	int ret;

	ret = yoga_c630_ec_ucsi_read(uec->ec, buf);
	if (ret)
		return ret;

	memcpy(cci, buf, sizeof(*cci));

	return 0;
}

static int yoga_c630_ucsi_read_message_in(struct ucsi *ucsi,
					  void *val, size_t val_len)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);
	u8 buf[YOGA_C630_UCSI_READ_SIZE];
	int ret;

	ret = yoga_c630_ec_ucsi_read(uec->ec, buf);
	if (ret)
		return ret;

	memcpy(val, buf + YOGA_C630_UCSI_CCI_SIZE,
	       min(val_len, YOGA_C630_UCSI_DATA_SIZE));

	return 0;
}

static int yoga_c630_ucsi_async_control(struct ucsi *ucsi, u64 command)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);
	u8 buf[YOGA_C630_UCSI_WRITE_SIZE] = {};
	memcpy(buf, (u8*)&command, sizeof(command));
	return yoga_c630_ec_ucsi_write(uec->ec, buf);
}

static void pmic_glink_ucsi_update_connector(struct ucsi_connector *con)
{
	// struct gaokun_ucsi *ucsi = ucsi_get_drvdata(con->ucsi);

	con->typec_cap.orientation_aware = true;
}

static void pmic_glink_ucsi_connector_status(struct ucsi_connector *con)
{
	// struct gaokun_ucsi *ucsi = ucsi_get_drvdata(con->ucsi);
	int orientation, index;

	if (con->num > PMIC_GLINK_MAX_PORTS){
		return;
	}

	index = ((con->num - 1) + 2) * 2;

	orientation = usb_data[index] & EC_CON_REVERSE;
	if (orientation >= 0) {
		typec_set_orientation(con->port,
				      orientation ?
				      TYPEC_ORIENTATION_REVERSE :
				      TYPEC_ORIENTATION_NORMAL);
	}
}


const struct ucsi_operations yoga_c630_ucsi_ops = {
	.read_version = yoga_c630_ucsi_read_version,
	.read_cci = yoga_c630_ucsi_read_cci,
	.read_message_in = yoga_c630_ucsi_read_message_in,
	.sync_control = ucsi_sync_control_common,
	.async_control = yoga_c630_ucsi_async_control,
	.update_connector = pmic_glink_ucsi_update_connector,
	.connector_status = pmic_glink_ucsi_connector_status,
};

static int yoga_c630_ucsi_notify(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	u32 cci;
	struct gaokun_ucsi *uec = container_of(nb, struct gaokun_ucsi, nb);

	switch (action) {
	case EC_EVENT_USB:
		ucsi_connector_change(uec->ucsi, (int)usb_data[3]);
		return NOTIFY_OK;

	case EC_EVENT_UCSI:
		uec->ucsi->ops->read_cci(uec->ucsi, &cci);
		ucsi_notify_common(uec->ucsi, cci);
		// BIT(1) -> LEFT, BIT(2) -> RIGHT
		pr_info_ratelimited("%s: UCSI event triggered, cci is %*ph\n", __func__, 4, (u8 *)&cci);
		return NOTIFY_OK;

	default:
		return NOTIFY_DONE;
	}
}

static int yoga_c630_ucsi_probe(struct auxiliary_device *adev,
				const struct auxiliary_device_id *id)
{
	struct gaokun_ec *ec = adev->dev.platform_data;
	struct gaokun_ucsi *uec;
	int ret;

	uec = devm_kzalloc(&adev->dev, sizeof(*uec), GFP_KERNEL);
	if (!uec)
		return -ENOMEM;

	uec->ec = ec;
	uec->nb.notifier_call = yoga_c630_ucsi_notify;

	uec->ucsi = ucsi_create(&adev->dev, &yoga_c630_ucsi_ops);
	if (IS_ERR(uec->ucsi))
		return PTR_ERR(uec->ucsi);

	ucsi_set_drvdata(uec->ucsi, uec);

	// uec->version = yoga_c630_ec_ucsi_get_version(uec->ec);
	uec->version = 0x0100;

	auxiliary_set_drvdata(adev, uec);

	ret = gaokun_ec_register_notify(ec, &uec->nb);
	if (ret)
		return ret;

	return ucsi_register(uec->ucsi);
}

static void yoga_c630_ucsi_remove(struct auxiliary_device *adev)
{
	struct gaokun_ucsi *uec = auxiliary_get_drvdata(adev);

	gaokun_ec_unregister_notify(uec->ec, &uec->nb);
	ucsi_unregister(uec->ucsi);
	ucsi_destroy(uec->ucsi);
}

static const struct auxiliary_device_id yoga_c630_ucsi_id_table[] = {
	{ .name = GAOKUN_MOD_NAME "." GAOKUN_DEV_UCSI, },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, yoga_c630_ucsi_id_table);

static struct auxiliary_driver yoga_c630_ucsi_driver = {
	.name = GAOKUN_DEV_UCSI,
	.id_table = yoga_c630_ucsi_id_table,
	.probe = yoga_c630_ucsi_probe,
	.remove = yoga_c630_ucsi_remove,
};

module_auxiliary_driver(yoga_c630_ucsi_driver);

MODULE_DESCRIPTION("HUAWEI Matebook E Go UCSI driver");
MODULE_LICENSE("GPL");
