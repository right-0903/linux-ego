// SPDX-License-Identifier: GPL-2.0-only
/* Huawei Matebook E Go (sc8280xp) Embedded Controller
 *
 * Copyright (C) 2024 Pengyu Luo <mitltlatltl@gmail.com>
 *
 */

#ifndef __HUAWEI_GAOKUN_EC_H__
#define __HUAWEI_GAOKUN_EC_H__

#define GAOKUN_UCSI_CCI_SIZE	4
#define GAOKUN_UCSI_DATA_SIZE	16
#define GAOKUN_UCSI_READ_SIZE	(GAOKUN_UCSI_CCI_SIZE + GAOKUN_UCSI_DATA_SIZE)
#define GAOKUN_UCSI_WRITE_SIZE	0x18

#define GAOKUN_TZ_REG_NUM	20
#define GAOKUN_SMART_CHARGE_DATA_SIZE	4 /* mode, delay, start, end */

/* -------------------------------------------------------------------------- */

struct gaokun_ec;
struct notifier_block;

#define GAOKUN_MOD_NAME			"huawei_gaokun_ec"
#define GAOKUN_DEV_PSY			"psy"
#define GAOKUN_DEV_WMI			"wmi"
#define GAOKUN_DEV_UCSI			"ucsi"

/* -------------------------------------------------------------------------- */
/* Common API */

int gaokun_ec_register_notify(struct gaokun_ec *ec,
			      struct notifier_block *nb);
void gaokun_ec_unregister_notify(struct gaokun_ec *ec,
				 struct notifier_block *nb);

int gaokun_ec_read(struct gaokun_ec *ec, const u8 *req,
		   size_t resp_len, u8 *resp);
int gaokun_ec_write(struct gaokun_ec *ec, u8 *req);
int gaokun_ec_read_byte(struct gaokun_ec *ec, u8 *req, u8 *byte);

/* -------------------------------------------------------------------------- */
/* API For PSY */

int gaokun_ec_psy_multi_read(struct gaokun_ec *ec, u8 reg,
			     size_t resp_len, u8 *resp);

static inline int gaokun_ec_psy_read_byte(struct gaokun_ec *ec,
					  u8 reg, u8 *byte)
{
	return gaokun_ec_psy_multi_read(ec, reg, 1, byte);
}

static inline int gaokun_ec_psy_read_word(struct gaokun_ec *ec,
					  u8 reg, u16 *word)
{
	return gaokun_ec_psy_multi_read(ec, reg, 2, (u8 *)word);
}

/* -------------------------------------------------------------------------- */
/* API For WMI */

int gaokun_ec_wmi_get_threshold(struct gaokun_ec *ec, u8 *value, int ind);
int gaokun_ec_wmi_set_threshold(struct gaokun_ec *ec, u8 start, u8 end);

int gaokun_ec_wmi_get_smart_charge_param(struct gaokun_ec *ec, u8 *value);
int gaokun_ec_wmi_set_smart_charge_param(struct gaokun_ec *ec, u8 value);

int gaokun_ec_wmi_get_smart_charge(struct gaokun_ec *ec,
				   u8 data[GAOKUN_SMART_CHARGE_DATA_SIZE]);
int gaokun_ec_wmi_set_smart_charge(struct gaokun_ec *ec,
				   u8 data[GAOKUN_SMART_CHARGE_DATA_SIZE]);

int gaokun_ec_wmi_get_fn_lock(struct gaokun_ec *ec, u8 *on);
int gaokun_ec_wmi_set_fn_lock(struct gaokun_ec *ec, u8 on);

int gaokun_ec_wmi_get_temp(struct gaokun_ec *ec, s16 temp[GAOKUN_TZ_REG_NUM]);

/* -------------------------------------------------------------------------- */
/* API For UCSI */

int gaokun_ec_ucsi_read(struct gaokun_ec *ec, u8 resp[GAOKUN_UCSI_READ_SIZE]);
int gaokun_ec_ucsi_write(struct gaokun_ec *ec,
			 const u8 req[GAOKUN_UCSI_WRITE_SIZE]);

int gaokun_ec_ucsi_get_reg(struct gaokun_ec *ec, u8 *ureg);
int gaokun_ec_ucsi_pan_ack(struct gaokun_ec *ec, int port_id);


#endif /* __HUAWEI_GAOKUN_EC_H__ */
