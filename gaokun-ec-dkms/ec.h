// SPDX-License-Identifier: GPL-2.0-only

#ifndef __GAOKUN_EC_H__
#define __GAOKUN_EC_H__

#include <linux/delay.h>
#include <linux/i2c.h>

#define DATA_OFFSET		0x2

#define EC_EVENT		0x06	// SCI_EVT ?

/* Also can be found in ACPI specification 12.3 */
#define EC_READ			0X80
#define EC_QUERY_EVENT	0X84

#define EC_EVENT_LID	0x81

#define EC_LID_STATE	0x80
#define EC_LID_CLOSE	BIT(1)

/* Buffer length for ECCD method, length have already -2 for each, because
 * we don't need seats for the Status field and the Reserved field.
 */
#define EC_INPUT_BUFFER_LENGTH 0xFD
#define EC_OUTPUT_BUFFER_LENGTH 0xFF


struct gaokun_ec;
struct notifier_block;

#define GAOKUN_MOD_NAME			"gaokun_ec" /* cannnot use gaokun-ec */
#define GAOKUN_DEV_PSY			"psy"
#define GAOKUN_DEV_WMI			"wmi"
#define GAOKUN_DEV_UCSI			"ucsi"

int gaokun_ec_register_notify(struct gaokun_ec *ec, struct notifier_block *nb);
void gaokun_ec_unregister_notify(struct gaokun_ec *ec, struct notifier_block *nb);


u8 *ec_command_data(struct gaokun_ec *ec, u8 mcmd, u8 scmd, u8 ilen, const u8 *buf, u8 olen);
int gaokun_ec_request(struct gaokun_ec *ec, const u8 *req, size_t resp_len, u8 *resp);


int gaokun_ec_write(struct gaokun_ec *ec, u8 *req);
int gaokun_ec_multi_read(struct gaokun_ec *ec, u8 reg, size_t resp_len, u8 *resp);

static inline int gaokun_ec_read_word(struct gaokun_ec *ec, u8 reg, u16 *word)
{
	return gaokun_ec_multi_read(ec, reg, 2, (u8 *)word);
}

static inline int gaokun_ec_read_byte(struct gaokun_ec *ec, u8 reg, u8 *byte)
{
	return gaokun_ec_multi_read(ec, reg, 1, byte);
}


#endif /* __GAOKUN_EC_H__ */
