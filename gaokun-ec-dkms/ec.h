// SPDX-License-Identifier: GPL-2.0-only

#ifndef __GAOKUN_EC_H__
#define __GAOKUN_EC_H__

#include <linux/delay.h>
#include <linux/i2c.h>


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

#define EC_CON_REVERSE			BIT(0)
#define PMIC_GLINK_MAX_PORTS	2
#define EC_EVENT_USB 	0x22

extern u8 usb_data[9];


struct gaokun_ec;
struct notifier_block;

#define GAOKUN_MOD_NAME			"gaokun_ec" /* cannnot use gaokun-ec */
#define GAOKUN_DEV_USBC			"altmode"
#define GAOKUN_DEV_UCSI			"ucsi"
#define GAOKUN_DEV_PSY			"psy"
#define GAOKUN_DEV_WMI			"wmi"

int gaokun_ec_register_notify(struct gaokun_ec *ec, struct notifier_block *nb);
void gaokun_ec_unregister_notify(struct gaokun_ec *ec, struct notifier_block *nb);


u8 *ec_command_data(struct gaokun_ec *ec, u8 mcmd, u8 scmd, u8 ilen, const u8 *buf, u8 olen);


static inline int ec_read_word_data(struct gaokun_ec *ec, u8 reg_l, u8 reg_h, u16 *data)
{
	u8 *obuf;
	int offset;
	offset = 2; // STAT BLEN DAT0 DAT1 ...

	obuf = ec_command_data(ec, 0x2, EC_READ, 1, (u8 []){reg_l}, 4);

/*	// do not handle for now
	if(obuf[0]){
		return -
	}
*/
	// little endian
	*data = obuf[offset];

	usleep_range(2500, 3000); // Sleep (0x02)

	obuf = ec_command_data(ec, 0x2, EC_READ, 1, (u8 []){reg_h}, 4);
	*data |= obuf[offset] << 8;

	return 0;
}



#endif /* __GAOKUN_EC_H__ */
