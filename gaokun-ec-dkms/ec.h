// SPDX-License-Identifier: GPL-2.0

#ifndef __GAOKUN_EC_H__
#define __GAOKUN_EC_H__

#include <linux/delay.h>
#include <drm/drm_bridge.h>
#include <linux/i2c.h>

/* ========================================================================== */

#define EC_BAT_VENDOR 0x01 /* from 0x01 to 0x16, SUNWODA */

/* _L: lower EC reg address, _H: higer EC reg address
 *
 * The following abbv.s are ACPI's invention
 * BCLP: last full charge, abbv. of Battery Capacity Last Point(by GPT) ?
 * DSCP: design capacity
 * DSVO: design voltage
 * SRNM: serial numbers
 * BCCL: battery charging cycle
 *
 * The following abbv.s are defined by me.
 * VOLT: voltage now
 * CPCT: capacity now
 * CRNT: current now
 * BCAP: percentage now
 */
#define EC_BAT_BCAP_L 0x90
#define EC_BAT_BCAP_H 0x91
#define EC_BAT_VOLT_L 0x92
#define EC_BAT_VOLT_H 0x93
#define EC_BAT_CPCT_L 0x94
#define EC_BAT_CPCT_H 0x95
#define EC_BAT_BCLP_L 0x96
#define EC_BAT_BCLP_H 0x97
#define EC_BAT_CRNT_L 0x9A
#define EC_BAT_CRNT_H 0x9B
#define EC_BAT_DSCP_L 0xA2
#define EC_BAT_DSCP_H 0xA3
#define EC_BAT_DSVO_L 0xA4
#define EC_BAT_DSVO_H 0xA5
#define EC_BAT_SRNM_L 0xA6
#define EC_BAT_SRNM_H 0xA7
#define EC_BAT_BCCL_L 0xAA
#define EC_BAT_BCCL_H 0xAB

/* possible value: 1: discharge 2: charge, other, maybe 8:full */
#define EC_BAT_STATE		0x82
#define EC_BAT_DISCHARGING	BIT(0)
#define EC_BAT_CHARGING		BIT(1)

/* possible value: 2: detach, 3: attach */
#define EC_ADP_STATE		0x81
#define EC_AC_STATUS 		BIT(0)

#define EC_EVENT_BAT_A0 0xA0
#define EC_EVENT_BAT_A1 0xA1
#define EC_EVENT_BAT_A2 0xA2
#define EC_EVENT_BAT_A3 0xA3
#define EC_EVENT_BAT_B1 0xB1

/* ========================================================================== */


#define MILLI_TO_MICRO	1000

#define EC_EVENT		0x06	// SCI_EVT ?

/* Also can be found in ACPI specification 12.3 */
#define EC_READ			0X80
#define EC_QUERY_EVENT	0X84

#define EC_EVENT_LID	0x81

#define EC_LID_STATE	0x80
#define EC_LID_CLOSE	BIT(1)

#define EC_EVENT_USB 	0x22

/* Buffer length for ECCD method, length have already -2 for each, because
 * we don't need seats for the Status field and the Reserved field.
 */
#define EC_INPUT_BUFFER_LENGTH 0xFD
#define EC_OUTPUT_BUFFER_LENGTH 0xFF


struct gaokun_ec {
	struct i2c_client *client;
    struct mutex lock;
	struct power_supply *bat_psy;
	struct power_supply *adp_psy;
	struct input_dev *idev;
	bool suspended;
};

u8 *ec_command_data(struct gaokun_ec *ec, u8 mcmd, u8 scmd, u8 ilen, u8 *buf, u8 olen);


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
