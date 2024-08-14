// SPDX-License-Identifier: GPL-2.0

#ifndef __GAOKUN_BATTERY_H__
#define __GAOKUN_BATTERY_H__

void battery_event_handler(struct gaokun_ec *ec, int event_id);
int gaokun_battery_setup(struct gaokun_ec *ec);
void gaokun_battery_cleanup(void);

#endif /* __GAOKUN_BATTERY_H__ */
