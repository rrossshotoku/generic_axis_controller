/*
 * boot_od — UDP OD dispatcher for the bootloader. Handles the reduced OD
 * subset (0x1F50/51/56/57) plus the three segmented-SDO message types.
 *
 * Uses the same UDP framing + port (5000) as the app's app/od/od.c so
 * the PC tool talks to bootloader vs app with identical wire format —
 * only the OD range answered differs.
 *
 * Init opens UDP socket 4 and waits. The tick polls for datagrams and
 * dispatches. Both must be called from boot/main.c's main loop.
 */

#ifndef BOOT_OD_H
#define BOOT_OD_H

#include <stdbool.h>

void boot_od_init(void);
void boot_od_tick(void);

#endif
