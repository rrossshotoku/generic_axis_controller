/*
 * bsp/net/wizchip_glue.h — internal port layer for the WIZnet ioLibrary.
 *
 * Not a public API. Only bsp/net/net.c should include this. It exists as a
 * separate file to keep the hardware-specific SPI/CS/RESET code out of the
 * socket-API file, and to make the CubeMX pin overrides obvious.
 */

#ifndef BSP_NET_WIZCHIP_GLUE_H
#define BSP_NET_WIZCHIP_GLUE_H

#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 * Pin assignment.
 *
 * The Lightweight_CMC.ioc labels the W6100 control pins as:
 *   SPI2_NSS  (PB12)  — chip select
 *   WIZ_RSTn  (PC8)   — active-low reset
 *   WIZ_INT   (PC9)   — IRQ line (EXTI9, not used yet — polled mode)
 *
 * CubeMX emits SPI2_NSS_GPIO_Port / SPI2_NSS_Pin and
 * WIZ_RSTn_GPIO_Port / WIZ_RSTn_Pin in Core/Inc/main.h. We use those
 * directly so any pinout change in the .ioc propagates here on regen.
 *---------------------------------------------------------------------------*/

#include "main.h"            /* CubeMX-generated pin label macros */

#define WIZCHIP_CS_GPIO_Port   SPI2_NSS_GPIO_Port
#define WIZCHIP_CS_Pin         SPI2_NSS_Pin

#define WIZCHIP_RST_GPIO_Port  WIZ_RSTn_GPIO_Port
#define WIZCHIP_RST_Pin        WIZ_RSTn_Pin

/*----------------------------------------------------------------------------
 * Lifecycle
 *---------------------------------------------------------------------------*/

/* Drive the W6100 RST line through a reset pulse: CS high, RST low 10 ms,
 * RST high, wait 150 ms for the chip to come up. Blocking; called only at
 * net_init time. */
void wizchip_glue_reset_pulse(void);

/* Register the CRIS, CS and SPI callbacks with the ioLibrary. Must be
 * called after wizchip_glue_reset_pulse(). */
void wizchip_glue_register_callbacks(void);

#endif /* BSP_NET_WIZCHIP_GLUE_H */
