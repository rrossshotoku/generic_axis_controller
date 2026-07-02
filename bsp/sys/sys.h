/*
 * bsp/sys — small system-level primitives. Currently just the soft reboot.
 *
 * Soft reboot triggers an NVIC system-reset, which jumps to the Cortex-M
 * reset handler exactly as if the chip was just powered on. All
 * peripherals re-init, .data is re-copied from flash, .bss is zeroed.
 * Used by the web /api/reboot endpoint to re-apply changes that need a
 * fresh W6100 init (most importantly: a new IP address).
 */

#ifndef BSP_SYS_H
#define BSP_SYS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Soft-reboot the CMC. Never returns. */
__attribute__((noreturn))
void sys_reboot(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYS_H */
