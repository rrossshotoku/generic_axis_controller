/*
 * bsp/sys — see header.
 */

#include "sys.h"

#include "stm32g4xx_hal.h"

void sys_reboot(void)
{
    /* Flush any pending memory accesses before reset. */
    __DSB();
    NVIC_SystemReset();
    /* NVIC_SystemReset() is noreturn but the compiler doesn't always
     * know that. Spin in case the reset is delayed. */
    for (;;) { __NOP(); }
}
