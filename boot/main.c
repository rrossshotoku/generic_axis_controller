/*
 * Bootloader entry point.
 *
 * The CPU boots here at 0x08000000. Depending on the persistent boot flag
 * we either jump straight to the app at 0x08008000 or stay resident and
 * serve firmware update over UDP (segmented SDO on port 5000).
 *
 * Flow:
 *   1. Standard startup + SystemInit — clock config identical to the app.
 *   2. Init HAL + minimum peripherals (SPI to W6100, GPIO, SysTick).
 *   3. Read BOOT persist flag (0x0807D800). CLEAR -> jump to app.
 *   4. STAY -> init W6100, listen on UDP 5000, serve OD dispatch until
 *      COMMIT command arrives (or WDG resets us).
 *   5. On COMMIT -> validate app CRC, jump to app.
 *
 * Design ref: Documentation/dual_bootloader_design.md §5.
 */

#include "boot_flag.h"
#include "boot_od.h"

#include "stm32g4xx_hal.h"

#include <stdint.h>

/* Cube-generated peripheral initialisers we reuse from the app tree. */
extern void SystemClock_Config(void);
extern void MX_GPIO_Init(void);
extern void MX_SPI2_Init(void);       /* W6100 SPI — must match app's config */

#define APP_FLASH_BASE   0x08008000u
#define APP_STACK_ADDR   (*(volatile uint32_t *)(APP_FLASH_BASE + 0u))
#define APP_RESET_VECTOR (*(volatile uint32_t *)(APP_FLASH_BASE + 4u))

/* Exposed so boot_od.c can call it after PROG_COMMIT — see boot_od.h. */
void boot_jump_to_app(void);

/* Sanity: an initial stack pointer must live in RAM (0x2000_0000 range
 * on G474). Otherwise the app image is missing/corrupt and we must stay
 * in bootloader mode to give the operator a chance to flash it. */
static bool app_image_looks_valid(void)
{
    uint32_t sp = APP_STACK_ADDR;
    return (sp >= 0x20000000u) && (sp <= 0x20020000u);   /* 128 KB SRAM */
}

/* Jump to app. Disables interrupts, deinits peripherals, resets stack,
 * hands control to the app's Reset_Handler. Does not return.
 *
 * Called from two places:
 *   - main() at boot when the flag is CLEAR.
 *   - boot_od.c on PROG_COMMIT (bypasses reset — see design ref in
 *     boot_od.c). Deliberately does NOT clear the flag. The flag stays
 *     STAY until the app clears it after BOOT_META_HEALTHY_MS of no-fault
 *     runtime (app/boot_meta.c). This preserves the brick-proof property:
 *     if the freshly-programmed app crashes before that 5 s window, the
 *     flag is still STAY and the next reboot re-enters bootloader. */
void boot_jump_to_app(void)
{
    /* Disable and clear all peripheral interrupts so the app's HAL init
     * starts from a clean slate. */
    __disable_irq();
    HAL_DeInit();
    for (int i = 0; i < 8; ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }

    /* Point VTOR at the app's vector table BEFORE re-enabling interrupts
     * so any that fire during app init route to the app's handlers. The
     * app's own SystemInit will do this again, but doing it here removes
     * a window where an interrupt would land on the bootloader vectors. */
    SCB->VTOR = APP_FLASH_BASE;
    __DSB();
    __ISB();

    uint32_t app_sp    = APP_STACK_ADDR;
    uint32_t app_entry = APP_RESET_VECTOR;

    /* Set MSP to the app's initial stack pointer, then branch.
     *
     * Re-enable PRIMASK before the branch. The CPU boots with interrupts
     * enabled (PRIMASK=0), and the STM32 HAL assumes that state on entry
     * to HAL_Init — HAL_InitTick enables the SysTick IRQ source but never
     * touches PRIMASK. Leaving PRIMASK=1 from our earlier __disable_irq
     * blocks SysTick globally, so HAL_GetTick never advances and any
     * HAL_Delay in app boot hangs forever. Confirmed empirically on
     * first-boot: CPU spun in HAL_Delay at PC=0x08009A72 with PRIMASK=1. */
    __enable_irq();
    __set_MSP(app_sp);
    ((void (*)(void))app_entry)();
    /* not reached */
    for (;;) { __NOP(); }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* Read the boot flag BEFORE bringing up the network — cheap and
     * unambiguous, so a normal-boot power cycle wastes no time on W6100
     * initialisation. */
    if (!boot_flag_is_stay()) {
        if (app_image_looks_valid()) {
            boot_jump_to_app();
        }
        /* App image looks bad — fall through to bootloader mode as the
         * only way to recover. The PC tool will find us on port 5000. */
    }

    /* Bootloader mode. Bring up SPI + W6100 + OD dispatcher. boot_od_init
     * handles net_init internally (uses hard-coded defaults for MVP —
     * see boot_od.c). */
    MX_SPI2_Init();
    boot_od_init();

    while (1) {
        boot_od_tick();
        HAL_Delay(1);
    }
}
