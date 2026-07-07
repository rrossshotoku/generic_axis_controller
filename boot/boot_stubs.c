/*
 * boot_stubs — provides symbols that Cube-generated code references but
 * the bootloader defines itself (or as no-ops).
 *
 *   SystemClock_Config — clock init. Verbatim copy of Core/Src/main.c's
 *                        version so bootloader + app boot the same PLL
 *                        (170 MHz sysclk).
 *   Error_Handler      — infinite loop on unrecoverable HAL error. Same
 *                        behaviour as the app's.
 *   Interrupt handlers — minimal set (SysTick + defaults). Cube's
 *                        stm32g4xx_it.c is deliberately NOT compiled into
 *                        the bootloader because it references app-only
 *                        peripheral handles (hi2c2, hdma_*, htim*, ...).
 */

#include "stm32g4xx_hal.h"

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType       = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState             = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue  = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState         = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource        = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM             = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN             = 85;
    RCC_OscInitStruct.PLL.PLLP             = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ             = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR             = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { /* halt for debugger */ }
}

/* --- Minimal interrupt handlers ------------------------------------------
 * The bootloader needs SysTick (drives HAL_Delay, HAL time base). The rest
 * fall through to the weak defaults in Core/Startup/startup_stm32g474retx.s.
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* Empty exception handlers — override the startup file's weak defaults
 * with slightly nicer "halt cleanly" behaviour if any of these fire, so a
 * debugger attach can see us stuck here rather than in the linker's dead
 * loop. */
void NMI_Handler        (void) { while (1) {} }
void HardFault_Handler  (void) { while (1) {} }
void MemManage_Handler  (void) { while (1) {} }
void BusFault_Handler   (void) { while (1) {} }
void UsageFault_Handler (void) { while (1) {} }
void SVC_Handler        (void) { /* no RTOS */ }
void DebugMon_Handler   (void) { /* no debug monitor */ }
void PendSV_Handler     (void) { /* no RTOS */ }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
    Error_Handler();
}
#endif
