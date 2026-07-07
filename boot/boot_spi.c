/*
 * boot_spi — minimal SPI2 init for the bootloader.
 *
 * Mirrors Core/Src/spi.c's MX_SPI2_Init exactly (same clock config,
 * same GPIO alt-function mapping — the app must produce identical SPI
 * timing when it takes over) but drops the DMA setup: bsp/net's
 * wizchip_glue uses polled HAL_SPI_TransmitReceive, so the bootloader
 * doesn't need DMA channels wired up. Dropping DMA also saves us from
 * having to include stm32g4xx_hal_dma.c + defining hdma_* handles.
 *
 * `hspi2` is the same global the app's spi.c defines — declared here
 * because we don't compile the app's spi.c into the bootloader.
 */

#include "stm32g4xx_hal.h"

SPI_HandleTypeDef hspi2;

void MX_SPI2_Init(void)
{
    hspi2.Instance                 = SPI2;
    hspi2.Init.Mode                = SPI_MODE_MASTER;
    hspi2.Init.Direction           = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize            = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity         = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase            = SPI_PHASE_1EDGE;
    hspi2.Init.NSS                 = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler   = SPI_BAUDRATEPRESCALER_16;
    hspi2.Init.FirstBit            = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode              = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation      = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial       = 7;
    hspi2.Init.CRCLength           = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode            = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
}

/* HAL_SPI_MspInit callback — invoked from HAL_SPI_Init above. Only
 * configures GPIO + SPI2 clock; no DMA. */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}
