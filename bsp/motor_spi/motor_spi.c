/*
 * bsp/motor_spi — SPI3 master, fixed 64-byte full-duplex transactions.
 *
 * The .ioc has PA15 as SPI3_NSS in hardware-output mode. STM32 hardware
 * NSS is tied to SPE: NSS is driven low while the SPI peripheral is
 * enabled (SPE = 1) and goes high when disabled (SPE = 0). To produce
 * an NSS edge between frames — which the motor MCU's slave DMA uses to
 * synchronise to a frame boundary — we explicitly disable SPE between
 * transfers. HAL_SPI_TransmitReceive re-enables SPE at the start of
 * each call, which gives the slave a clean falling NSS edge.
 */

#include "motor_spi.h"

#include "spi.h"                  /* CubeMX-generated; declares hspi3 */
#include "stm32g4xx_hal.h"

void motor_spi_init(void)
{
    /* SPI3 is configured by CubeMX (MX_SPI3_Init); leave SPE disabled here
     * so the very first motor_spi_transfer() produces an NSS falling edge. */
    __HAL_SPI_DISABLE(&hspi3);
}

int32_t motor_spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t timeout_ms)
{
    if (tx == NULL || rx == NULL) return -1;

    /* Belt-and-braces: ensure SPE is off so the upcoming HAL enable
     * produces a real NSS falling edge. */
    __HAL_SPI_DISABLE(&hspi3);

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
        &hspi3,
        (uint8_t *)tx, rx, MOTOR_SPI_FRAME_BYTES,
        timeout_ms);

    /* Always leave SPE off so NSS returns high (deasserted) between
     * transactions. The motor MCU's NSS-edge frame sync depends on this. */
    __HAL_SPI_DISABLE(&hspi3);

    return (st == HAL_OK) ? 0 : -1;
}
