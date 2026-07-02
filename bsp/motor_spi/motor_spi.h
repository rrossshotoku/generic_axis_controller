/*
 * bsp/motor_spi — SPI3 master driver to the motor-control MCU.
 *
 * Each call performs one fixed-size 64-byte full-duplex transaction.
 * The Interface contract (../Interface/INTERFACE_SPEC.md §1-§2) mandates:
 *   - 64-byte frame per SPI transaction (always exactly 64 bytes clocked),
 *   - NSS edges as frame markers (slave DMA re-syncs on the NSS edge),
 *   - SPI mode 0, 8-bit, MSB-first,
 *   - clock at MC_IF_SPI_CLOCK_HZ_INITIAL = 6 MHz initially.
 *
 * The CubeMX-generated SPI3 init handles mode/word-size/clock. This module
 * adds the NSS-per-transaction behaviour by toggling SPE around each
 * transfer (SPE off → NSS high; SPE on → NSS low).
 *
 * Polled blocking implementation (HAL_SPI_TransmitReceive with timeout).
 * At 6 MHz a 64-byte transfer takes ~85 us; CPU is busy-waiting for that
 * time. Upgrade to DMA when sustained throughput becomes a concern; the
 * API does not change.
 */

#ifndef BSP_MOTOR_SPI_H
#define BSP_MOTOR_SPI_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed transfer size per the Interface contract. The caller MUST provide
 * exactly 64-byte buffers; shorter or longer is a programming error. */
#define MOTOR_SPI_FRAME_BYTES   64u

void motor_spi_init(void);

/* Perform one full-duplex 64-byte transaction.
 * tx and rx may alias.
 * Returns 0 on success, <0 on error (SPI fault, timeout). On error the
 * contents of rx are undefined and the SPI peripheral is left disabled. */
int32_t motor_spi_transfer(const uint8_t tx[MOTOR_SPI_FRAME_BYTES],
                                 uint8_t rx[MOTOR_SPI_FRAME_BYTES],
                           uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_SPI_H */
