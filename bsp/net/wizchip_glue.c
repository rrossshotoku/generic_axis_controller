/*
 * bsp/net/wizchip_glue.c — WIZnet ioLibrary port for STM32G4 + W6100 on SPI2.
 *
 * Pattern follows the working reference in uc_camd_interface/Core/Src/wizchip_port.c
 * but uses polled HAL_SPI_TransmitReceive instead of DMA. Polled is simpler,
 * easier to debug, and adequate for Phase 0b throughput (heartbeat logs).
 * If profiling later shows SPI as the bottleneck, swap to DMA — the
 * callback signatures don't change.
 *
 * CRIS callbacks are empty: the library uses them to bracket multi-step SPI
 * transactions against contention from other bus users. We have a single
 * caller and single SPI bus dedicated to the W6100, so there is nothing to
 * lock. Keeping them empty matches the reference's DMA-mode rationale.
 */

#include "wizchip_glue.h"
#include "wizchip_conf.h"

#include "spi.h"                /* CubeMX-generated; declares hspi2 */
#include "stm32g4xx_hal.h"

/*----------------------------------------------------------------------------
 * SPI timing
 *---------------------------------------------------------------------------*/

#define SPI_TIMEOUT_MS   50     /* per-transaction timeout */

/*----------------------------------------------------------------------------
 * CS control
 *---------------------------------------------------------------------------*/

static void cs_select(void)
{
    HAL_GPIO_WritePin(WIZCHIP_CS_GPIO_Port, WIZCHIP_CS_Pin, GPIO_PIN_RESET);
}

static void cs_deselect(void)
{
    HAL_GPIO_WritePin(WIZCHIP_CS_GPIO_Port, WIZCHIP_CS_Pin, GPIO_PIN_SET);
}

/*----------------------------------------------------------------------------
 * CRIS (critical section) — no-op
 *---------------------------------------------------------------------------*/

static void cris_enter(void) { }
static void cris_exit (void) { }

/*----------------------------------------------------------------------------
 * SPI byte/burst — polled, blocking, with timeout.
 *
 * Errors silently drop bytes. The ioLibrary's higher-level operations
 * (e.g. socket() returning failure) surface the consequence. A future
 * recovery layer can be added by tracking failures and re-initialising
 * SPI/HW reset; matches the reference's recover_spi() pattern.
 *---------------------------------------------------------------------------*/

static uint8_t spi_readbyte(void)
{
    uint8_t tx = 0xFF, rx = 0xFF;
    (void)HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, SPI_TIMEOUT_MS);
    return rx;
}

static void spi_writebyte(uint8_t b)
{
    uint8_t rx;
    (void)HAL_SPI_TransmitReceive(&hspi2, &b, &rx, 1, SPI_TIMEOUT_MS);
}

static void spi_readburst(uint8_t *buf, datasize_t len)
{
    /* Drive 0xFF out while clocking bytes in. We could use HAL_SPI_Receive
     * (which drives whatever the MOSI line floats at), but explicit-0xFF
     * matches the reference and avoids surprise behaviour on weird PHYs.
     *
     * datasize_t is int16_t in the ioLibrary. We treat it as unsigned
     * internally since lengths are never negative in practice. */
    static const uint8_t dummy[64] = {
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    };
    uint16_t remaining = (uint16_t)len;
    uint8_t  *p = buf;
    while (remaining > 0) {
        uint16_t chunk = (remaining < sizeof(dummy)) ? remaining : (uint16_t)sizeof(dummy);
        (void)HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)dummy, p, chunk, SPI_TIMEOUT_MS);
        p         += chunk;
        remaining -= chunk;
    }
}

static void spi_writeburst(uint8_t *buf, datasize_t len)
{
    /* HAL_SPI_Transmit ignores MISO. Adequate for register writes. */
    (void)HAL_SPI_Transmit(&hspi2, buf, (uint16_t)len, SPI_TIMEOUT_MS);
}

/*----------------------------------------------------------------------------
 * Public lifecycle
 *---------------------------------------------------------------------------*/

void wizchip_glue_reset_pulse(void)
{
    /* CS high (deselected) before touching RST. */
    cs_deselect();

    /* Active-low reset pulse. The W6100 datasheet specifies a minimum
     * 500 us reset pulse and ~150 ms internal startup. We give it more. */
    HAL_GPIO_WritePin(WIZCHIP_RST_GPIO_Port, WIZCHIP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(WIZCHIP_RST_GPIO_Port, WIZCHIP_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
}

void wizchip_glue_register_callbacks(void)
{
    /* Order matters per ioLibrary convention: CRIS, then CS, then SPI. */
    reg_wizchip_cris_cbfunc(cris_enter, cris_exit);
    reg_wizchip_cs_cbfunc(cs_select, cs_deselect);
    reg_wizchip_spi_cbfunc(spi_readbyte, spi_writebyte,
                           spi_readburst, spi_writeburst);
}
