/*
 * boot_log_stub — no-op implementation of app/log's public API.
 *
 * bsp/net (and possibly other bsp modules) call LOG_INFO / LOG_ERROR
 * expecting the app's log module to be present. In the bootloader we
 * don't ship the log module (which drags in TCP, ring buffers, and the
 * whole telemetry stack). This stub satisfies the linker with zero
 * runtime cost.
 *
 * If bootloader diagnostics ever become important (e.g. UART echo of
 * update progress), replace this with something more useful.
 */

#include <stdarg.h>
#include <stdint.h>

/* Deliberately mirror the app/log/log.h prototype but keep this file
 * standalone — no #include so this file can be added to any bootloader
 * build without header path fuss. */
void log_printf(int level, const char *fmt, ...);
void log_printf(int level, const char *fmt, ...)
{
    (void)level;
    (void)fmt;
}
