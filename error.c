/**@file error.c
 *
 * @date Nov 6, 2018
 * @author Craig Hesling
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include "io.h"

/**
 * This function is referenced from @ref rtos.cfg as an exception hook function.
 * It's role is to optionally reset the MCU if JTAG is not present.
 *
 * rtos.cfg:
 * m3Hwi.enableException = true;
 * m3Hwi.excHookFunc = "&ExceptionHandle";
 * The m3Hwi.excHandlerFunc must be unset or
 * set to Hwi.excHandlerMax or Hwi.excHandlerMin .
 *
 * Note that the after the call to this function BIOS_exit(0) is executed.
 * So, the handlers for a bios exit must be considered also.
 *
 * See Hwi.xdc, Hwi.xs, and Hwi.c for more information.
 * They are in the sysbios/family/arm/m3 directory
 *
 * @param excContext
 */
Void ExceptionHandle(Hwi_ExcContext *excContext) {
    if (!isjtagpoweron()) {
        hardreset();
    }
}

/**
 * This function is called when BIOS_exit has been  called.
 * It's role is to optionally reset the MCU if JTAG is not present.
 *
 * This function should be registered in the rtos.cfg
 * as System.exitFxn.
 *
 * @param stat The status code given to BIOS_exit or BIOS_abort
 */
Void ExitHandler(Int stat) {
    if (!isjtagpoweron()) {
        hardreset();
    }
    for (;;) ;
}

/**
 * This function is called when BIOS_abort (maybe System_abort("")) has been called.
 * It's role is to optionally reset the MCU if JTAG is not present.
 *
 * This function be registered in the rtos.cfg
 * as System.abortFxn .
 *
 */
Void AbortHandler() {
    ExitHandler(-1);
}

/**
 * Get the current timestamp
 * @param[out] time The
 */
inline
void TimestampNow(Types_Timestamp64 *time) {
    Timestamp_get64(time);
}

/**
 * Return the number of nanoseconds between before and after timestamps
 * If an overflow is reported, the output nanoseconds value should be considered incorrect.
 *
 *
 * @param[out] nanoseconds The output number of nanoseconds
 * @param[in] before The earlier time we should consider for the difference
 * @param[in] after The later time we should consider for the difference
 * @return True if no overflow occurred. False if an overflow occurred.
 */
inline
bool TimestampDiffNs(uint64_t *nanoseconds, Types_Timestamp64 *before, Types_Timestamp64 *after) {
    /// @note The overflow return value does not currently work.
    /// @todo Implement the part by part multi and addition to check for overflows
    Types_FreqHz freq;
    Timestamp_getFreq(&freq);

    uint64_t before64 = ((uint64_t)(before->hi))<<32 | ((uint64_t)(before->lo));
    uint64_t after64  = ((uint64_t)(after->hi))<<32  | ((uint64_t)(after->lo));
    uint64_t freq64   = ((uint64_t)(freq.hi))<<32   | ((uint64_t)(freq.lo));

    *nanoseconds = ((after64-before64) * 1e9) / freq64;
    return true;
}


#ifndef DISABLE_DEBUG_PRINT

/**
 * Printf to UART and JTAG
 *
 * @note This CANNOT be used in a Hwi or Swi (pin callbacks included), since this function blocks in uart
 */
void debugprintf(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    // Print to JTAG debugger
    // System_vprintf only consumes about 144 bytes stack space, whereas vprintf consumes 1,300 bytes
    System_vprintf(format, args);
    System_flush();
    // Print to UART console
    uartvprintf(format, args);

    va_end(args);
}

void debughexdump(uint8_t *data, size_t size)
{
    hexdump(data, size);
    uarthexdump(data, size);
}

#endif
