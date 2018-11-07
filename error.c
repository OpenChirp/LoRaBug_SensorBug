/**@file error.c
 *
 * @date Nov 6, 2018
 * @author Craig Hesling
 */

#include <xdc/std.h>
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
