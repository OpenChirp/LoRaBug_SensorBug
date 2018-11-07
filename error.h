/**@file error.c
 *
 * @date Nov 7, 2018
 * @author Craig Hesling
 */

#ifndef ERROR_H_
#define ERROR_H_

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Timestamp.h>

#include <stdbool.h>

/*--------------------------------------------------------*
 *                    Configuration                       *
 *--------------------------------------------------------*/

/**@def DISABLE_DEBUG_PRINT
 * When defined, all debugprintf and debughexdump functions
 * are disabled and the constants associated with them
 * disappear.
 */
//#define DISABLE_DEBUG_PRINT

/*--------------------------------------------------------*
 *                  Interface Functions                   *
 *--------------------------------------------------------*/

void TimestampNow(Types_Timestamp64 *time);
bool TimestampDiffNs(uint64_t *nanoseconds, Types_Timestamp64 *before, Types_Timestamp64 *after);

#ifndef DISABLE_DEBUG_PRINT

/**
 * Printf to UART and JTAG
 */
void debugprintf(const char *format, ...)
        __attribute__ ((format (printf, 1, 2)));
void debughexdump(uint8_t *data, size_t size);

#else

#define debugprintf(fmt, ...) do { } while(0)
#define debughexdump(data, size) do { } while(0)

#endif


#endif /* ERROR_H_ */
