/**@file io.h
 *
 * @author Craig Hesling
 * @date Apr 21, 2017
 */

#ifndef IO_H_
#define IO_H_

#include <stdint.h>
#include <stdbool.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>

#ifdef __cplusplus
extern "C" {
#endif


/*--------------------------------------------------------*
 *                    Configuration                       *
 *--------------------------------------------------------*/

//#define DISABLE_DEBUG_PRINT

/*--------------------------------------------------------*
 *                    Setup Functions                     *
 *--------------------------------------------------------*/
void setuppins();
void setupuart();


/*--------------------------------------------------------*
 *                  Interface Functions                   *
 *--------------------------------------------------------*/
void uartwrite(const char *str, size_t size);
void uartputs(const char *str);
void uartprintf(const char *format, ...)
    __attribute__ ((format (printf, 1, 2)));
void uartvprintf(const char *format, va_list args)
    __attribute__ ((format (printf, 1, 0)));
char *uartreadline();

#ifndef DISABLE_DEBUG_PRINT
/**
 * Printf to UART and JTAG
 */
void debugprintf(const char *format, ...)
        __attribute__ ((format (printf, 1, 2)));
#else
#define debugprintf(fmt, ...) do { } while(0)
#endif

void setPin(PIN_Id pin, uint_t value);
void togglePin(PIN_Id pin);
int  getPinInput(PIN_Id pin);

void enableLeds();
void disableLeds();
void setLed(PIN_Id pin, uint_t value);
void toggleLed(PIN_Id pin);

int getButtonState();
void setBtnCallback(void (*callback)(void));

void hexdump(uint8_t *data, size_t size);
void uarthexdump(uint8_t *data, size_t size);
#ifndef DISABLE_DEBUG_PRINT
void debughexdump(uint8_t *data, size_t size);
#else
#define debughexdump(data, size) do { } while(0)
#endif

void hardreset();
bool isjtagpoweron();

#ifdef __cplusplus
}
#endif

#endif /* IO_H_ */
