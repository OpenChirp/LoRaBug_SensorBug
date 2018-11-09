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
 *                    Setup Functions                     *
 *--------------------------------------------------------*/
void setuppins();
void setupuart();


/*--------------------------------------------------------*
 *                  Interface Functions                   *
 *--------------------------------------------------------*/

/* Utiities */
typedef void printf_t(const char *format, ...)
__attribute__ ((format (printf, 1, 2)));

void phexdump(printf_t prnf, uint8_t *data, size_t size);

void hardreset(void);

/* uart.c */
bool uart_isopen(void);
void uart_write(const char *str, size_t size);
void uart_puts(const char *str);
void uart_printf(const char *format, ...)
    __attribute__ ((format (printf, 1, 2)));
void uart_vprintf(const char *format, va_list args)
    __attribute__ ((format (printf, 1, 0)));
char *uart_readline();
void uart_hexdump(uint8_t *data, size_t size);

/* jtag.c */
bool jtag_ispowered(void);
void jtag_vprintf(const char *format, va_list args)
    __attribute__ ((format (printf, 1, 0)));
void jtag_printf(const char *format, ...)
    __attribute__ ((format (printf, 1, 2)));
void jtag_hexdump(uint8_t *data, size_t size);

void setPin(PIN_Id pin, uint_t value);
void togglePin(PIN_Id pin);
int  getPinInput(PIN_Id pin);

void enableLeds();
void disableLeds();
void setLed(PIN_Id pin, uint_t value);
void toggleLed(PIN_Id pin);
/**
 * Turn on the onboard led for a fixed amount of time
 * @param pin Board_GLED or Board_RLED
 * @param ms The timeout in milliseconds
 */
void timedLed(PIN_Id pin, unsigned ms);

int getButtonState();
void setBtnCallback(void (*callback)(void));

/**
 * Printf to UART and JTAG
 */
void allprintf(const char *format, ...)
        __attribute__ ((format (printf, 1, 2)));
void allhexdump(uint8_t *data, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* IO_H_ */
