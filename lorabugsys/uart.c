/**@file uart.c
 *
 * @date Nov 8, 2018
 * @author Craig Hesling
 */

#include <string.h>

#include "LORABUG.h"
#include "PERIPHERALS.h"

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/gates/GateMutexPri.h>
#include <ti/sysbios/knl/Semaphore.h>
//#include <ti/sysbios/knl/Clock.h>

#include "io.h"

static PIN_State uartSensePinState;
static PIN_Handle uartSensePinHandle;

/* UART driver handle */
static UART_Handle uartHandle = NULL;

// Mutex to protect entry into functions where we use provided buffer
static GateMutexPri_Struct uartWriteMutexStruct;
// Semaphore to represent if we can issue another UART_write
static Semaphore_Struct    uartWriteSemStruct;

/*--------------------------------------------------------*
 *                    Configuration                       *
 *--------------------------------------------------------*/

#define UART_BAUD               115200
//#define UART_BAUD               3000000
#define UART_PRINTF_BUFFER_SIZE 256 // Should be a minimum of 5+17 for uarthexdump
#define UART_RECV_LINE_LENGTH   768


// Buffer for snprintf and uart_write
static char uartwritebuf[UART_PRINTF_BUFFER_SIZE];
static char uartlinebuf[UART_RECV_LINE_LENGTH + 1]; // +1 for '\0'

static const PIN_Config uartSensePinTable[] = {
        Board_UART_RX | PIN_INPUT_EN | PIN_NOPULL | PIN_HYSTERESIS | PIN_IRQ_POSEDGE,
        PIN_TERMINATE
};


static void uartWriteCallback(UART_Handle uart, void *buf, size_t count) {
    Semaphore_post(Semaphore_handle(&uartWriteSemStruct));
}

/**
 * @note This routine should not race with the uartsetup function because the interrupt
 *       is only registered after we decide not to call uart_open.
 */
static void uartOpen()
{
    if (uartHandle == NULL)
    {
        PIN_registerIntCb(uartSensePinHandle, NULL);
        PIN_setConfig(uartSensePinHandle, PIN_BM_IRQ|PIN_BM_PULLING, Board_UART_RX|PIN_IRQ_DIS|PIN_NOPULL);
        PIN_close(uartSensePinHandle);

        UART_Params uartParams;
        UART_Params_init(&uartParams);
        uartParams.baudRate = UART_BAUD;
        uartParams.readMode = UART_MODE_BLOCKING;
        uartParams.writeMode = UART_MODE_CALLBACK;
        // UARTCC26xx does not implement any of these parameters
        //    uartParams.readDataMode = UART_DATA_TEXT;
        //    uartParams.writeDataMode = UART_DATA_TEXT;
        //    uartParams.readReturnMode = UART_RETURN_NEWLINE;
        //    uartParams.readEcho = UART_ECHO_ON;
        uartParams.writeCallback = &uartWriteCallback;
        uartHandle = UART_open(Board_UART, &uartParams);
        if (!uartHandle)
        {
            System_abort("Failed to open UART\n");
        }
    }
}

/**
 * This callback is intended for the one time setup of UART,
 * in the event that USB is ever plugged in.
 */
static void uartRxIntCallback(PIN_Handle handle, PIN_Id pinId)
{
    uartOpen();
}

static void uartBufferedWrite(const char *data, size_t size) {
    if (size == 0) return;

    for (size_t start = 0; start < size; start += sizeof(uartwritebuf)) {
        Semaphore_pend(Semaphore_handle(&uartWriteSemStruct), BIOS_WAIT_FOREVER);
        size_t count = (size-start) < sizeof(uartwritebuf) ? (size-start) : sizeof(uartwritebuf);
        memcpy(uartwritebuf, &data[start], count);
        if (UART_write(uartHandle, uartwritebuf, count) == UART_ERROR)
        {
            System_abort("Failed to write str to uart\n");
            // should not need to post to semaphore, since the system should terminate
        }
    }
}

/*--------------------------------------------------------*
 *                  Interface Functions                   *
 *--------------------------------------------------------*/

bool uart_isopen()
{
    return uartHandle != NULL;
}

void uart_setup()
{
    GateMutexPri_construct(&uartWriteMutexStruct, NULL);
    Semaphore_construct(&uartWriteSemStruct, 1, NULL);

    uartSensePinHandle = PIN_open(&uartSensePinState, uartSensePinTable);
    if (uartSensePinHandle == NULL)
    {
        System_abort("Failed to open UART sense pins\n");
    }

    int val = PIN_getInputValue(PIN_ID(Board_UART_RX));
    if (val == 1) {
        uartOpen();
    } else {
        PIN_registerIntCb(uartSensePinHandle, uartRxIntCallback);
    }

}

void uart_write(const char *str, size_t size)
{
    if (uartHandle == NULL) return;

    IArg key = GateMutexPri_enter(GateMutexPri_handle(&uartWriteMutexStruct));

    uartBufferedWrite(str, size);

    GateMutexPri_leave(GateMutexPri_handle(&uartWriteMutexStruct), key);
}

/**
 * Print the given C string to UART followed by carriage return and new line.
 *
 * We define this little function, since we want the primary
 * logging to be over cJTAG, not UART.
 * @param str The C string to print
 */
void uart_puts(const char *str)
{
    if (uartHandle == NULL) return;

    IArg key = GateMutexPri_enter(GateMutexPri_handle(&uartWriteMutexStruct));

    uartBufferedWrite(str, strlen(str));
    uartBufferedWrite("\r\n", 2);

    GateMutexPri_leave(GateMutexPri_handle(&uartWriteMutexStruct), key);
}

void uart_vprintf(const char *format, va_list args)
{
    if (uartHandle == NULL) return;

    IArg key = GateMutexPri_enter(GateMutexPri_handle(&uartWriteMutexStruct));

    System_vsnprintf(uartwritebuf, sizeof(uartwritebuf), format, args);
    Semaphore_pend(Semaphore_handle(&uartWriteSemStruct), BIOS_WAIT_FOREVER);
    if (UART_write(uartHandle, uartwritebuf, strlen(uartwritebuf)) == UART_ERROR)
    {
        System_abort("Failed to write formatted buffer to uart\n");
    }

    GateMutexPri_leave(GateMutexPri_handle(&uartWriteMutexStruct), key);
}

void uart_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    uart_vprintf(format, args);
    va_end(args);
}

/**
 * Fetch a line from UART.
 * @return The null terminated line of at most \a UART_RECV_LINE_LENGTH string length.
 */
char *uart_readline()
{
    if (uartHandle == NULL) return "";

    size_t index;
    for (index = 0; index < (sizeof(uartlinebuf) - 1); index++)
    {
        int count = UART_read(uartHandle, uartlinebuf + index, 1);
        if (count == UART_ERROR)
        {
            System_abort("Failed to read line from uart\n");
        }
        if (uartlinebuf[index] == '\n')
        {
            break;
        }
    }
    uartlinebuf[index] = '\0';
    return uartlinebuf;
}

void uart_hexdump(uint8_t *data, size_t size)
{
    phexdump(uart_printf, data, size);
}
