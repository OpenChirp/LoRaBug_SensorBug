/**@file io.c
 *
 * @author Craig Hesling
 * @date Apr 21, 2017
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#include "LORABUG.h"
#include "PERIPHERALS.h"
#include "board.h"

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* TI-RTOS Header files */
// #include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
//#include <ti/drivers/SPI.h>
//#include <ti/drivers/UART.h>
//#include <ti/drivers/Watchdog.h>

/* BIOS Header files */
#include <ti/sysbios/gates/GateMutexPri.h>
#include <ti/sysbios/knl/Clock.h>

/* Non-RTOS Drivers */
#include <driverlib/sys_ctrl.h> // SysCtrlSystemReset()
#include <driverlib/aon_wuc.h>  // AONWUCPowerStatusGet() & AONWUC_JTAG_POWER_ON

#include "io.h"

/* Settings */
#define HEXDUMP_STR_PREFIX "# " // Must be defined, but can be ""

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;
static PIN_Handle hdrPinHandle;
static PIN_State hdrPinState;

static PIN_Handle btnPinHandle;
static PIN_State btnPinState;
static void (*btnCallback)(void) = NULL;

/* Clock used for debounce logic */
static Clock_Struct buttonClock;
static Clock_Handle hButtonClock;

/* Clock used for LED timeout logic */
static Clock_Struct gLedClock;
static Clock_Struct rLedClock;

static bool ledsEnabled = true;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
static const PIN_Config ledPinTable[] = {
        Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
        PIN_TERMINATE
};

static const PIN_Config hdrPinTable[] = {
//     Board_HDR_HDIO0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//     Board_HDR_HDIO1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//     Board_HDR_HDIO2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//     Board_HDR_ADIO0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
//     Board_HDR_ADIO1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
//     Board_HDR_ADIO2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
//     Board_HDR_ADIO3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
//     Board_HDR_ADIO4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
//     Board_HDR_ADIO5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
//     Board_HDR_ADIO6 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
//     Board_HDR_ADIO7 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,
        PERIPHERALS_HDR_IO_PINS
        PIN_TERMINATE
};

static const PIN_Config btnPinTable[] = {
        Board_BTN | PIN_INPUT_EN | PIN_NOPULL | PIN_HYSTERESIS | PIN_IRQ_NEGEDGE,
        PIN_TERMINATE
};


/*!*****************************************************************************
 *  @brief      Button clock callback
 *
 *  Called when the debounce period is over. Stopping the clock, toggling
 *  the device mode based on activeButtonPinId:
 *
 *  Reenabling the interrupts and resetting the activeButtonPinId.
 *
 *  @param      arg  argument (PIN_Handle) connected to the callback
 *
 ******************************************************************************/
static void buttonClockCb(UArg arg)
{
    PIN_Handle buttonHandle = (PIN_State *) arg;

    /* Stop the button clock */
    Clock_stop(hButtonClock);

    /* Debounce logic, only toggle if the button is still pushed (low) */
    if (!PIN_getInputValue(Board_BTN))
    {
        /* Take button action - now that we have confirmed the proper edge */
        if (btnCallback == NULL)
        {
            hardreset();
        }
        btnCallback();
    }

    /* Re-enable interrupts to detect button release. */
    PIN_setConfig(buttonHandle, PIN_BM_IRQ, Board_BTN | PIN_IRQ_NEGEDGE);
}


/*******************************************************************************
 *  @brief      Button callback
 *
 *  Initiates the debounce period by disabling interrupts, setting a timeout
 *  for the button clock callback and starting the button clock.
 *
 *  Default action will be to reset the board (which may trigger bootloader).
 *  Alternatively, the user registered callback will occur if available.
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void btnIntCallback(PIN_Handle handle, PIN_Id pinId)
{
    /* Disable interrupts during debounce */
    PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);

    /* Set timeout 50 ms from now and re-start clock */
    Clock_setTimeout(hButtonClock, (50 * (1000 / Clock_tickPeriod)));
    Clock_start(hButtonClock);
}

/*!*****************************************************************************
 *  @brief      LED Timeout callback
 *
 *  Called when the LED timeout period has expired. The LED is assumed to still
 *  be on and should now be turned off.
 *
 *  Default action is to turn off the specified LED.
 *
 *  @param      arg  argument (PIN_Handle) connected to the callback
 *
 ******************************************************************************/
static void ledTimeoutCb(UArg arg) {
    PIN_Id pin = (PIN_Id) arg;

    if (PIN_setOutputValue(ledPinHandle, pin, 0) != PIN_SUCCESS)
    {
        System_abort("Failed to set LED value\n");
    }
}

void setuppins()
{
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (ledPinHandle == NULL)
    {
        System_abort("Failed to open board LED pins\n");
    }
    hdrPinHandle = PIN_open(&hdrPinState, hdrPinTable);
    if (hdrPinHandle == NULL)
    {
        System_abort("Failed to open board header pins\n");
    }
    btnPinHandle = PIN_open(&btnPinState, btnPinTable);
    if (btnPinHandle == NULL)
    {
        System_abort("Failed to open board BTN pin\n");
    }

    /* Construct clock for debounce */
    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
    clockParams.arg = (UArg) btnPinHandle;
    Clock_construct(&buttonClock, buttonClockCb, 0, &clockParams);
    hButtonClock = Clock_handle(&buttonClock);

    /* Contruct clocks for Led timeouts */
    Clock_Params_init(&clockParams);
    clockParams.arg = (UArg) Board_GLED;
    Clock_construct(&gLedClock, ledTimeoutCb, 0, &clockParams);
    Clock_Params_init(&clockParams);
    clockParams.arg = (UArg) Board_RLED;
    Clock_construct(&rLedClock, ledTimeoutCb, 0, &clockParams);


    /* Register Callback for Button Interrupt */
    if (PIN_registerIntCb(btnPinHandle, btnIntCallback) != PIN_SUCCESS)
    {
        System_abort("Failed to register btn int callback\n");
    }
}


void setPin(PIN_Id pin, uint_t value)
{
    if (PIN_setOutputValue(hdrPinHandle, pin, value) != PIN_SUCCESS)
    {
        System_abort("Failed to set pin value\n");
    }
}

void togglePin(PIN_Id pin)
{
    if (PIN_setOutputValue(hdrPinHandle, pin, !PIN_getOutputValue(pin))
            != PIN_SUCCESS)
    {
        System_abort("Failed to toggle pin value\n");
    }
}

int getPinInput(PIN_Id pin)
{
    return (int) PIN_getInputValue(pin);
}


void enableLeds() {
    ledsEnabled = true;
}

void disableLeds() {
    ledsEnabled = false;

    if (PIN_setOutputValue(ledPinHandle, Board_GLED, 0) != PIN_SUCCESS)
    {
        System_abort("Failed to set pin value\n");
    }
    if (PIN_setOutputValue(ledPinHandle, Board_RLED, 0) != PIN_SUCCESS)
    {
        System_abort("Failed to set pin value\n");
    }
}

void setLed(PIN_Id pin, uint_t value)
{
    if (!ledsEnabled) {
        return;
    }

    if (PIN_setOutputValue(ledPinHandle, pin, value) != PIN_SUCCESS)
    {
        System_abort("Failed to set pin value\n");
    }
}

void toggleLed(PIN_Id pin)
{
    if (!ledsEnabled) {
        return;
    }

    if (PIN_setOutputValue(ledPinHandle, pin, !PIN_getOutputValue(pin))
            != PIN_SUCCESS)
    {
        System_abort("Failed to toggle pin value\n");
    }
}

void timedLed(PIN_Id pin, unsigned ms) {
    Clock_Handle clk;

    if (!ledsEnabled) {
        return;
    }

    switch (pin) {
    case Board_GLED:
        clk = Clock_handle(&gLedClock);
        break;
    case Board_RLED:
        clk = Clock_handle(&rLedClock);
        break;
    default:
        System_abort("timedLed given bad pin\n");
    }

    setLed(pin, 1);

    Clock_stop(clk);
    Clock_setTimeout(clk, (UInt32)(ms*1000 / Clock_tickPeriod));
    Clock_start(clk);
}

int getButtonState()
{
    return (int) PIN_getInputValue(Board_BTN);
}

void setBtnCallback(void (*callback)(void))
{
    btnCallback = callback;
}

/* Utilities */

typedef void printf_t(const char *format, ...)
__attribute__ ((format (printf, 1, 2)));

static void _hexdump(printf_t prnf, uint8_t *data, size_t size)
{
    // Stolen from https://gist.github.com/ccbrown/9722406
    char ascii[17];
    size_t i, j;
    ascii[16] = '\0';
    for (i = 0; i < size; ++i)
    {
        prnf("%02X ", ((unsigned char*) data)[i]);
        if (((unsigned char*) data)[i] >= ' '
                && ((unsigned char*) data)[i] <= '~')
        {
            ascii[i % 16] = ((unsigned char*) data)[i];
        }
        else
        {
            ascii[i % 16] = '.';
        }
        if ((i + 1) % 8 == 0 || i + 1 == size)
        {
            prnf(" ");
            if ((i + 1) % 16 == 0)
            {
                prnf("|  %s \n", ascii);
            }
            else if (i + 1 == size)
            {
                ascii[(i + 1) % 16] = '\0';
                if ((i + 1) % 16 <= 8)
                {
                    prnf(" ");
                }
                for (j = (i + 1) % 16; j < 16; ++j)
                {
                    prnf("   ");
                }
                prnf("|  %s \n", ascii);
            }
        }
    }
}

void hexdump(uint8_t *data, size_t size)
{
    _hexdump(jtag_printf, data, size);
}

void uarthexdump(uint8_t *data, size_t size)
{
    _hexdump(uart_printf, data, size);
}


/**
 * Printf to UART and JTAG
 *
 * @note This CANNOT be used in a Hwi or Swi (pin callbacks included), since this function blocks in uart
 */
void allprintf(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    // Print to JTAG debugger
    // System_vprintf only consumes about 144 bytes stack space, whereas vprintf consumes 1,300 bytes
    jtag_vprintf(format, args);
    // Print to UART console
    uart_vprintf(format, args);

    va_end(args);
}

void allhexdump(uint8_t *data, size_t size)
{
    hexdump(data, size);
    uarthexdump(data, size);
}


inline void hardreset() {
    SysCtrlSystemReset();
}
