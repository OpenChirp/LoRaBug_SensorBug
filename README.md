# Example Summary
This example turns a LoRaBug into a LoRaWAN remote button trigger.
Upon powering up, the LoRaBug will attempt to Join the LoRaWAN network
(unless APB provisioned). Immediately after Joining, it will go to sleep until
the button is pressed (typically less than 300nA).
Upon a button press, the LoRaBug will send an unconfirmed LoRaWAN packet
containing a counter value and it's current battery voltage.
The payload is the counter followed by the battery voltage as plain uint32s
in little endian.

See [app/main.c](app/main.c) for more information.

# ByteTranslator Config

| Parameter Name       | Parameter Value  |
| -------------------- | ---------------- |
| Incoming Field Names | counter, battery |
| Incoming Field Types | uint32, uint32   |
| Endianness           | little           |


# Cloning and Submodules

```
git clone https://github.com/OpenChirp/LoRaBug_BlinkySleep.git
git submodule update --init --recursive
```

# Peripherals Exercised

* `Board_RLED`
* `Board_GLED`
* `Board_BTN`
* LoRaWAN

# Application Design Details

## Minimal footprint
This example is based on the CC2650 Launchpad empty_min example project.

The empty_min is the same as the Empty example except many development
and debug features are disabled. For example:

* No Kernel Idle task
* No stack overflow checking
* No Logs or Asserts are enabled

> The ROM is being used in this example. This is controlled
> by the following lines in the *.cfg* file:

```
var ROM = xdc.useModule('ti.sysbios.rom.ROM');
    ROM.romName = ROM.CC2650;
```
> Since the kernel in the ROM is being used, there is no logging or assert
checking done by the kernel.

## Bootloader
The button is configured in trigger the bootloader if depressed during reset.
This is configured in the [ccfg.c](ccfg.c) file.

# References

* For GNU and IAR users, please read the following website for details
  about enabling [semi-hosting](http://processors.wiki.ti.com/index.php/TI-RTOS_Examples_SemiHosting)
  in order to view console output.

* Please refer to the __Memory Footprint Reduction__ section in the
TI-RTOS User Guide *spruhd4.pdf* for a complete and detailed list of the
differences between the empty minimal and empty projects.

# Developer Notes
Unfortunately, the LoRaMAC-node implementation of LoRaWAN uses C99's in for loop
variable declaration. For this reason, you MUST change the C compiler Language
mode to allow C99 syntax.
Currently, the C99 syntax usage is isolated to the
[LoRaMac.c](loramac/src/mac/LoRaMac.c) file.