# Example Summary
This example is designed to make use of the [SensorBug accessory board](https://github.com/OpenChirp/LoRaBug/tree/master/Modules/SensorBug).
Upon powering up, the LoRaBug will attempt to Join the LoRaWAN network
(unless APB provisioned). The green LED will remain on during the join process
and will turn off when successfully joined.
After joining, the LoRaBug starts the main runtime, which is configured
to wake up at preset intervals and set all sensor reading.

Pressing the button, at any time, will result in the following two actions:
1. Scheduling sensor readings to be sent as soon as possible
2. If enabled, immediate transmission of BLE advertisements containing the
   DevEUI and AppKey. Using a BLE scanner the a device with the name **'L'** and MAC address **`00:11:22:33:44:55`** will contain two services 0xFA with the LoRa device EUID and 0xFB with the LoRa APPKEY.

It should also be noted that the LoRaBug will print it's DevEUI and AppKey
on boot over UART/USB and JTAG.
The USB/UART is configured to use 115200 baud, 8 data bits, 1 stop bit,
and not hardware or software flow control.
Futhermore, debug messages can be read from the device at any time, by simply
plugging in USB and opening a serial console.

See [app/main.c](app/main.c) for more information.

# Easybits Config

| Parameter Name       | Parameter Value  |
| -------------------- | ---------------- |
| `rxconfig` | `"counter,uint32,1", "battery,uint32,2", "light,uint32,3", "pir,uint32,4", "motion,uint32,5", "temperature,float,6", "humidity,float,7", "pressure,float,8", "gas,float,9", "noise,uint32,10", "rate,uint32,11", "motionenabled,bool,12", "lightenabled,bool,13", "micenabled,bool,14"` |
| `txconfig` | `"set_rate,uint32,1", "set_motionenabled,bool,2", "set_lightenabled,bool,3", "set_micenabled,bool,4", "reset,bool,10"` |



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
* BLE
* Accessory Header with SensorBug Sensors

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