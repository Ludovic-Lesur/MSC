# Summary
The MSC is a solar charger providing a 5V output on a standard female USB connector. The device is composed of two independant sub-boards: the charger itself and an optional monitoring block controlled by an MCU (featuring voltage and current measurements, IHM and data transmission).

# Hardware
The board was designed on **Circuit Maker V1.3**. Hardware documentation and design files are available @ https://circuitmaker.com/Projects/Details/Ludovic-Lesur/MSCHW1-1

# Embedded software

## Environment
The embedded software was developed under **Eclipse IDE** version 2019-06 (4.12.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

## Target
The MSC board is based on the **STM32L041K6U6** of the STMicroelectronics L0 family microcontrollers. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected target.

## Structure
The project is organized as follow:
* `inc` and `src`: **source code** split in 5 layers:
    * `registers`: MCU **registers** adress definition.
    * `peripherals`: internal MCU **peripherals** drivers.
    * `components`: external **components** drivers.
    * `sigfox`: **Sigfox library** API and low level implementation.
    * `applicative`: high-level **application** layers.
* `lib`: **Sigfox protocol library** files.
* `startup`: MCU **startup** code (from ARM).
* `linker`: MCU **linker** script (from ARM).

## Sigfox library

The Sigfox library is a compiled middleware which implements Sigfox protocol regarding framing, timing and RF frequency computation. It is based on low level drivers which depends on the hardware architecture (MCU and transceiver). Once implemented, the high level API exposes a simple interface to send messages over Sigfox network.

Last version of Sigfox library can be downloaded @ https://build.sigfox.com/sigfox-library-for-devices

For this project, the Cortex-M0+ version compiled with GCC is used.
