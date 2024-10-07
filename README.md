stm32boy
--------

Running Game Boy ROMS on the STM32 ARM Cortex microcontroller.

![logo](/img/stm32boy.png) ![gif](img/demo.gif?raw=1)

Build
-----

Create a `config.mk` with the following path to the cross compiler GCC:

    TOOLCHAIN_ROOT=/path/to/gcc-arm-none-eabi-XX.XX-XX/bin/

Then run:

    make

⚠️  This will not compile without a ROM file, which is NOT part of this repository. ⚠️

Getting the hardware
--------------------

Get the following board:
[STM32F429 Discovery Board](https://www.st.com/en/evaluation-tools/32f429idiscovery.html)

Available e.g. at:
 - [Farnell](https://de.farnell.com/stmicroelectronics/stm32f429i-disc1/entwicklungsboard-advanced-line/dp/2506924)
 - [DigiKey](https://www.digikey.com/en/products/detail/stmicroelectronics/STM32F429I-DISC1/5731713)
 - [Mouser](https://eu.mouser.com/ProductDetail/STMicroelectronics/STM32F429I-DISC1?qs=79dOc3%2F91%2Fed3%252BRc5JUCEw%3D%3D)
 - [Reichelt](https://www.reichelt.de/discovery-kit-stm32f411-128kb-ram-512kb-flash-stm32f429i-disc1-p353434.html)

Adding a ROM file
-----------------

You must manually add your ROM file as `Core/Src/gameboy_rom.h`.
With the `xxd` tool installed on Linux, just run:

    cd Core/Src
    xxd -i gameboy_rom.gb > gameboy_rom.h

The variable name must be `gameboy_rom_gb` and the (large) file should look
something like this:

    unsigned char gameboy_rom_gb[] = {
      0xc3, 0x0c, 0x02, ...


Thanks
------

Thanks to deltabeard for the single header Peanug-GB emulator library.

