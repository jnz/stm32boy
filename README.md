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

Warning !! This will not compile without a ROM file, which is NOT part of this repository.

Adding a ROM file
-----------------

You must manually add your ROM file as `Core/Src/gameboy_rom.h`.
With the `xxd`, just run:

    cd Core/Src
    xxd -i gameboy_rom.gb > gameboy_rom.h

The variable name must be `gameboy_rom_gb` and the (large) file should look
something like this:

    unsigned char gameboy_rom_gb[] = {
      0xc3, 0x0c, 0x02, ...

