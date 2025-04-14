## Embedded software for STMicro MCU in handheld PEMF device 

This repo houses embedded software for the [STM32G431CBT6 MCU](https://www.lcsc.com/datasheet/lcsc_datasheet_2304140030_STMicroelectronics-STM32G431CBT6_C529355.pdf) employed in a handheld, battery powered [PEMF-PROTOTYPE](https://github.com/ajdonich/pemf-prototype) device. The primary function of this software is to receive I2C frequency change commands and control the input signal to the electromagnet driver circuit. The DMA-DAC on the STM32G431 can deliver a relatively high resolution (~1Msps) input signal to the driver. Initial dev for this MCU was done on this [Nucleo-32 Dev Board](https://www.st.com/resource/en/user_manual/um2397-stm32g4-nucleo32-board-mb1430-stmicroelectronics.pdf).

Note: the code stored in this repo is the minimal necessary to track manual changes and is **not** itself sufficient to compile or run. It must be used in conjuction with an appropriately configured [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) project for the STM32G431.

