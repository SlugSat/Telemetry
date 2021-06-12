
CC1200 Driver for the STM32 Microcontroller
===========================================

[//]: # (############################### Table of Contents ############################)
## Table of Contents
- [Introduction](#introduction)
- [Setup](#setup)
- [Features](#features)
- [References](#references)

[//]: # (############################### Introduction ############################)
## Introduction
Contained in this project are the basic driver files for communicating
with and controlling the CC1200 transceiver chip. Below is a list of all the main driver files:

- *cc1200.h/.c*
	- Main SPI implementation for interacting with the cc1200
- *cc1200_reg.h*
	- Main definition file where all relevant constants are defined
- *CircularBuffer.h/.c*
	- Circular buffer implementation which is used for received and
		and transmitted data.

[//]: # (############################### Setup ############################)
## Setup
There are several development tools which can be used with the STM32,
but here I will only introduce the tools I have personally used as
these are the only ones I have experience with.

For basic peripheral code generation I used the STM32CubeMX code
generator software. The link for this can be found in the references section.

The IDE used was Attolic TrueSTUDIO. The link to the download page can
also be found in the references section.

For initial register configuration testing and quick development, SmartRF Studio 7 was used along with the SmartRF development kit. The cc1200 evaluation modules are plugged into SmartRF evaluation boards. The SmartRF EBs are then plugged into a host computer running SmartRF Studio which is used to program the cc1200 chips and test different transmission/reception setups. The register settings can then be exported as an array of register_setting structs. This can then be easily imported into the user STM32 program for program loading.

[//]: # (############################### Features ############################)
## Features
Currently the driver software supports the following interactions
with the cc1200 transceiver:

- Read from and write to all cc1200 configuration registers in both
the normal address space and the extended address space.
- Configure the cc1200 with register settings copied from the SmartRF
software
- Transmit packets using standard FIFO access
- Receive packets using standard FIFO access
- Detect and receive packets using the eWOR (Enhanced Wake-on-Radio) mode on the cc1200.

[//]: # (############################### REFERENCES ############################)
## References
#### CC1200
Use the following references for CC1200 related datasheets:
- [CC120X]
- [CC1200]

#### STM32
Use the following reference for the STM32 datasheet:
- [STM32L152RET6]

#### Development Tools
Download the development tools here:
- [STM32CubeMX]
- [Attolic TrueSTUDIO]
- [SmartRF Studio 7]
- [CC1200 Development Kit]

[//]: # (############################### REFERENCE LINKS ############################)
[CC120X]:https://www.ti.com/lit/ug/swru346b/swru346b.pdf?ts=1612317166591&ref_url=https%253A%252F%252Fwww.google.com%252F
[CC1200]:https://www.ti.com/lit/ds/symlink/cc1200.pdf?ts=1612287308778&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FCC1200:

[STM32L152RET6]:https://www.st.com/resource/en/reference_manual/cd00240193-stm32l100xx-stm32l151xx-stm32l152xx-and-stm32l162xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

[STM32CubeMX]:https://www.st.com/en/development-tools/stm32cubemx.html
[Attolic TrueSTUDIO]:https://www.st.com/en/development-tools/truestudio.html
[SmartRF Studio 7]:https://www.ti.com/tool/SMARTRFTM-STUDIO
[CC1200 Development Kit]:https://www.ti.com/tool/CC1200DK
