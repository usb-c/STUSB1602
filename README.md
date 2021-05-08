# STUSB1602 SW drivers (USB-C Power Delivery)
Dual-role USB-PD controller for STM32, with Power up to 100W (20V@5A).<br/>
It runs the USB-PD software stack for STM32. <br/>
<br/>
USB-C PD 3.0 analog front-end controller, managed by both I2C and SPI interfaces. Need to run the USB-PD stack in software.<br/>
The provided USB-PD stack only works on STM32 microcontroller family.<br/>
Certified by USB-IF consortium for USB Type-C and USB Power Delivery applications (USB-C & USB PD). <br/>


Info:
----------------
* __Device:__       STUSB1602A : USB-C Power Delivery with PPS <br />
* __Manufacturer:__ STMicroelectronics
* __Typical Application:__ To communicate over the CC line of the USB-C connector, and negociate power on VBUS
* __USB power role:__ Source, Sink, or Dual-Role Power (DRP)
* __USB data  role:__ independant (works with No Data, or with USB Data Device that can be USB2.0 or USB3.1)   <br />
Note: If using the USB-C connector for both Power and Data, then the USB-C specification requires for a Power Sink application that its initial data role (at cable attachment) is always UFP (Device).  <br />
Similarly, the initial data role of a Power Source application is always DFP (Host). <br />
To change the role, it is needed to send a USB PD command: PowerRole_Swap or DataRole_Swap.

* __Features:__ software based solution (STM32+STUSB1602) <br />
* __Requirements:__ ARM Cortex-M microcontroller : <br />
  - STM32F072 (ARM Cortex-M0 core)
  - or STM32F446 (ARM Cortex-M4 core)
  - or STM32G474 (ARM Cortex-M4 core)
  - or STM32L4R5 (ARM Cortex-M4 core)
* __GPIO Requirements:__ SPI (4 pins) + I2C (2 pins) + Interrupt (1 pin) : <br />
  - For 1 USB-C port : 1x SPI + 1x I2C + 1x Interrupt
  - For 2 USB-C ports: 2x SPI + 1x I2C + 2x Interrupt
  <br />
  Note: SPI is used for USB-PD messaging to CC pin, and I2C is used for chip register's configuration

* __Operating supply:__ 5V to 20V directly from Vbus of the USB-C cable. <br /> Or a lower power (3.3V) for battery supplied applications <br />
* __Power Consumption:__ 113uA for Sink role, 158uA for Source role<br />
* __Package:__ QFN24 EP 4x4mm <br />
* __Part number:__ STUSB1602QTR, STUSB1602AQTR
* __USB Spec:__ USB Type-C v1.2 , USB PD v2.0 & v3.0
* __USB Certification:__ test ID =  1010032

Notes:
----------------
- The USB-PD software stack has only been tested on STM32 with official USB-C test equipments to pass the USB certification, so it is only supported for STM32.
- STUSB1602 Linux drivers: not available
