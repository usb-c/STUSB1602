# STUSB1602 SW drivers (USB Power Delivery)
Dual-role USB-PD controller for STM32.<br/>
<br/>
USB-C PD 3.0 analog front-end controller, managed by both I2C and SPI interfaces. Need to run the USB-PD stack in software.<br/>
The provided USB-PD stack only works on STM32F0 and STM32F4.<br/>
Certified by USB-IF consortium for USB Type-C and USB Power Delivery applications (USB-C & USB PD). <br/>


Info:
----------------
* __Device:__       STUSB1602A : USB PD controller  <br />
* __Manufacturer:__ STMicroelectronics
* __Typical Application:__ To communicate over the CC line of the USB-C connector, and negociate power on VBUS
* __USB power role:__ Source, Sink, or Dual-Role Power (DRP)
* __USB data  role:__ independant (works with No Data, or with USB Data Device that can be USB2.0 or USB3.1)   <br />
Note: If using the USB-C connector for both Power and Data, then the USB-C specification requires for a Power Sink application that its initial data role (at cable attachment) is always UFP (Device).  <br />
Similarly, the initial data role of a Power Source application is always DFP (Host). <br />
To change the role, it is needed to send a USB PD command: PowerRole_Swap or DataRole_Swap.

* __Features:__ software based solution (STM32+STUSB1602) <br />
* __Requirement:__ STM32F0 or STM32F4 microcontroller <br />
* __Operating supply:__ 5V to 20V directly from Vbus of the USB-C cable. <br /> Or a lower power (3.3V) for battery supplied applications <br />
* __Power Consumption:__ 113uA for Sink role, 158uA for Source role<br />
* __Package:__ QFN24 EP 4x4mm <br />
* __Part number:__ STUSB1602QTR, STUSB1602AQTR
* __USB Spec:__ USB Type-C v1.2 , USB PD v2.0 & v3.0
* __USB Certification:__ test ID =  1010032

