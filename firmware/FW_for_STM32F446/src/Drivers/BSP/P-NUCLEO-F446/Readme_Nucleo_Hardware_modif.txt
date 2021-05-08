*****************************************************
**  Hardware modifications to be done on MB1137    **
**  in order to use MB1303 shield offering         **
**  2 USBPD ports based on STUSB1602 + STM32F446ZE **
*****************************************************

Remove:
SB118
SB5
SB6
R37 (for 2nd port LEDS)
R38 (for 2nd port LEDs)

Close
SB144 (for 2nd port LEDs)
SB145 (for 2nd port LEDs)
Add:
38 pins connectors on both CN11 and CN12

For Trace:
add wire between PA9-CN9(pin21) of MB1303 and CN5-RX on Nucleo-MB1137