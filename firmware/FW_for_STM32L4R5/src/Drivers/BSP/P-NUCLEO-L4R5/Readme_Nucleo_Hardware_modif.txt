*****************************************************
**  Hardware modifications to be done on MB1312    **
**  in order to use MB1303 shield offering         **
**  2 USBPD ports based on STUSB1602 + stm32L4R5ZI **
*****************************************************

Remove:
SB124
SB132
SB130
SB141
SB131
R39 (for 2nd port LEDS)
R40 (for 2nd port LEDs)

Close
SB145 (for 2nd port LEDs)
SB146 (for 2nd port LEDs)
Add:
38 pins connectors on both CN11 and CN12

For Trace:
connect wires between MB1303 CN2 and Nucleo CN6 (Tx-Rx)