*****************************************************
**  Hardware modifications to be done on MB1367C    **
**  in order to use MB1303 shield offering          **
**  2 USBPD ports based on STUSB1602 + STM32G474RE  **
*****************************************************

Remove:
SB17
SB23
SB13
SB19

strap 
SB18 

solder wire between T_VCP_TX from R23 and CN10-pin21 
solder wire betwwen T_VCP_RX from R22 and CN10-pin33
connect wire between CN10-1 and CN10-18
connect wire between CN10-2 and CN10-25

LED for Port 2
remove 
SB30
SB31
strap 
SB29
SB32
