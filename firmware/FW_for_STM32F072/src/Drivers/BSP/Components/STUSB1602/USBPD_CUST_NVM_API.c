/*  **/
#include <stdint.h>
#include "User_BSP.h"
#include "STUSB1602_Peripherals_if.h"

#define FTP_CUST_PASSWORD_REG	0x95
#define FTP_CUST_PASSWORD		0x47
#define FTP_CTRL_0              0x96
#define FTP_CUST_PWR	0x80 
#define FTP_CUST_RST_N	0x40
#define FTP_CUST_REQ	0x10
#define FTP_CUST_SECT 0x07
#define FTP_CTRL_1              0x97
#define FTP_CUST_SER 0xF8
#define FTP_CUST_OPCODE 0x07
#define RW_BUFFER               0x53

extern STUSB1602_StatusTypeDef STUSB1602_ReadReg(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size);
extern STUSB1602_StatusTypeDef STUSB1602_WriteReg(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size);

/*"000" then No Operation
"001" then Read 
"010" and FTP_ADR[2:0]="000" then Shift-In Write Bit Data (0x20-0x28). (to be done before Programming)
"010" and FTP_ADR[2:0]="001" then Shift-In Erase Sector Data (0x20). (to be done before Erasing)
"011" and FTP_ADR[2:0]="000" then Shift-Out Read Bit Data (0x20-0x28). (to be done after Reading) 
"011" and FTP_ADR[2:0]="001" then Shift-Out Erase Sector Data (0x20). (to be done after Erasing) 
"100" then Verify (to be done after Programming)
"101" then Erase
"110" then Program
"111" then Soft Programming (to be done after Erasing)*/
#define READ 0x00
#define WRITE_PL 0x01
#define WRITE_SER 0x02
#define READ_PL	0x03
#define READ_SER 0x04
#define ERASE_SECTOR 0x05
#define PROG_SECTOR 0x06
#define SOFT_PROG_SECTOR 0x07

#define	SECTOR_0	0x01
#define	SECTOR_1	0x02
#define	SECTOR_2	0x04
#define	SECTOR_3	0x08
#define	SECTOR_4	0x10

typedef enum
{
  NVM_OKK       = 0x00,		/*!< Device OK 		*/
  NVM_ERROR    = 0x01,		/*!< Device ERROR 	*/
  NVM_BUSY     = 0x02,		/*!< Device BUSY 	*/
  NVM_TIMEOUT  = 0x03,		/*!< Device TIMEOUT */
  NVM_WRITTEN = 0x04   
} STUSB1602_NVMTypeDef;
uint8_t nvm_flash(uint8_t Addr);
uint8_t nvm_flash_recup(uint8_t Addr);

STUSB1602_StatusTypeDef CUST_EnterWriteMode(uint8_t Addr,unsigned char ErasedSector);
STUSB1602_StatusTypeDef CUST_EnterReadMode(uint8_t Addr);
STUSB1602_StatusTypeDef CUST_ReadSector(uint8_t Addr,char SectorNum, unsigned char *SectorData);
STUSB1602_StatusTypeDef CUST_WriteSector(uint8_t Addr,char SectorNum, unsigned char *SectorData);
STUSB1602_StatusTypeDef CUST_ExitTestMode(uint8_t Addr);

uint8_t nvm_flash(uint8_t Addr)
{
  STUSB1602_StatusTypeDef status =  STUSB1602_ERROR;
  uint8_t Write_flag=0;
  uint8_t sector0[8];
  uint8_t sector1[8];
  uint8_t sector2[8];
  uint8_t sector3[8];
  uint8_t sector4[8];
  
  /* output VVAR on VVAR_ADDR0 pin */	
  if ( CUST_EnterReadMode(Addr) != STUSB1602_OK ) return status;
  if ( CUST_ReadSector(Addr,0,&sector0[0])!= STUSB1602_OK ) return status;
  if ( CUST_ReadSector(Addr,1,&sector1[0])!= STUSB1602_OK ) return status;
  if ( CUST_ReadSector(Addr,2,&sector2[0])!= STUSB1602_OK ) return status;
  if ( CUST_ReadSector(Addr,3,&sector3[0])!= STUSB1602_OK ) return status;
  if ( CUST_ReadSector(Addr,4,&sector4[0])!= STUSB1602_OK ) return status;

#if !defined (_STUSB4761)  
  if (!(sector1[0]&&0x80) )
  {  
    sector1[0] = sector1[0] | 0x80 ; 
    if ( CUST_EnterWriteMode(Addr, SECTOR_1  )!= STUSB1602_OK ) return status;
    if ( CUST_WriteSector(Addr,1,&sector1[0])!= STUSB1602_OK ) return status;
    Write_flag = 1;
  }
#endif  
  /*enable Discharge on VBUS_EN_SNK while in SRC mode */	
#if defined (_SRC) && !defined (_DRP)
  if (!(sector4[6]&0x80) )
  {
    sector4[6] = sector4[6] | 0x80 ; 
    if ( CUST_EnterWriteMode(Addr, SECTOR_4  )!= STUSB1602_OK ) return status;
    if ( CUST_WriteSector(Addr,4,&sector4[0])!= STUSB1602_OK ) return status;
    Write_flag = 1;
  }
#elif defined (_STUSB4761)
  if ((sector4[5]& 0x91)!= 0x91 )
  {
    sector4[5] = sector4[5] | 0x91 ; /* Disable Monitoring and VDDtracking at startup */
    if ( CUST_EnterWriteMode(Addr, SECTOR_4  )!= STUSB1602_OK ) return status;
    if ( CUST_WriteSector(Addr,4,&sector4[0])!= STUSB1602_OK ) return status;
    Write_flag = 1;
  }  
#endif 
  if ( CUST_ExitTestMode(Addr)!= STUSB1602_OK ) return status;
  
  if (Write_flag )
    return 4 ;
  return status;
  
}
uint8_t nvm_flash_recup(uint8_t Addr)
{
  STUSB1602_StatusTypeDef status =  HAL_ERROR;
  uint8_t sector0[8]={0x00,0x00,0x39,0xAD,0x02,0x16,0x4F,0xC3};
  uint8_t sector1[8]={0x00,0xC0,0xAC,0x1F,0x75,0x71,0x14,0xE5};
  uint8_t sector2[8]={0x51,0x45,0x00,0x55,0x0B,0x33,0xC0,0x03};	
  uint8_t sector3[8]={0x19,0x3C,0x02,0x7F,0x70,0x55,0xE0,0x15};
  uint8_t sector4[8]={0x05,0x3C,0x00,0x00,0x00,0x01,0x00,0xF3};
  
  if ( CUST_EnterWriteMode(Addr, SECTOR_0|SECTOR_1|SECTOR_2|SECTOR_3|SECTOR_4)!= STUSB1602_OK ) return status;
  if ( CUST_WriteSector(Addr,0,sector0)!= STUSB1602_OK ) return status;
  if ( CUST_WriteSector(Addr,1,sector1)!= STUSB1602_OK ) return status;
  if ( CUST_WriteSector(Addr,2,sector2)!= STUSB1602_OK ) return status;
  if ( CUST_WriteSector(Addr,3,sector3)!= STUSB1602_OK ) return status;
  if ( CUST_WriteSector(Addr,4,sector4)!= STUSB1602_OK ) return status;
  if ( CUST_ExitTestMode(Addr)!= STUSB1602_OK ) return status;
    return 4  ;
  
}
/*
FTP Registers
o FTP_CUST_PWR (0x9E b(7), ftp_cust_pwr_i in RTL); power for FTP
o FTP_CUST_RST_N (0x9E b(6), ftp_cust_reset_n_i in RTL); reset for FTP
o FTP_CUST_REQ (0x9E b(4), ftp_cust_req_i in RTL); request bit for FTP operation
o FTP_CUST_SECT (0x9F (2:0), ftp_cust_sect1_i in RTL); for customer to select between sector 0 to 4 for read/write operations (functions as lowest address bit to FTP, remainders are zeroed out)
o FTP_CUST_SER[4:0] (0x9F b(7:4), ftp_cust_ser_i in RTL); customer Sector Erase Register; controls erase of sector 0 (00001), sector 1 (00010), sector 2 (00100), sector 3 (01000), sector 4 (10000) ) or all (11111).
o FTP_CUST_OPCODE[2:0] (0x9F b(2:0), ftp_cust_op3_i in RTL). Selects opcode sent to
o RW_BUFFER (0x53 to 0x5A ) 64 bit Read Write buffer 
FTP. Customer Opcodes are:
o 000 = Read sector
o 001 = Write Program Load register (PL) with data to be written to sector 0 or 1
o 010 = Write Sector Erase Register (SER) with data reflected by state of FTP_CUST_SER[4:0]
o 011 = Read Program Load register (PL)
o 100 = Read SER;
o 101 = Erase sector 0 to 4  (depending upon the mask value which has been programmed to SER)
o 110 = Program sector 0  to 4 (depending on FTP_CUST_SECT1)
o 111 = Soft program sector 0 to 4 (depending upon the value which has been programmed to SER)*/


/* For All function Port is pointing to a structure that give I2C port number(1 or 2) and device adresses 7bit wise (0x28 , 0x29 ....)*/
/***************************  void CUST_EnterWriteMode(uint8_t Port,unsigned char ErasedSector)  ***************************/
/*this function prepare the flash to be wirtten  by erasing the sectors*/

/************************************************************************************/

STUSB1602_StatusTypeDef CUST_EnterWriteMode(uint8_t Addr,unsigned char ErasedSector)
{
  uint8_t Buffer[10];
  
  Buffer[0]=FTP_CUST_PASSWORD;   /* Set Password*/
  if( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CUST_PASSWORD_REG, 1) != STUSB1602_OK ) return STUSB1602_ERROR;
  
  Buffer[0]= 0 ;   /* this register must be NULL for Partial Erase feature */
   if( STUSB1602_WriteReg(&Buffer[0], Addr, RW_BUFFER, 1) != STUSB1602_OK ) return STUSB1602_ERROR;

  Buffer[0]=0; /* Reset PWR and RST_N bits to cycle NVM power supply*/
   if( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR;
  
  
  Buffer[0]=FTP_CUST_PWR | FTP_CUST_RST_N; /* Set PWR and RST_N bits */
  if( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR;
  Buffer[0]=((ErasedSector << 3) & FTP_CUST_SER) | ( WRITE_SER & FTP_CUST_OPCODE) ;  /* Load 0xF1 to erase all sectors of FTP and Write SER Opcode */
  if( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_1, 1) != STUSB1602_OK ) return STUSB1602_ERROR; /* Set Write SER Opcode */
  
  Buffer[0]=FTP_CUST_PWR | FTP_CUST_RST_N | FTP_CUST_REQ ; 
  if( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR; /* Load Write SER Opcode */
  do 
  {
    STUSB1602_ReadReg(&Buffer[0], Addr, FTP_CTRL_0, 1); /* Wait for execution */
  }
  while(Buffer[0] & FTP_CUST_REQ); 
  Buffer[0]=  SOFT_PROG_SECTOR & FTP_CUST_OPCODE ;  
  if( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_1, 1) != STUSB1602_OK ) return STUSB1602_ERROR;  /* Set Soft Prog Opcode */
  
  Buffer[0]=FTP_CUST_PWR | FTP_CUST_RST_N | FTP_CUST_REQ ; 
  if( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR; /* Load Soft Prog Opcode */
  
  do 
  {
    STUSB1602_ReadReg(&Buffer[0], Addr, FTP_CTRL_0, 1); /* Wait for execution */
  }
  while(Buffer[0] & FTP_CUST_REQ);
  Buffer[0]= ERASE_SECTOR & FTP_CUST_OPCODE ;  
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_1, 1) != STUSB1602_OK ) return STUSB1602_ERROR; /* Set Erase Sectors Opcode */
  
  Buffer[0]=FTP_CUST_PWR | FTP_CUST_RST_N | FTP_CUST_REQ ; 
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR; /* Load Erase Sectors Opcode */
  
  do 
  {
    STUSB1602_ReadReg(&Buffer[0], Addr, FTP_CTRL_0, 1); /* Wait for execution */
  }
  while(Buffer[0] & FTP_CUST_REQ);	
  
  return STUSB1602_OK;
}


/***************************  void CUST_EnterReadMode(uint8_t Port)  ***************************
this function prepare the flash to be red  
setting power enable and flash reset 
************************************************************************************/
STUSB1602_StatusTypeDef CUST_EnterReadMode(uint8_t Addr)
{
  unsigned char Buffer[10];
  
  Buffer[0]=FTP_CUST_PASSWORD;  /* Set Password*/
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CUST_PASSWORD_REG, 1) != STUSB1602_OK ) return STUSB1602_ERROR;
  
  Buffer[0]= FTP_CUST_PWR |FTP_CUST_RST_N ;
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR;	
  Buffer[0]= 0;
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR;
  return STUSB1602_OK;
}

/***************************  void CUST_ReadSector(uint8_t Port,char SectorNum, unsigned char *SectorData) ***************************
this function read sectors number SectorNum  and put it in SectorData pointer 
************************************************************************************/
STUSB1602_StatusTypeDef CUST_ReadSector(uint8_t Addr,char SectorNum, unsigned char *SectorData)
{
  unsigned char Buffer[10];
  
  Buffer[0]= FTP_CUST_PWR |FTP_CUST_RST_N ;
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR;	

  Buffer[0]= (SectorNum & FTP_CUST_SECT) | FTP_CUST_PWR |FTP_CUST_RST_N ;
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR;
  
  Buffer[0]= (READ & FTP_CUST_OPCODE);
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_1, 1) != STUSB1602_OK ) return STUSB1602_ERROR;/* Set Read Sectors Opcode */
  Buffer[0]= (SectorNum & FTP_CUST_SECT) |FTP_CUST_PWR |FTP_CUST_RST_N | FTP_CUST_REQ;
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR;  /* Load Read Sectors Opcode */
  do 
  {
    STUSB1602_ReadReg(&Buffer[0], Addr, FTP_CTRL_0, 1); /* Wait for execution */
  }
  while(Buffer[0] & FTP_CUST_REQ);
  STUSB1602_ReadReg(&SectorData[0], Addr, RW_BUFFER, 8);
  
  Buffer[0] = 0 ;
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR;
  
  return STUSB1602_OK;
}

/***************************  void CUST_WriteSector(uint8_t Port,char SectorNum, unsigned char *SectorData) ***************************
this function write Data's in SectorData pointer to  sectors number SectorNum  
************************************************************************************/
STUSB1602_StatusTypeDef CUST_WriteSector(uint8_t Addr,char SectorNum, unsigned char *SectorData)
{
  unsigned char Buffer[10];
  
  
  if ( STUSB1602_WriteReg(SectorData, Addr, RW_BUFFER, 8) != STUSB1602_OK ) return STUSB1602_ERROR;
  Buffer[0]=FTP_CUST_PWR | FTP_CUST_RST_N; /*Set PWR and RST_N bits*/
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR;
  Buffer[0]= (WRITE_PL & FTP_CUST_OPCODE); /*Set Write to PL Opcode*/
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_1, 1) != STUSB1602_OK ) return STUSB1602_ERROR;
  Buffer[0]=FTP_CUST_PWR |FTP_CUST_RST_N | FTP_CUST_REQ;  /* Load Write to PL Sectors Opcode */  
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR;
  
  do 
  {
    STUSB1602_ReadReg(&Buffer[0], Addr, FTP_CTRL_0, 1); /* Wait for execution */
  }		 
  while(Buffer[0] & FTP_CUST_REQ) ;
  
  
  Buffer[0]= (PROG_SECTOR & FTP_CUST_OPCODE);
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_1, 1) != STUSB1602_OK ) return STUSB1602_ERROR;/*Set Prog Sectors Opcode*/
  Buffer[0]=(SectorNum & FTP_CUST_SECT) |FTP_CUST_PWR |FTP_CUST_RST_N | FTP_CUST_REQ;
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 1) != STUSB1602_OK ) return STUSB1602_ERROR; /* Load Prog Sectors Opcode */  
  do 
  {
    STUSB1602_ReadReg(&Buffer[0], Addr, FTP_CTRL_0, 1); /* Wait for execution */
  }
  while(Buffer[0] & FTP_CUST_REQ) ;
  return STUSB1602_OK;
}

/***************************  void CUST_ExitTestMode(uint8_t Port) ***************************
this function put device in operating mode  
************************************************************************************/
STUSB1602_StatusTypeDef CUST_ExitTestMode(uint8_t Addr)
{
  unsigned char Buffer[10];
  
  Buffer[0]= FTP_CUST_RST_N; Buffer[1]=0x00;  /* clear registers */
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CTRL_0, 2) != STUSB1602_OK ) return STUSB1602_ERROR;
  Buffer[0]=0x00; 
  if ( STUSB1602_WriteReg(&Buffer[0], Addr, FTP_CUST_PASSWORD_REG, 1) != STUSB1602_OK ) return STUSB1602_ERROR;  /* Clear Password */
  return STUSB1602_OK;
}



