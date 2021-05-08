/**
  ******************************************************************************
  * @file    cli_commands.c
  * @author  System Lab
  * @brief   CLI Commands defintion and implementation.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
#include "cli_api.h"

#include "usbpd_core.h"


#ifdef USBPD_CLI

/* Includes ------------------------------------------------------------------*/
#include "usbpd_trace.h"
#include "cli_commands.h"
#include "usbpd_def.h"
#include "usbpd_dpm_user.h"
#include "usbpd_cad_hw_if.h"
#include "usbpd_pwr_if.h"
#include "usbpd_pdo_defs.h"
#if USBPD_PORT_COUNT == 1 
#define PORT_PARAM_ENABLE 0
#else
#define PORT_PARAM_ENABLE 1
#endif  /* USBPD_PORT_COUNT */
#define USBPD_DEF_PORT 0
/* External vars -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MSG_NOT_IMPLEMENTED " not implemented!\r\n"
#define CLI_PORTNUM_INVALID 0xFF


USBPD_CLITypeDef CLI_APDO_params =
{
  .CLI_APDO_Voltagemin= 3400,
  .CLI_APDO_Voltagemax= 5900,
  .CLI_APDO_Voltagestep= 20,
  .CLI_APDO_runningvalue = 3400,
};

extern USBPD_USER_SettingsTypeDef  DPM_USER_Settings[USBPD_PORT_COUNT];
extern USBPD_PWR_Port_PDO_Storage_TypeDef PWR_Port_PDO_Storage[USBPD_PORT_COUNT];

/* Private variables ---------------------------------------------------------*/
/** @defgroup CLI_Commands_Private_Variable CLI Commands Private Variables
 * @{
 */
/** 
 * brief  handle of the thread
 */
osThreadId xCmdThreadId;

/** 
 * brief  receive in this queue the command (without endline)
 */
static osMessageQId xQueueIn = NULL;
/** 
 * brief  send to this queue the output generated
 */
static osMessageQId xQueueOut = NULL;

/** 
 * brief  Buffer for the input string
 */
static char cInputString[ CLI_INPUT_MAX_SIZE ];
/** 
 * 
 */
static uint32_t cSNKPDONum = 0;
static uint32_t aSNKPDOBuffer[USBPD_MAX_NB_PDO];
static uint8_t cSNKPDOType = 0;
static uint32_t cSRCPDONum = 0;
static uint32_t aSRCPDOBuffer[USBPD_MAX_NB_PDO];
static uint8_t cSRCPDOType = 0;
/**
 * @}
 */

/* Private function prototypes -----------------------------------------------*/
static void prvCommandThread( void const * argument );
#if PORT_PARAM_ENABLE == 1
static portCHAR prvCommandCheckPortNumber(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
#endif /* PORT_PARAM_ENABLE */
static inline void prvGetVoltageCurrentFromPDO(uint32_t PdoValue, float *pVoltage, float *pCurrent);
static inline void prvGetVoltageCurrentFromSNKAPDO(uint32_t PdoValue, float *pVoltage, float *pVoltagemin,float *pVoltagemax, float *pCurrent);
static inline void prvGetVoltageCurrentFromSRCAPDO(uint32_t PdoValue, float *pVoltage, float *pVoltagemin,float *pVoltagemax, float *pCurrent);

static BaseType_t prvWelcomeCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvProfilesCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvStatusCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvRequestCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvPRSwapCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvAPDOreqCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/* Commands structure definition ---------------------------------------------*/
/** @defgroup CLI_Commands_Definition CLI Commands Definition
 * @{
 */
/** 
 * brief  Welcome command definition
 */
static const CLI_Command_Definition_t xWelcomeCommand =
{
  "welcome",
  "w | welcome : Print out the welcome message\r\n",
  prvWelcomeCommandFunc, /* The function to run. */
  0 /* No parameter is expected. */
};
static const CLI_Command_Definition_t xWelcomeCommand2 =
{
  "w",
  "",
  prvWelcomeCommandFunc, /* The function to run. */
  0 /* No parameter is expected. */
};
/** 
 * brief  Profiles command definition
 */
static const CLI_Command_Definition_t xProfilesCommand =
{
  "profiles",
#if PORT_PARAM_ENABLE == 1        
  "p | profiles <port>: show the available profiles for the port\r\n",
#else
  "p | profiles : show the available profiles \r\n",
#endif /* PORT_PARAM_ENABLE */
  prvProfilesCommandFunc, /* The function to run. */
  PORT_PARAM_ENABLE 
};
static const CLI_Command_Definition_t xProfilesCommand2 =
{
  "p",
  "",
  prvProfilesCommandFunc, /* The function to run. */
  PORT_PARAM_ENABLE 
};

/** 
 * brief  Status command definition
 */
static const CLI_Command_Definition_t xStatusCommand =
{
  "status",
#if PORT_PARAM_ENABLE == 1        
  "s | status <port> : show the status of the PD comm for the port\r\n",
#else
  "s | status : show the status of the PD comm\r\n",
#endif /* PORT_PARAM_ENABLE */
  prvStatusCommandFunc, /* The function to run. */
  PORT_PARAM_ENABLE 
};
static const CLI_Command_Definition_t xStatusCommand2 =
{
  "s",
  "",
  prvStatusCommandFunc, /* The function to run. */
  PORT_PARAM_ENABLE 
};
/** 
 * brief  Request command definition
 */
static const CLI_Command_Definition_t xRequestCommand =
{
  "request",
#if PORT_PARAM_ENABLE == 1        
  "r | request <port> <profile> : change PD profile (only sink) for the port\r\n",
#else
  "r | request <profile> : change PD profile (only consumer)\r\n",
#endif /* PORT_PARAM_ENABLE */
  prvRequestCommandFunc, /* The function to run. */
  (PORT_PARAM_ENABLE+1) 
};
static const CLI_Command_Definition_t xRequestCommand2 =
{
  "r",
  "",
  prvRequestCommandFunc, /* The function to run. */
  (PORT_PARAM_ENABLE+1) 
};

/** 
 * brief  Request command definition
 */
static const CLI_Command_Definition_t xPRSwapCommand =
{
  "prswap",
#if PORT_PARAM_ENABLE == 1        
  "x | prswap <port> : start a power role swap for the port\r\n",
#else
  "x | prswap : start a power role swap\r\n",
#endif /* PORT_PARAM_ENABLE */
  prvPRSwapCommandFunc, /* The function to run. */
  PORT_PARAM_ENABLE 
};

static const CLI_Command_Definition_t xPRSwapCommand2 =
{
  "x",
  "",
  prvPRSwapCommandFunc, /* The function to run. */
  PORT_PARAM_ENABLE 
};

///ajout APDO ///
static const CLI_Command_Definition_t xAPDOCommand =
{
  "apdo",
#if PORT_PARAM_ENABLE == 1        
  "apdo  <port> : APDO setting for next request for the port\r\n",
#else
  "apdo <APDOmin> <APDOmax> <step> : APDO setting for next request for the port\r\n",
#endif /* PORT_PARAM_ENABLE */
  prvAPDOreqCommandFunc, /* The function to run. */
  3 /* need to enter 3 parameters  */
};

//static const CLI_Command_Definition_t xAPDOCommand2 =
//{
//  "APDO",
//  "",
//  prvAPDOreqCommandFunc, /* The function to run. */
//  PORT_PARAM_ENABLE 
//};

//////
/**
 * @}
 */

/**
 * @brief  This function registers the commands in the FreeRTOS-CLI.
 */
void CLI_RegisterCommands( void )
{
  /* Register all the command line commands defined immediately above. */
  FreeRTOS_CLIRegisterCommand( &xWelcomeCommand );
  FreeRTOS_CLIRegisterCommand( &xProfilesCommand );
  FreeRTOS_CLIRegisterCommand( &xStatusCommand );
  FreeRTOS_CLIRegisterCommand( &xRequestCommand );
  FreeRTOS_CLIRegisterCommand( &xPRSwapCommand );
  FreeRTOS_CLIRegisterCommand( &xAPDOCommand );

  FreeRTOS_CLIRegisterCommand( &xWelcomeCommand2 );
  FreeRTOS_CLIRegisterCommand( &xProfilesCommand2 );
  FreeRTOS_CLIRegisterCommand( &xStatusCommand2 );
  FreeRTOS_CLIRegisterCommand( &xRequestCommand2 );
  FreeRTOS_CLIRegisterCommand( &xPRSwapCommand2 );
//  FreeRTOS_CLIRegisterCommand( &xAPDOCommand2 );

}

/** @defgroup CLI_Commands_Callbacks CLI Commands Callbacks
 * @{
 */

/**
 * @brief  CLI callback for the Welcome command.
 */
static BaseType_t prvWelcomeCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* To storage the status of the send */
  static unsigned char cStep = 0;
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
        
  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );

  /* reset the count */
  if (cStep >= CLI_WELCOME_MESSAGE_LEN) 
  {
    cStep = 0;
  }
  
  /* Copy welcome string in the output */
  sprintf( pcWriteBuffer, CLI_ConfigWelcomeMessage[cStep++]);
  return cStep < CLI_WELCOME_MESSAGE_LEN ? pdTRUE : pdFALSE; /* to manage the iteraction */

}

/**
 * @brief  CLI callback for the profiles command.
 */
static BaseType_t prvProfilesCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* local variables definition */
  static unsigned char cPort = USBPD_DEF_PORT;
  static unsigned char cSection = 0xFF; /* 0 as source, 1 as sink */
  static unsigned char cIndex = 0xFF;

  static int8_t cConnectionStatus = 0;
  static USBPD_PortPowerRole_TypeDef cRole = USBPD_PORTPOWERROLE_SNK;
  static uint8_t cCondAsSink = 0;
  float voltage = 0, current = 0;
  uint8_t cDrpSupport = USBPD_FALSE;
  
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
        
  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );

#if PORT_PARAM_ENABLE == 1
  /* check the port parameter */
  cPort = prvCommandCheckPortNumber(pcWriteBuffer, xWriteBufferLen, pcCommandString);
  if (cPort == CLI_PORTNUM_INVALID)
  {
    /* in case of a wrong PortNumber complete the command */
    return pdFALSE;
  }
#endif /* PORT_PARAM_ENABLE */
  
  /* get the available profiles */
  if (cSection == 0xFF)
  {
    /* show the pdo message type provider / consumer */
    cIndex = 0xFF; /* used to print the title */
    cSection = 0;
    cConnectionStatus = 0;

    USBPD_DPM_CLI_GetCurrentRole(cPort, &cRole, &cConnectionStatus, &cDrpSupport);
    cCondAsSink = (cRole == USBPD_PORTPOWERROLE_SNK);
    
    /* get the profiles according to the connection status */
    switch(cConnectionStatus)
    {
    case 0: /* Unplugged */
      if (cDrpSupport == USBPD_TRUE)
      {
        strcpy(pcWriteBuffer, "DRP role Unplugged\r\n");
        cSRCPDOType = 1; /* MY_SRC_PDO */
        cSNKPDOType = 1; /* MY_SNK_PDO */        
      }
      else
      {
        strcpy(pcWriteBuffer, cCondAsSink ? "Sink" : "Source");
        strcat(pcWriteBuffer, " role Unplugged\r\n");
        cSRCPDOType = cCondAsSink ? 0 : 1; /* NO_SRC_PDO : MY_SRC_PDO */
        cSNKPDOType = cCondAsSink ? 1 : 0; /* MY_SNK_PDO : NO_SNK_PDO */
      }
      break;

    case 1: /* Default 5v, Type-C only */
      strcpy(pcWriteBuffer, cCondAsSink ? "Sink" : "Source");
      strcat(pcWriteBuffer, " role Type-C only\r\n");
      cSRCPDOType = cCondAsSink ? 0 : 1; /* MY_SRC_PDO */
      cSNKPDOType = cCondAsSink ? 1 : 0; /* MY_SNK_PDO */
      break;
    case 2: /* Implicit Contract Done */
      strcpy(pcWriteBuffer, cCondAsSink ? "Sink" : "Source");
      strcat(pcWriteBuffer, " role - Implicit contract\r\n");
        /* as Source in explicit contract it shows the MY_SRC_PDO and RX_SNK_PDO */
      cSRCPDOType = cCondAsSink ? 2 : 1; /* RX_SRC_PDO : MY_SRC_PDO */
      cSNKPDOType = cCondAsSink ? 1 : 2; /* MY_SNK_PDO : RX_SNK_PDO */
      break;
    case 3: /* Explicit Contract Done */
      strcpy(pcWriteBuffer, cCondAsSink ? "Sink" : "Source");
      strcat(pcWriteBuffer, " role - Explicit contract\r\n");
        /* as Source in explicit contract it shows the MY_SRC_PDO and RX_SNK_PDO */
      cSRCPDOType = cCondAsSink ? 2 : 1; /* RX_SRC_PDO : MY_SRC_PDO */
      cSNKPDOType = cCondAsSink ? 1 : 2; /* MY_SNK_PDO : RX_SNK_PDO */
      break;
    default:
      strcpy( pcWriteBuffer, "Error: unknown connection status\r\n");
      cSRCPDOType = 1; /* MY_SRC_PDO */
      cSNKPDOType = 1; /* MY_SNK_PDO */
      break;
    }

    cSRCPDONum = 0;
    memset(aSRCPDOBuffer, 0x00, sizeof(aSRCPDOBuffer));
    if (cSRCPDOType > 0)
    {
      USBPD_DPM_GetDataInfo(cPort, cSRCPDOType == 1 ? USBPD_CORE_DATATYPE_SRC_PDO : USBPD_CORE_DATATYPE_RCV_SRC_PDO, (uint8_t *)aSRCPDOBuffer, &cSRCPDONum);
      if (cSRCPDONum) cSRCPDONum /=  4 ; // DPM_Ports[cPort].DPM_NumberOfRcvSRCPDO;
    }
    cSNKPDONum = 0;
    memset(aSNKPDOBuffer, 0x00, sizeof(aSNKPDOBuffer));
    if (cSNKPDOType > 0)
    {
      USBPD_DPM_GetDataInfo(cPort, cSNKPDOType == 1 ? USBPD_CORE_DATATYPE_SNK_PDO : USBPD_CORE_DATATYPE_RCV_SNK_PDO, (uint8_t*)aSNKPDOBuffer, &cSNKPDONum);
      
      if (cSNKPDONum)cSNKPDONum /= 4; //DPM_Ports[cPort].DPM_NumberOfRcvSNKPDO;
    }

    return cSection == 0xFF ? pdFALSE : pdTRUE; //continue if ok
  } /* PDO calculation */

  if (cSection == 0)
  {
    /* print all my/rx source PDOs */
    strcpy(pcWriteBuffer, "");
    /* if 0 skip and move in the next section */
    if (cSRCPDOType != 0) 
    {
      /* first time print the title */
      if (cIndex == 0xFF)
      {
        strcpy(pcWriteBuffer, cSRCPDOType == 1 ? "Local" : "Received");
        strcat(pcWriteBuffer, " Source PDOs:\r\n");
        if (cSRCPDONum == 0)
        {
          strcat(pcWriteBuffer, " no PDO available\r\n");
        }
        else
        {
          cIndex = 0x00;
        }
      }
      else 
      {
        /* check if there are PDOs to print */
        if (cIndex >= cSRCPDONum || cIndex >= USBPD_MAX_NB_PDO)
        {
          /* completed */
          cIndex = 0xFF;
        }
        else
        { 
          USBPD_PDO_TypeDef  pdosrc;
            pdosrc.d32 = DPM_Ports[0].DPM_ListOfRcvSRCPDO[cIndex];
            if (pdosrc.GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_FIXED)
            {
          /* print the current cIndex */
          prvGetVoltageCurrentFromPDO(aSRCPDOBuffer[cIndex], &voltage, &current);
//           prvGetVoltageCurrentFromPDO(DPM_Ports[0].DPM_ListOfRcvSRCPDO[cIndex], &voltage, &current);   
          sprintf( pcWriteBuffer, "%d) %d.%.2dV %d.%.2dA\r\n", cIndex+1,
                   (uint16_t)(voltage/1000), ((uint16_t)((uint16_t)(voltage/10)%100)),
                   (uint16_t)(current/1000), ((uint16_t)((uint16_t)(current/10)%100)));
            }
            else
              if (pdosrc.GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_APDO)
              {
                          /* print the current cIndex */
          float voltage = 0, current = 0, voltagemin = 0, voltagemax =0;
          prvGetVoltageCurrentFromSRCAPDO(aSRCPDOBuffer[cIndex], &voltage, &voltagemin, &voltagemax,&current);
            sprintf( pcWriteBuffer, "%d) APDO: Min: %dmV Max: %dmV Current: %dmA\r\n", cIndex+1,
                   (uint16_t)(voltagemin), (uint16_t)(voltagemax),
                   (uint16_t)(current));

              }
          cIndex++;
        }
      }
    }
    
    if (cIndex == 0xFF)
    {
      /* move to next section*/
      cSection++;
    }
  }
  else if (cSection == 1)
  {
    /* print all my/rx sink PDOs */
    strcpy(pcWriteBuffer, "");
    /* if 0 skip and move in the next section */
    if (cSNKPDOType != 0) 
    {
      /* first time print the title */
      if (cIndex == 0xFF)
      {
        strcpy(pcWriteBuffer, cSNKPDOType == 1 ? "Local" : "Received");
        strcat(pcWriteBuffer, " Sink PDOs:\r\n");
        if (cSNKPDONum == 0)
        {
          strcat(pcWriteBuffer, " no PDO available\r\n");
        }
        else
        {
          cIndex = 0x00;
        }
      }
      else 
      {
        /* check if there are PDOs to print */
        if (cIndex >= cSNKPDONum || cIndex >= USBPD_MAX_NB_PDO)
        {
          /* completed */
          cIndex = 0xFF;
        }
        else
        {
          USBPD_PDO_TypeDef  rdosnk;
          rdosnk.d32 = aSNKPDOBuffer[cIndex];  
            if (rdosnk.GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_FIXED)
            {
          /* print the current cIndex */
          float voltage = 0, current = 0;
          prvGetVoltageCurrentFromPDO(aSNKPDOBuffer[cIndex], &voltage, &current);
          sprintf( pcWriteBuffer, "%d) %d.%.2dV %d.%.2dA\r\n", cIndex+1,
                   (uint16_t)(voltage/1000), ((uint16_t)((uint16_t)(voltage/10)%100)),
                   (uint16_t)(current/1000), ((uint16_t)((uint16_t)(current/10)%100)));
            }
           else
              if (rdosnk.GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_APDO)
              {
          /* print the current cIndex */
          float voltage = 0, current = 0, voltagemin = 0, voltagemax =0;
          prvGetVoltageCurrentFromSNKAPDO(aSNKPDOBuffer[cIndex], &voltage, &voltagemin, &voltagemax,&current);
            sprintf( pcWriteBuffer, "%d) APDO: Min: %dmV Max: %dmV Current: %dmA\r\n", cIndex+1,
                   (uint16_t)(voltagemin), (uint16_t)(voltagemax),
                   (uint16_t)(current));
          }
          cIndex++;

          }
        }
      }
  //  }
    
    if (cIndex == 0xFF)
    {
      /* move to next section*/
      cSection++;
    }    
  }
  else
  {
    /* stop */
    cIndex = 0xFF;
    cSection = 0xFF;
    strcpy(pcWriteBuffer, "");
  }
  
  return cSection == 0xFF ? pdFALSE : pdTRUE;
}

/**
 * @brief  CLI callback for the status command.
 */

#define CONNSTATUSTEXT_COUNT 6
#define CONNSTATUSTEXT_SAFEINDEX(index) (((index) >= 0 && (index)<CONNSTATUSTEXT_COUNT) ? index : 0)
static char const * connStatusText[CONNSTATUSTEXT_COUNT] = {
  "Status unknown",
  "Unplugged",
  "Default 5V",
  "Implicit contract",
  "Explicit contract",
  "Power Transition",
};
#define CURRROLETEXT_COUNT 4
static char const * currRoleText[CURRROLETEXT_COUNT] = {
  "Unknown",
  "Sink",
  "Source",
  "Dual Role Port",
};
static inline uint8_t prvPortPowerRole2Index(USBPD_PortPowerRole_TypeDef PortPowerRole)
{
  uint8_t ret = 0;
  switch(PortPowerRole)
  {
  case USBPD_PORTPOWERROLE_SNK:
    ret = 1;
    break;
  case USBPD_PORTPOWERROLE_SRC:
    ret = 2;
    break;
  //case USBPD_PORTPOWERROLE_DRP:
//  case USBPD_PORTPOWERROLE_DRP_SNK:
//  case USBPD_PORTPOWERROLE_DRP_SRC:
//    ret = 3;
//    break;
  default:
    ret = 0;
    break;
  }
  return ret;
}

static BaseType_t prvStatusCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* local variables definition */
  static portCHAR cPort = USBPD_DEF_PORT;
  static int8_t connStatus = -1;

  USBPD_PortPowerRole_TypeDef currRole = USBPD_CABLEPLUG_FROMDFPUFP;
  CCxPin_TypeDef  cc = CCNONE;
  float voltage = 0;
  uint8_t profile = 0;
  uint8_t cDrpSupport = USBPD_FALSE;
  USBPD_StatusTypeDef res;
  
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
        
  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );
  
  if (connStatus == -1)
  {
#if PORT_PARAM_ENABLE == 1  
  /* check the port parameter */
    cPort = prvCommandCheckPortNumber(pcWriteBuffer, xWriteBufferLen, pcCommandString);
    /* in case of a wrong PortNumber this command */
    if (cPort == CLI_PORTNUM_INVALID)
    {
      /* reset parameter */
      connStatus = -1;
      cPort = USBPD_DEF_PORT;
      
      /* stop next call */
      return pdFALSE;
    }
#endif /* PORT_PARAM_ENABLE */

    /* print the connection status and role and eventually the value */
    USBPD_DPM_CLI_GetCurrentRole((uint8_t)cPort, &currRole, &connStatus, &cDrpSupport);
    if (connStatus == 0) /* unplugged */
    {
      sprintf( pcWriteBuffer, "Role: %s - %s", 
               cDrpSupport == USBPD_TRUE ? currRoleText[3] : currRoleText[prvPortPowerRole2Index(currRole)], 
               connStatusText[CONNSTATUSTEXT_SAFEINDEX(connStatus+1)]
             );
    }
    else
    {
      sprintf( pcWriteBuffer, "Role: %s - %s", 
               currRoleText[prvPortPowerRole2Index(currRole)], 
               connStatusText[CONNSTATUSTEXT_SAFEINDEX(connStatus+1)]
             );
    }
  }
  else
  {
    /* Get the CC line */
    cc = USBPD_DPM_CLI_GetCCLine(cPort);

    /* manage different status */
    switch(connStatus)
    {
    case 0: /* unplugged */
      strcpy(pcWriteBuffer, "");
      break;
    case 1: /* Default 5V, Type-C only */
      sprintf(pcWriteBuffer, " CC%d", (int)cc);
      break;
    case 2: /* Implicit contract done */
    case 3: /* Explicit contract done */
      res = USBPD_DPM_CLI_GetStatusInfo(cPort, &profile, &voltage, NULL);
      if (res == USBPD_OK)
      {
        sprintf(pcWriteBuffer, " CC%d Profile %d %d.%.2dV", (int)cc, profile, (uint16_t)voltage, ((uint16_t)((uint16_t)(voltage*100)%100)));
      }
      else
      {
        sprintf(pcWriteBuffer, " CC%d", (int)cc);
      }
      break;
    default: /* Unknown */
      sprintf(pcWriteBuffer, " CC%d Unknown", (int)cc);
      break;
    }
    strcat(pcWriteBuffer, "\r\n");
    connStatus = -1;
  }

  return connStatus == -1 ? pdFALSE : pdTRUE;
}

/**
 * @brief  CLI callback for the request command.
 */
static BaseType_t prvRequestCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* local variables definition */
  static portCHAR cPort = USBPD_DEF_PORT;
  static portCHAR cPDOIndex = 0;
  static int8_t connStatus = -1;

  const char *pcParameter = NULL;
  BaseType_t xParameterStringLength;
  USBPD_PortPowerRole_TypeDef currRole = USBPD_CABLEPLUG_FROMDFPUFP;
  uint8_t cDrpSupport = USBPD_FALSE;
  
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
  ( void ) pcParameter;

  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );
        
  /* 
    Get parameters 
    if the user specify all parameters the command is a set (for the blink mode need also the period)
    otherwise the command is a get
  */
  
#if PORT_PARAM_ENABLE == 1
  /* check the port parameter */
  cPort = prvCommandCheckPortNumber(pcWriteBuffer, xWriteBufferLen, pcCommandString);
  if (cPort == CLI_PORTNUM_INVALID)
  {
    /* in case of a wrong PortNumber complete the command */
    return pdFALSE;
  }
#endif /* PORT_PARAM_ENABLE */

  USBPD_DPM_CLI_GetCurrentRole((uint8_t)cPort, &currRole, &connStatus, &cDrpSupport);
  
  /* check if the cable is plugged and a contract is reached */
  if (connStatus != 3)
  {
    strcpy(pcWriteBuffer, "Request failed: no contract reached.\r\n");
    return pdFALSE;
  }

  /* the current role must be a Sink */
  if (currRole != USBPD_PORTPOWERROLE_SNK)
  {
    strcpy(pcWriteBuffer, "Request failed: allowed only for sink.\r\n");
    return pdFALSE;
  }  

  /* get the received source capabilities */
  cSRCPDONum = 0;
  memset(aSRCPDOBuffer, 0x00, sizeof(aSRCPDOBuffer));
  USBPD_DPM_GetDataInfo(cPort, USBPD_CORE_DATATYPE_RCV_SRC_PDO, (uint8_t *)aSRCPDOBuffer, &cSRCPDONum);
  cSRCPDONum /= 4;

  /* check if there is almost one source capability */
  if (cSRCPDONum == 0)
  {
    strcpy(pcWriteBuffer, "Request failed: no source caps available.\r\n");
    return pdFALSE;
  }  

  /* get the pdo index required through the command */
#if PORT_PARAM_ENABLE == 1
  pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);  
#else
  pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);  
#endif
  cPDOIndex = atoi(pcParameter);
  
  /* the current role must be a Sink */
  if (cPDOIndex == 0 || cPDOIndex > cSRCPDONum)
  {
    sprintf(pcWriteBuffer, "Request failed: wrong index param '%s' specified.\r\n", pcParameter);
    return pdFALSE;
  }  

//  uint32_t pdo_selected;
//  pdo_selected = cPDOIndex;
  /* send the command through the DPM API */  
  USBPD_StatusTypeDef res = USBPD_DPM_RequestNewPowerProfile(cPort, cPDOIndex);
// USBPD_StatusTypeDef res = USBPD_DPM_EvaluateRequest(cPort, (uint8_t)cPDOIndex );
  if (res != USBPD_OK)
  {
    strcpy(pcWriteBuffer, "Request failed: error on command execution.\r\n");
    return pdFALSE;
  }

     USBPD_PDO_TypeDef  pdosrc;
     USBPD_PDO_TypeDef rdosnk;
     rdosnk.d32 = PWR_Port_PDO_Storage[0].SinkPDO.ListOfPDO[cPDOIndex-1];
     //  aSNKPDOBuffer[cPDOIndex-1];
     pdosrc.d32 = aSRCPDOBuffer[cPDOIndex-1];  

     if (rdosnk.GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_FIXED)
     {
       
       float voltage = 0, current = 0;
       prvGetVoltageCurrentFromPDO(aSRCPDOBuffer[cPDOIndex], &voltage, &current);
       
       sprintf( pcWriteBuffer, "Requested %d : %d.%.2dV %d.%.2dA\r\n", cPDOIndex,
               (uint16_t)(voltage/1000), ((uint16_t)((uint16_t)(voltage/10)%100)),
               (uint16_t)(current/1000), ((uint16_t)((uint16_t)(current/10)%100)));
     }
     else
     {
       if (rdosnk.GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_APDO)
       {
         /* print the current cIndex */
//         float voltage = 0, current = 0, voltagemin = 0, voltagemax =0;
//         prvGetVoltageCurrentFromSRCAPDO(aSRCPDOBuffer[cPDOIndex-1], &voltage, &voltagemin, &voltagemax,&current);
         
         sprintf( pcWriteBuffer, "SRC APDO %d - Request voltage: %dmV and Current: %dmA\r\n", cPDOIndex,
                 (uint16_t)(CLI_APDO_params.CLI_APDO_runningvalue), (uint16_t)(DPM_Ports[0].DPM_RequestedCurrent));
       }
     }
     
     
  return pdFALSE;
}

/**
 * @brief  CLI callback for the request command.
 */
static BaseType_t prvPRSwapCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* local variables definition */
  static portCHAR cPort = USBPD_DEF_PORT;
  const char *pcParameter = NULL;
  int8_t connStatus = -1;
  uint8_t cDrpSupport = USBPD_FALSE;
  USBPD_PortPowerRole_TypeDef currRole = USBPD_CABLEPLUG_FROMDFPUFP;

  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
  ( void ) pcParameter;

  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );
        
  /* 
    Get parameters 
    if the user specify all parameters the command is a set (for the blink mode need also the period)
    otherwise the command is a get
  */
  
#if PORT_PARAM_ENABLE == 1
  /* check the port parameter */
  cPort = prvCommandCheckPortNumber(pcWriteBuffer, xWriteBufferLen, pcCommandString);
  if (cPort == CLI_PORTNUM_INVALID)
  {
    /* in case of a wrong PortNumber complete the command */
    return pdFALSE;
  }
#endif /* PORT_PARAM_ENABLE */

  USBPD_DPM_CLI_GetCurrentRole((uint8_t)cPort, &currRole, &connStatus, &cDrpSupport);
  
  if (connStatus == (USBPD_POWER_EXPLICITCONTRACT+1))
  {
    sprintf(pcWriteBuffer, "Power role swap on Port %d\r\n", cPort);
    strcat(pcWriteBuffer, "Current role: ");
    strcat(pcWriteBuffer,currRoleText[prvPortPowerRole2Index(currRole)]);
    strcat(pcWriteBuffer, "\r\n");
    USBPD_DPM_RequestPowerRoleSwap(cPort);
    osDelay(300);
    USBPD_DPM_CLI_GetCurrentRole((uint8_t)cPort, &currRole, &connStatus, &cDrpSupport);
    strcat(pcWriteBuffer, "New role: ");
    strcat(pcWriteBuffer,currRoleText[prvPortPowerRole2Index(currRole)]);
    strcat(pcWriteBuffer, "\r\n");
  }
  else
  {
    strcpy(pcWriteBuffer, "Warning : power role swap not sent, missing explicit contract\r\n");
  }

  
  return pdFALSE;
}

static BaseType_t prvAPDOreqCommandFunc( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  /* local variables definition */
  static portCHAR cPort = USBPD_DEF_PORT;
  const char *pcParameter = NULL;
  const char *voltagemin = NULL;
  const char *voltagemax = NULL;
//  const char *voltagedir = NULL;
  const char *voltagestep = NULL;

  int8_t connStatus = -1;
  uint8_t cDrpSupport = USBPD_FALSE;
  static uint16_t cPDOvoltmin = 0;
  static uint16_t cPDOvoltmax = 0;
  static uint16_t cPDOvoltstep = 0;
//  static uint16_t cPDOvoltdir = 0;
  USBPD_PortPowerRole_TypeDef currRole = USBPD_CABLEPLUG_FROMDFPUFP;
  BaseType_t xParameterStringLength;
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
  ( void ) pcParameter;

  /* Check pointers */
  configASSERT( pcWriteBuffer );
  configASSERT( pcCommandString );
        
  /* 
    Get parameters 
    if the user specify all parameters the command is a set (for the blink mode need also the period)
    otherwise the command is a get
  */
  
#if PORT_PARAM_ENABLE == 1
  /* check the port parameter */
  cPort = prvCommandCheckPortNumber(pcWriteBuffer, xWriteBufferLen, pcCommandString);
  if (cPort == CLI_PORTNUM_INVALID)
  {
    /* in case of a wrong PortNumber complete the command */
    return pdFALSE;
  }
#endif /* PORT_PARAM_ENABLE */

  USBPD_DPM_CLI_GetCurrentRole((uint8_t)cPort, &currRole, &connStatus, &cDrpSupport);
  
  /* First parameter: minimum voltage for APDO ramp */
    voltagemin = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);  
    cPDOvoltmin = atoi(voltagemin);
  /* clamp of APDO min voltage if necessary */
    if ( (cPDOvoltmin < CLI_APDO_params.CLI_APDO_Voltagemin_save) || (cPDOvoltmin > CLI_APDO_params.CLI_APDO_Voltagemax_save))
      {      
      CLI_APDO_params.CLI_APDO_Voltagemin= CLI_APDO_params.CLI_APDO_Voltagemin_save;
      cPDOvoltmin = CLI_APDO_params.CLI_APDO_Voltagemin;
      }
    else
      {
      CLI_APDO_params.CLI_APDO_Voltagemin=cPDOvoltmin;
       }
          
  /* Second parameter: maximum voltage for APDO ramp */
    voltagemax = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);  
    cPDOvoltmax = atoi(voltagemax);
 /* clamp of APDO max voltage if necessary */
   if (cPDOvoltmax < (cPDOvoltmin + 20))
     cPDOvoltmax = cPDOvoltmin+20;
   if ( (cPDOvoltmax < CLI_APDO_params.CLI_APDO_Voltagemin_save) || (cPDOvoltmax > CLI_APDO_params.CLI_APDO_Voltagemax_save))
      {      
      CLI_APDO_params.CLI_APDO_Voltagemax= CLI_APDO_params.CLI_APDO_Voltagemax_save;
      cPDOvoltmax = CLI_APDO_params.CLI_APDO_Voltagemax;
      }
    else
      {
      CLI_APDO_params.CLI_APDO_Voltagemax = cPDOvoltmax;
      }
         
  /* Third parameter: step voltage for APDO ramp */
    voltagestep = FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength);  
    cPDOvoltstep = atoi(voltagestep);
     /* clamp of APDO step voltage if necessary */
    if (cPDOvoltstep < 20)
      cPDOvoltstep = 20;
    if (cPDOvoltstep > (cPDOvoltmax - cPDOvoltmin))
      cPDOvoltstep = 20;
     cPDOvoltstep = (((int)(cPDOvoltstep / 20)) * 20);
    CLI_APDO_params.CLI_APDO_Voltagestep = cPDOvoltstep;
    
// /* Fourth parameter: direction of APDO ramp */ 
//    voltagedir = FreeRTOS_CLIGetParameter(pcCommandString, 4, &xParameterStringLength);  
//    cPDOvoltdir = atoi(voltagedir);
//    if ((cPDOvoltdir != 0) && (cPDOvoltdir != 1))
//      cPDOvoltdir = 0;
//    CLI_APDO_params.CLI_APDO_Voltagedir = cPDOvoltdir;
//    sprintf(pcWriteBuffer, "APDO settings on Port %d: Min %dmV Max %dmV Step:%dmV \r\n", cPort,cPDOvoltmin, cPDOvoltmax, cPDOvoltstep);
//      
//    if (0 == CLI_APDO_params.CLI_APDO_Voltagedir)
//        CLI_APDO_params.CLI_APDO_Voltagestep = (-1) * CLI_APDO_params.CLI_APDO_Voltagestep;

    CLI_APDO_params.CLI_APDO_runningvalue = CLI_APDO_params.CLI_APDO_Voltagemin - CLI_APDO_params.CLI_APDO_Voltagestep;
//    DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MinOperatingVoltageInmVunits = CLI_APDO_params.CLI_APDO_Voltagemin;
//    DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MaxOperatingVoltageInmVunits = CLI_APDO_params.CLI_APDO_Voltagemax;
    return pdFALSE;
}


/*-----------------------------------------------------------*/

/**
 * @brief  API To perform a start of the Commands module.
 * @param  usStackSize       specify the stack size of the task
 * @param  uxPriority        specify the priority of the task
 * @param  xQueueInParam     input queue where find the command
 * @param  xQueueOutParam    output queue where redirect the output
 */
void CLI_CommandStart( uint16_t usStackSize, 
                      osPriority xPriority, 
                      xQueueHandle xQueueInParam, 
                      xQueueHandle xQueueOutParam )
{
  CLI_RegisterCommands();
  
  xQueueIn = xQueueInParam;
  xQueueOut = xQueueOutParam;
  
  /* Create that thread that handles the console itself. */
  osThreadDef(CLICmd, prvCommandThread, xPriority, 0, usStackSize);
  xCmdThreadId = osThreadCreate(osThread(CLICmd), NULL);
  configASSERT( xCmdThreadId != NULL );

  /* Send the welcome message. */
  if (CLI_ConfigWelcomeMessage != NULL)
  {
    uint8_t x;
    for (x=0; x < CLI_WELCOME_MESSAGE_LEN; x++)
    {
      xQueueSendToBack( xQueueOut, ( signed char * ) CLI_ConfigWelcomeMessage[x], 100); 
    }
    xQueueSendToBack( xQueueOut, ( signed char * ) CLI_ConfigEndOfOutputMessage, 100); 
  }
  
//    CLI_APDO_params.CLI_APDO_Voltagemin_save= DPM_USER_Settings[0].DPM_SNKRequestedPower.MinOperatingVoltageInmVunits;
//  CLI_APDO_params.CLI_APDO_Voltagemax_save= DPM_USER_Settings[0].DPM_SNKRequestedPower.MaxOperatingVoltageInmVunits;

}

/**
 * @brief  Task that perfom a command process, calling the FreeRTOS-CLI API.
 * @param  pvParameters is the pointer to the task parameter
 */
static void prvCommandThread( void const * argument )
{
  //check err queue if item
  portBASE_TYPE xStatus;
  BaseType_t xReturned;
  char *pcOutputString;
  ( void ) argument;
  
  pcOutputString = FreeRTOS_CLIGetOutputBuffer();
  
  for( ;; )
  {
    xStatus = xQueueReceive( xQueueIn, cInputString, portMAX_DELAY);
    if (xStatus == pdPASS)
    {
      if (cInputString[0] != 0)
      {
        do
        {
                /* Get the next output string from the command interpreter. */
                xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, CLI_INPUT_MAX_SIZE );

                /* Write the generated string to the UART. */
                xQueueSendToBack( xQueueOut, ( signed char * ) pcOutputString, 0); 
                //printf(pcOutputString);
        } while( xReturned != pdFALSE );
      }
      xQueueSendToBack( xQueueOut, CLI_ConfigEndOfOutputMessage, 0);
    }
  }
}
static inline void prvGetVoltageCurrentFromPDO(uint32_t PdoValue, float *pVoltage, float *pCurrent)
{
  USBPD_PDO_TypeDef  pdo;
  
  pdo.d32 = PdoValue;
  switch(pdo.GenericPDO.PowerObject)
  {
  case USBPD_CORE_PDO_TYPE_FIXED:
    {
      USBPD_SRCFixedSupplyPDO_TypeDef fixedpdo = pdo.SRCFixedPDO;
      /* convert voltage if pointer is not null */
      if (pVoltage)
      {
        *pVoltage = PWR_DECODE_50MV(fixedpdo.VoltageIn50mVunits);
      }
      /* convert current if pointer is not null */
      if (pCurrent)
      {
        *pCurrent = PWR_DECODE_10MA(fixedpdo.MaxCurrentIn10mAunits);
      }
    }
  default:
    break;
  }
}


               
static inline void prvGetVoltageCurrentFromSNKAPDO(uint32_t PdoValue, float *pVoltage, float *pVoltagemin,float *pVoltagemax, float *pCurrent)
{
  USBPD_PDO_TypeDef  pdo;
  
  pdo.d32 = PdoValue;
  switch(pdo.GenericPDO.PowerObject)
  {
  case USBPD_CORE_PDO_TYPE_FIXED:
    {
      USBPD_SRCFixedSupplyPDO_TypeDef fixedpdo = pdo.SRCFixedPDO;
      /* convert voltage if pointer is not null */
      if (pVoltage)
      {
        *pVoltage = PWR_DECODE_50MV(fixedpdo.VoltageIn50mVunits);
        *pVoltagemin = 0;
        *pVoltagemax = 0;
      }
      /* convert current if pointer is not null */
      if (pCurrent)
      {
        *pCurrent = PWR_DECODE_10MA(fixedpdo.MaxCurrentIn10mAunits);
      }
     break;

    }
   case USBPD_CORE_PDO_TYPE_APDO:
    {
      USBPD_ProgrammablePowerSupplyAPDO_TypeDef apdo = pdo.SRCSNKAPDO;
      /* convert voltage if pointer is not null */
      if (pVoltagemin)
      {
//          *pVoltagemin = PWR_DECODE_100MV(apdo.MinVoltageIn100mV);
//          *pVoltagemax = PWR_DECODE_100MV(apdo.MaxVoltageIn100mV);
        *pVoltagemin = CLI_APDO_params.CLI_APDO_Voltagemin;
        *pVoltagemax = CLI_APDO_params.CLI_APDO_Voltagemax;
        *pVoltage = 0;
      }
      /* convert current if pointer is not null */
      if (pCurrent)
      {
        *pCurrent = PWR_DECODE_50MA(apdo.MaxCurrentIn50mAunits);
      }
    } 
  default:
    break;
  }
}

static inline void prvGetVoltageCurrentFromSRCAPDO(uint32_t PdoValue, float *pVoltage, float *pVoltagemin,float *pVoltagemax, float *pCurrent)
{
  USBPD_PDO_TypeDef  pdo;
  
  pdo.d32 = PdoValue;
  switch(pdo.GenericPDO.PowerObject)
  {
  case USBPD_CORE_PDO_TYPE_FIXED:
    {
      USBPD_SRCFixedSupplyPDO_TypeDef fixedpdo = pdo.SRCFixedPDO;
      /* convert voltage if pointer is not null */
      if (pVoltage)
      {
        *pVoltage = PWR_DECODE_50MV(fixedpdo.VoltageIn50mVunits);
        *pVoltagemin = 0;
        *pVoltagemax = 0;
      }
      /* convert current if pointer is not null */
      if (pCurrent)
      {
        *pCurrent = PWR_DECODE_10MA(fixedpdo.MaxCurrentIn10mAunits);
      }
     break;

    }
   case USBPD_CORE_PDO_TYPE_APDO:
    {
      USBPD_ProgrammablePowerSupplyAPDO_TypeDef apdo = pdo.SRCSNKAPDO;
      /* convert voltage if pointer is not null */
      if (pVoltagemin)
      {
          *pVoltagemin = PWR_DECODE_100MV(apdo.MinVoltageIn100mV);
          *pVoltagemax = PWR_DECODE_100MV(apdo.MaxVoltageIn100mV);
//        *pVoltagemin = CLI_APDO_params.CLI_APDO_Voltagemin;
//        *pVoltagemax = CLI_APDO_params.CLI_APDO_Voltagemax;
        *pVoltage = 0;
      }
      /* convert current if pointer is not null */
      if (pCurrent)
      {
        *pCurrent = PWR_DECODE_50MA(apdo.MaxCurrentIn50mAunits);
      }
    } 
  default:
    break;
  }
}


#if PORT_PARAM_ENABLE == 1
static portCHAR prvCommandCheckPortNumber(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  portCHAR cPort = CLI_PORTNUM_INVALID;
  const char *pcParameter = NULL;
  BaseType_t xParameterStringLength;
  
  /* To avoid warnings */
  ( void ) pcWriteBuffer;
  ( void ) xWriteBufferLen;
  ( void ) pcCommandString;
  
  if (pcCommandString)
  {
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    cPort = pcParameter != NULL && xParameterStringLength == 1 && (pcParameter[0] >= '0' && pcParameter[0] <= ('0' + 1))  ? pcParameter[0] - '0' : 0xFF;
    if (pcWriteBuffer != NULL && cPort == CLI_PORTNUM_INVALID)
    {
      sprintf( pcWriteBuffer, "Error: invalid parameter port '%s'\r\n", pcParameter != NULL ? pcParameter : "<NULL>");
    }
  }
  return cPort;
}
#endif /* PORT_PARAM_ENABLE */
#endif /* USBPD_CLI */
