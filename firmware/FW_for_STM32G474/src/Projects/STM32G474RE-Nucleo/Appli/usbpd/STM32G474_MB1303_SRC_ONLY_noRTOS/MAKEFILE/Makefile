##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.12.0-B5] date: [Mon Mar 08 10:43:07 CET 2021]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = Nucleo_STM32G474_MB1303_SRC_ONLY_noRTOS


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Os


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
../Src/main.c \
../Src/stm32g4xx_hal_msp.c \
../Src/usbpd_dpm_user.c \
../Src/usbpd_vdm_user.c \
../Src/usbpd_pwr_if.c \
../../../../../../Drivers/BSP/Components/STUSB1602/STUSB1602_Peripherals_if.c \
../../../../../../Drivers/BSP/Components/STUSB1602/STUSB1602_Registers_if.c \
../../../../../../Drivers/BSP/Components/STUSB1602/USBPD_CUST_NVM_API.c \
../../../../../../Drivers/BSP/P-NUCLEO-G474/p-nucleo-G474.c \
../../../../../../Drivers/BSP/P-NUCLEO-G474/stm32g4xx_it.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usart.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
../../../../../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_spi.c \
../Src/system_stm32g4xx.c \
../../../../../../Utilities/TRACER_EMB/tracer_emb.c \
../../../../../../Utilities/TRACER_EMB/tracer_emb_hw.c \
../../../../../../Utilities/LED_SERVER/led_server.c \
../../../../../../Middlewares/Third_Party/FreeRTOS/Source/list.c \
../../../../../../Middlewares/Third_Party/FreeRTOS/Source/queue.c \
../../../../../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
../../../../../../Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_dpm_core.c \
../../../../../../Middlewares/ST/STM32_USBPD_Library/Core/src/usbpd_trace.c \
../../../../../../Middlewares/ST/STM32_USBPD_Library/Devices/STUSB1602/src/usbpd_cad_hw_if.c \
../../../../../../Middlewares/ST/STM32_USBPD_Library/Devices/STUSB1602/src/usbpd_hw_if_it.c \
../../../../../../Middlewares/ST/STM32_USBPD_Library/Devices/STUSB1602/src/usbpd_phy_hw_if.c \
../../../../../../Middlewares/ST/STM32_USBPD_Library/Devices/STUSB1602/src/usbpd_timersserver.c \
../../../../../../Middlewares/ST/STM32_USBPD_Library/Devices/STUSB1602/src/usbpd_phy.c \
../../../../../../Middlewares/ST/STM32_USBPD_Library/Devices/STUSB1602/src/usbpd_Stusb_Dpm_if.c  

# ASM sources
ASM_SOURCES =  \
startup_stm32g474xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =  \
-DSTM32G474xx

# C defines
C_DEFS =  \
-DSTM32G474xx \
-DUSE_HAL_DRIVER \
-D_RTOS \
-DUSBPD_LED_SERVER \
-DUSBPD_PORT_COUNT=1 \
-D_SRC \
-DMB1303 \
-DUSE_FULL_LL_DRIVER \
-DUSBPD_STUSB1602 \
-DSPI_ONE_LINE_na \
-DCONF_NORMAL \
-DUSBPDCORE_LIB_PD3_CONFIG_MINSRC \
-D_TRACE \
-D_ERROR_RECOVERY \
-D_VCONN_SUPPORT \
-D_ADC_MONITORING_na \
-D_GPIO_FOR_SRC_na


# AS includes
AS_INCLUDES =  \
-I \
-I../Inc \
-I../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include

# C includes
C_INCLUDES =  \
-I../Inc \
-I../../../../../../Drivers/CMSIS/Device/ST/STM32G4xx/Include \
-I../../../../../../Drivers/STM32G4xx_HAL_Driver/Inc \
-I../../../../../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy \
-I../../../../../../Drivers/BSP/P-NUCLEO-G474 \
-I../../../../../../Drivers/BSP/P-NUCLEO-G474 \
-I../../../../../../Drivers/CMSIS/Include \
-I../../../../../../Drivers/BSP/Components/STUSB1602 \
-I../../../../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I../../../../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-I../../../../../../Middlewares/Third_Party/FreeRTOS/Source/include \
-I../../../../../../Middlewares/ST/STM32_USBPD_Library/Core/inc \
-I../../../../../../Utilities/TRACER_EMB \
-I../../../../../../Middlewares/ST/STM32_USBPD_Library/Devices/STUSB1602/inc \
-I../../../../../../Utilities/LED_SERVER \
-I../../../../../../Drivers/CMSIS/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32G474QE_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys  \
-l:USBPDCORE_PD3_CONFIG_MINSRC_CM4_wc32.a
LIBDIR =  \
-L../../../../../../Middlewares/ST/STM32_USBPD_Library/Core/lib
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***