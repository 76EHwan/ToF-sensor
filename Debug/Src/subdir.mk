################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/custom_flash.c \
../Src/custom_switch.c \
../Src/debug.c \
../Src/dwt_delay.c \
../Src/gpio.c \
../Src/lptim.c \
../Src/main.c \
../Src/memorymap.c \
../Src/rtc.c \
../Src/spi.c \
../Src/stm32h7xx_hal_msp.c \
../Src/stm32h7xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32h7xx.c \
../Src/tim.c \
../Src/usart.c \
../Src/usb_otg.c 

OBJS += \
./Src/adc.o \
./Src/custom_flash.o \
./Src/custom_switch.o \
./Src/debug.o \
./Src/dwt_delay.o \
./Src/gpio.o \
./Src/lptim.o \
./Src/main.o \
./Src/memorymap.o \
./Src/rtc.o \
./Src/spi.o \
./Src/stm32h7xx_hal_msp.o \
./Src/stm32h7xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32h7xx.o \
./Src/tim.o \
./Src/usart.o \
./Src/usb_otg.o 

C_DEPS += \
./Src/adc.d \
./Src/custom_flash.d \
./Src/custom_switch.d \
./Src/debug.d \
./Src/dwt_delay.d \
./Src/gpio.d \
./Src/lptim.d \
./Src/main.d \
./Src/memorymap.d \
./Src/rtc.d \
./Src/spi.d \
./Src/stm32h7xx_hal_msp.d \
./Src/stm32h7xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32h7xx.d \
./Src/tim.d \
./Src/usart.d \
./Src/usb_otg.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I../Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Src -I../main/Inc -I../Drivers/BSP/ST7789 -I../Drivers/BSP/VL53L0X/Core/Inc -I../Drivers/BSP/VL53L0X/Platform/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/adc.cyclo ./Src/adc.d ./Src/adc.o ./Src/adc.su ./Src/custom_flash.cyclo ./Src/custom_flash.d ./Src/custom_flash.o ./Src/custom_flash.su ./Src/custom_switch.cyclo ./Src/custom_switch.d ./Src/custom_switch.o ./Src/custom_switch.su ./Src/debug.cyclo ./Src/debug.d ./Src/debug.o ./Src/debug.su ./Src/dwt_delay.cyclo ./Src/dwt_delay.d ./Src/dwt_delay.o ./Src/dwt_delay.su ./Src/gpio.cyclo ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/lptim.cyclo ./Src/lptim.d ./Src/lptim.o ./Src/lptim.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/memorymap.cyclo ./Src/memorymap.d ./Src/memorymap.o ./Src/memorymap.su ./Src/rtc.cyclo ./Src/rtc.d ./Src/rtc.o ./Src/rtc.su ./Src/spi.cyclo ./Src/spi.d ./Src/spi.o ./Src/spi.su ./Src/stm32h7xx_hal_msp.cyclo ./Src/stm32h7xx_hal_msp.d ./Src/stm32h7xx_hal_msp.o ./Src/stm32h7xx_hal_msp.su ./Src/stm32h7xx_it.cyclo ./Src/stm32h7xx_it.d ./Src/stm32h7xx_it.o ./Src/stm32h7xx_it.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32h7xx.cyclo ./Src/system_stm32h7xx.d ./Src/system_stm32h7xx.o ./Src/system_stm32h7xx.su ./Src/tim.cyclo ./Src/tim.d ./Src/tim.o ./Src/tim.su ./Src/usart.cyclo ./Src/usart.d ./Src/usart.o ./Src/usart.su ./Src/usb_otg.cyclo ./Src/usb_otg.d ./Src/usb_otg.o ./Src/usb_otg.su

.PHONY: clean-Src

