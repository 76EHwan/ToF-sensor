################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/ST7789/fonts.c \
../Drivers/BSP/ST7789/st7789.c 

OBJS += \
./Drivers/BSP/ST7789/fonts.o \
./Drivers/BSP/ST7789/st7789.o 

C_DEPS += \
./Drivers/BSP/ST7789/fonts.d \
./Drivers/BSP/ST7789/st7789.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/ST7789/%.o Drivers/BSP/ST7789/%.su Drivers/BSP/ST7789/%.cyclo: ../Drivers/BSP/ST7789/%.c Drivers/BSP/ST7789/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I../Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Src -I../main/Inc -I../Drivers/BSP/ST7789 -I../Drivers/BSP/VL53L0X/Core/Inc -I../Drivers/BSP/VL53L0X/Platform/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-ST7789

clean-Drivers-2f-BSP-2f-ST7789:
	-$(RM) ./Drivers/BSP/ST7789/fonts.cyclo ./Drivers/BSP/ST7789/fonts.d ./Drivers/BSP/ST7789/fonts.o ./Drivers/BSP/ST7789/fonts.su ./Drivers/BSP/ST7789/st7789.cyclo ./Drivers/BSP/ST7789/st7789.d ./Drivers/BSP/ST7789/st7789.o ./Drivers/BSP/ST7789/st7789.su

.PHONY: clean-Drivers-2f-BSP-2f-ST7789

