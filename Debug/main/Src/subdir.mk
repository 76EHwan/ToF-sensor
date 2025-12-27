################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../main/Src/LSM6DS3TR.c \
../main/Src/drive.c \
../main/Src/init.c \
../main/Src/motor.c \
../main/Src/sensor.c 

OBJS += \
./main/Src/LSM6DS3TR.o \
./main/Src/drive.o \
./main/Src/init.o \
./main/Src/motor.o \
./main/Src/sensor.o 

C_DEPS += \
./main/Src/LSM6DS3TR.d \
./main/Src/drive.d \
./main/Src/init.d \
./main/Src/motor.d \
./main/Src/sensor.d 


# Each subdirectory must supply rules for building sources it contributes
main/Src/%.o main/Src/%.su main/Src/%.cyclo: ../main/Src/%.c main/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I../Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Src -I../main/Inc -I../Drivers/BSP/ST7789 -I../Drivers/BSP/VL53L0X/Core/Inc -I../Drivers/BSP/VL53L0X/Platform/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-main-2f-Src

clean-main-2f-Src:
	-$(RM) ./main/Src/LSM6DS3TR.cyclo ./main/Src/LSM6DS3TR.d ./main/Src/LSM6DS3TR.o ./main/Src/LSM6DS3TR.su ./main/Src/drive.cyclo ./main/Src/drive.d ./main/Src/drive.o ./main/Src/drive.su ./main/Src/init.cyclo ./main/Src/init.d ./main/Src/init.o ./main/Src/init.su ./main/Src/motor.cyclo ./main/Src/motor.d ./main/Src/motor.o ./main/Src/motor.su ./main/Src/sensor.cyclo ./main/Src/sensor.d ./main/Src/sensor.o ./main/Src/sensor.su

.PHONY: clean-main-2f-Src

