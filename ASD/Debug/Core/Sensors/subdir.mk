################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Sensors/DSC.c \
../Core/Sensors/IMU.c \
../Core/Sensors/MOT_ENC.c 

OBJS += \
./Core/Sensors/DSC.o \
./Core/Sensors/IMU.o \
./Core/Sensors/MOT_ENC.o 

C_DEPS += \
./Core/Sensors/DSC.d \
./Core/Sensors/IMU.d \
./Core/Sensors/MOT_ENC.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Sensors/%.o Core/Sensors/%.su Core/Sensors/%.cyclo: ../Core/Sensors/%.c Core/Sensors/subdir.mk
	arm-none-eabi-gcc  -gdwarf-4 "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Core/App -I../Core/Sensors -I../Core/Control -I../Core/Periphs -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Sensors

clean-Core-2f-Sensors:
	-$(RM) ./Core/Sensors/DSC.cyclo ./Core/Sensors/DSC.d ./Core/Sensors/DSC.o ./Core/Sensors/DSC.su ./Core/Sensors/IMU.cyclo ./Core/Sensors/IMU.d ./Core/Sensors/IMU.o ./Core/Sensors/IMU.su ./Core/Sensors/MOT_ENC.cyclo ./Core/Sensors/MOT_ENC.d ./Core/Sensors/MOT_ENC.o ./Core/Sensors/MOT_ENC.su

.PHONY: clean-Core-2f-Sensors

