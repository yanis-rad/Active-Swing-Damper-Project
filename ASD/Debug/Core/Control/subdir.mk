################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Control/PID.c 

OBJS += \
./Core/Control/PID.o 

C_DEPS += \
./Core/Control/PID.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Control/%.o Core/Control/%.su Core/Control/%.cyclo: ../Core/Control/%.c Core/Control/subdir.mk
	arm-none-eabi-gcc  -gdwarf-4 "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Core/App -I../Core/Sensors -I../Core/Control -I../Core/Periphs -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Control

clean-Core-2f-Control:
	-$(RM) ./Core/Control/PID.cyclo ./Core/Control/PID.d ./Core/Control/PID.o ./Core/Control/PID.su

.PHONY: clean-Core-2f-Control

