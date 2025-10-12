################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Periphs/MOT_DRV.c \
../Core/Periphs/SD.c 

OBJS += \
./Core/Periphs/MOT_DRV.o \
./Core/Periphs/SD.o 

C_DEPS += \
./Core/Periphs/MOT_DRV.d \
./Core/Periphs/SD.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Periphs/%.o Core/Periphs/%.su Core/Periphs/%.cyclo: ../Core/Periphs/%.c Core/Periphs/subdir.mk
	arm-none-eabi-gcc  -gdwarf-4 "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Core/App -I../Core/Sensors -I../Core/Control -I../Core/Periphs -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Periphs

clean-Core-2f-Periphs:
	-$(RM) ./Core/Periphs/MOT_DRV.cyclo ./Core/Periphs/MOT_DRV.d ./Core/Periphs/MOT_DRV.o ./Core/Periphs/MOT_DRV.su ./Core/Periphs/SD.cyclo ./Core/Periphs/SD.d ./Core/Periphs/SD.o ./Core/Periphs/SD.su

.PHONY: clean-Core-2f-Periphs

