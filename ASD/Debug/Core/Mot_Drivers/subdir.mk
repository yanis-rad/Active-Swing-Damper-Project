################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Mot_Drivers/MOT_DRV.c 

OBJS += \
./Core/Mot_Drivers/MOT_DRV.o 

C_DEPS += \
./Core/Mot_Drivers/MOT_DRV.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Mot_Drivers/%.o Core/Mot_Drivers/%.su Core/Mot_Drivers/%.cyclo: ../Core/Mot_Drivers/%.c Core/Mot_Drivers/subdir.mk
	arm-none-eabi-gcc  -gdwarf-4 "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Core/App -I../Core/Sensors -I../Core/Control -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Mot_Drivers

clean-Core-2f-Mot_Drivers:
	-$(RM) ./Core/Mot_Drivers/MOT_DRV.cyclo ./Core/Mot_Drivers/MOT_DRV.d ./Core/Mot_Drivers/MOT_DRV.o ./Core/Mot_Drivers/MOT_DRV.su

.PHONY: clean-Core-2f-Mot_Drivers

