################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/App/DSC.c \
../Core/App/IMU.c \
../Core/App/MOT.c \
../Core/App/PID.c 

OBJS += \
./Core/App/DSC.o \
./Core/App/IMU.o \
./Core/App/MOT.o \
./Core/App/PID.o 

C_DEPS += \
./Core/App/DSC.d \
./Core/App/IMU.d \
./Core/App/MOT.d \
./Core/App/PID.d 


# Each subdirectory must supply rules for building sources it contributes
Core/App/%.o Core/App/%.su Core/App/%.cyclo: ../Core/App/%.c Core/App/subdir.mk
	arm-none-eabi-gcc  -gdwarf-4 "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Core/App -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-App

clean-Core-2f-App:
	-$(RM) ./Core/App/DSC.cyclo ./Core/App/DSC.d ./Core/App/DSC.o ./Core/App/DSC.su ./Core/App/IMU.cyclo ./Core/App/IMU.d ./Core/App/IMU.o ./Core/App/IMU.su ./Core/App/MOT.cyclo ./Core/App/MOT.d ./Core/App/MOT.o ./Core/App/MOT.su ./Core/App/PID.cyclo ./Core/App/PID.d ./Core/App/PID.o ./Core/App/PID.su

.PHONY: clean-Core-2f-App

