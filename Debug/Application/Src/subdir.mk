################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Src/IMU.c \
../Application/Src/LineSensor.c \
../Application/Src/Radio.c 

OBJS += \
./Application/Src/IMU.o \
./Application/Src/LineSensor.o \
./Application/Src/Radio.o 

C_DEPS += \
./Application/Src/IMU.d \
./Application/Src/LineSensor.d \
./Application/Src/Radio.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Src/%.o Application/Src/%.su Application/Src/%.cyclo: ../Application/Src/%.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/horgo/STM32CubeIDE/workspace_1.13.2/JLBRacing2024/Application/Inc" -I../Drivers/BSP/Components/lsm6dsl -I../MEMS/Target -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Src

clean-Application-2f-Src:
	-$(RM) ./Application/Src/IMU.cyclo ./Application/Src/IMU.d ./Application/Src/IMU.o ./Application/Src/IMU.su ./Application/Src/LineSensor.cyclo ./Application/Src/LineSensor.d ./Application/Src/LineSensor.o ./Application/Src/LineSensor.su ./Application/Src/Radio.cyclo ./Application/Src/Radio.d ./Application/Src/Radio.o ./Application/Src/Radio.su

.PHONY: clean-Application-2f-Src

