################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Src/ADC.c \
../Application/Src/DistanceSensor.c \
../Application/Src/Encoder.c \
../Application/Src/IMU.c \
../Application/Src/LineSensor.c 

CPP_SRCS += \
../Application/Src/MotorControl.cpp \
../Application/Src/Radio.cpp \
../Application/Src/Servo.cpp \
../Application/Src/Tasks.cpp 

C_DEPS += \
./Application/Src/ADC.d \
./Application/Src/DistanceSensor.d \
./Application/Src/Encoder.d \
./Application/Src/IMU.d \
./Application/Src/LineSensor.d 

OBJS += \
./Application/Src/ADC.o \
./Application/Src/DistanceSensor.o \
./Application/Src/Encoder.o \
./Application/Src/IMU.o \
./Application/Src/LineSensor.o \
./Application/Src/MotorControl.o \
./Application/Src/Radio.o \
./Application/Src/Servo.o \
./Application/Src/Tasks.o 

CPP_DEPS += \
./Application/Src/MotorControl.d \
./Application/Src/Radio.d \
./Application/Src/Servo.d \
./Application/Src/Tasks.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Src/%.o Application/Src/%.su Application/Src/%.cyclo: ../Application/Src/%.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/horgo/STM32CubeIDE/workspace_1.13.2/JLBRacing2024/Application/Inc" -I"C:/Users/horgo/STM32CubeIDE/workspace_1.13.2/JLBRacing2024/Application/Inc/JLB/lib/can_matrix/conf" -I"C:/Users/horgo/STM32CubeIDE/workspace_1.13.2/JLBRacing2024/Application/Inc/JLB/lib/can_matrix/butl" -I"C:/Users/horgo/STM32CubeIDE/workspace_1.13.2/JLBRacing2024/Application/Inc/JLB/lib/can_matrix/lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/Src/%.o Application/Src/%.su Application/Src/%.cyclo: ../Application/Src/%.cpp Application/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++20 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/horgo/STM32CubeIDE/workspace_1.13.2/JLBRacing2024/Application/Inc" -I"C:/Users/horgo/STM32CubeIDE/workspace_1.13.2/JLBRacing2024/Application/Inc/JLB/lib/can_matrix/conf" -I"C:/Users/horgo/STM32CubeIDE/workspace_1.13.2/JLBRacing2024/Application/Inc/JLB/lib/can_matrix/butl" -I"C:/Users/horgo/STM32CubeIDE/workspace_1.13.2/JLBRacing2024/Application/Inc/JLB/lib/can_matrix/lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Src

clean-Application-2f-Src:
	-$(RM) ./Application/Src/ADC.cyclo ./Application/Src/ADC.d ./Application/Src/ADC.o ./Application/Src/ADC.su ./Application/Src/DistanceSensor.cyclo ./Application/Src/DistanceSensor.d ./Application/Src/DistanceSensor.o ./Application/Src/DistanceSensor.su ./Application/Src/Encoder.cyclo ./Application/Src/Encoder.d ./Application/Src/Encoder.o ./Application/Src/Encoder.su ./Application/Src/IMU.cyclo ./Application/Src/IMU.d ./Application/Src/IMU.o ./Application/Src/IMU.su ./Application/Src/LineSensor.cyclo ./Application/Src/LineSensor.d ./Application/Src/LineSensor.o ./Application/Src/LineSensor.su ./Application/Src/MotorControl.cyclo ./Application/Src/MotorControl.d ./Application/Src/MotorControl.o ./Application/Src/MotorControl.su ./Application/Src/Radio.cyclo ./Application/Src/Radio.d ./Application/Src/Radio.o ./Application/Src/Radio.su ./Application/Src/Servo.cyclo ./Application/Src/Servo.d ./Application/Src/Servo.o ./Application/Src/Servo.su ./Application/Src/Tasks.cyclo ./Application/Src/Tasks.d ./Application/Src/Tasks.o ./Application/Src/Tasks.su

.PHONY: clean-Application-2f-Src

