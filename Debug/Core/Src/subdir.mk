################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app_freertos.c \
../Core/Src/stm32l5xx_hal_msp.c \
../Core/Src/stm32l5xx_hal_timebase_tim.c \
../Core/Src/stm32l5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l5xx.c 

CPP_SRCS += \
../Core/Src/main.cpp 

C_DEPS += \
./Core/Src/app_freertos.d \
./Core/Src/stm32l5xx_hal_msp.d \
./Core/Src/stm32l5xx_hal_timebase_tim.d \
./Core/Src/stm32l5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l5xx.d 

OBJS += \
./Core/Src/app_freertos.o \
./Core/Src/main.o \
./Core/Src/stm32l5xx_hal_msp.o \
./Core/Src/stm32l5xx_hal_timebase_tim.o \
./Core/Src/stm32l5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l5xx.o 

CPP_DEPS += \
./Core/Src/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc" -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/conf" -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/butl" -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++20 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc" -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/conf" -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/butl" -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32l5xx_hal_msp.cyclo ./Core/Src/stm32l5xx_hal_msp.d ./Core/Src/stm32l5xx_hal_msp.o ./Core/Src/stm32l5xx_hal_msp.su ./Core/Src/stm32l5xx_hal_timebase_tim.cyclo ./Core/Src/stm32l5xx_hal_timebase_tim.d ./Core/Src/stm32l5xx_hal_timebase_tim.o ./Core/Src/stm32l5xx_hal_timebase_tim.su ./Core/Src/stm32l5xx_it.cyclo ./Core/Src/stm32l5xx_it.d ./Core/Src/stm32l5xx_it.o ./Core/Src/stm32l5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l5xx.cyclo ./Core/Src/system_stm32l5xx.d ./Core/Src/system_stm32l5xx.o ./Core/Src/system_stm32l5xx.su

.PHONY: clean-Core-2f-Src

