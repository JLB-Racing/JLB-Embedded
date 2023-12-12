################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Inc/JLB/lib/can_matrix/lib/jlb.c 

C_DEPS += \
./Application/Inc/JLB/lib/can_matrix/lib/jlb.d 

OBJS += \
./Application/Inc/JLB/lib/can_matrix/lib/jlb.o 


# Each subdirectory must supply rules for building sources it contributes
Application/Inc/JLB/lib/can_matrix/lib/%.o Application/Inc/JLB/lib/can_matrix/lib/%.su Application/Inc/JLB/lib/can_matrix/lib/%.cyclo: ../Application/Inc/JLB/lib/can_matrix/lib/%.c Application/Inc/JLB/lib/can_matrix/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc" -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/conf" -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/butl" -I"C:/Users/flieg/git/JLBracing_Robonaut/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Inc-2f-JLB-2f-lib-2f-can_matrix-2f-lib

clean-Application-2f-Inc-2f-JLB-2f-lib-2f-can_matrix-2f-lib:
	-$(RM) ./Application/Inc/JLB/lib/can_matrix/lib/jlb.cyclo ./Application/Inc/JLB/lib/can_matrix/lib/jlb.d ./Application/Inc/JLB/lib/can_matrix/lib/jlb.o ./Application/Inc/JLB/lib/can_matrix/lib/jlb.su

.PHONY: clean-Application-2f-Inc-2f-JLB-2f-lib-2f-can_matrix-2f-lib

