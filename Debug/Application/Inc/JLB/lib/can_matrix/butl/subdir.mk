################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Inc/JLB/lib/can_matrix/butl/jlb-binutil.c 

C_DEPS += \
./Application/Inc/JLB/lib/can_matrix/butl/jlb-binutil.d 

OBJS += \
./Application/Inc/JLB/lib/can_matrix/butl/jlb-binutil.o 


# Each subdirectory must supply rules for building sources it contributes
Application/Inc/JLB/lib/can_matrix/butl/%.o Application/Inc/JLB/lib/can_matrix/butl/%.su Application/Inc/JLB/lib/can_matrix/butl/%.cyclo: ../Application/Inc/JLB/lib/can_matrix/butl/%.c Application/Inc/JLB/lib/can_matrix/butl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I"C:/RobonAUT/JLB-Embedded/Application/Inc" -I"C:/RobonAUT/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/conf" -I"C:/RobonAUT/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/butl" -I"C:/RobonAUT/JLB-Embedded/Application/Inc/JLB/lib/can_matrix/lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Inc-2f-JLB-2f-lib-2f-can_matrix-2f-butl

clean-Application-2f-Inc-2f-JLB-2f-lib-2f-can_matrix-2f-butl:
	-$(RM) ./Application/Inc/JLB/lib/can_matrix/butl/jlb-binutil.cyclo ./Application/Inc/JLB/lib/can_matrix/butl/jlb-binutil.d ./Application/Inc/JLB/lib/can_matrix/butl/jlb-binutil.o ./Application/Inc/JLB/lib/can_matrix/butl/jlb-binutil.su

.PHONY: clean-Application-2f-Inc-2f-JLB-2f-lib-2f-can_matrix-2f-butl

