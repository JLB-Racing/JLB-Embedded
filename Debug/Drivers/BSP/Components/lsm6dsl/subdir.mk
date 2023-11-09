################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm6dsl/lsm6dsl.c \
../Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.c 

OBJS += \
./Drivers/BSP/Components/lsm6dsl/lsm6dsl.o \
./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lsm6dsl/lsm6dsl.d \
./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm6dsl/%.o Drivers/BSP/Components/lsm6dsl/%.su Drivers/BSP/Components/lsm6dsl/%.cyclo: ../Drivers/BSP/Components/lsm6dsl/%.c Drivers/BSP/Components/lsm6dsl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/horgo/STM32CubeIDE/workspace_1.13.2/JLBRacing2024/Application/Inc" -I../Drivers/BSP/Components/lsm6dsl -I../X-CUBE-MEMS1/Target -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsl

clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsl:
	-$(RM) ./Drivers/BSP/Components/lsm6dsl/lsm6dsl.cyclo ./Drivers/BSP/Components/lsm6dsl/lsm6dsl.d ./Drivers/BSP/Components/lsm6dsl/lsm6dsl.o ./Drivers/BSP/Components/lsm6dsl/lsm6dsl.su ./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.cyclo ./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.d ./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.o ./Drivers/BSP/Components/lsm6dsl/lsm6dsl_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsl

