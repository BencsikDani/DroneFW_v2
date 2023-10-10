################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/HCSR04/HCSR04.c 

OBJS += \
./Core/Src/HCSR04/HCSR04.o 

C_DEPS += \
./Core/Src/HCSR04/HCSR04.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/HCSR04/%.o Core/Src/HCSR04/%.su Core/Src/HCSR04/%.cyclo: ../Core/Src/HCSR04/%.c Core/Src/HCSR04/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-HCSR04

clean-Core-2f-Src-2f-HCSR04:
	-$(RM) ./Core/Src/HCSR04/HCSR04.cyclo ./Core/Src/HCSR04/HCSR04.d ./Core/Src/HCSR04/HCSR04.o ./Core/Src/HCSR04/HCSR04.su

.PHONY: clean-Core-2f-Src-2f-HCSR04

