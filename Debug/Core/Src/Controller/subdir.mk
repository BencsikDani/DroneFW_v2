################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Controller/PID.c 

OBJS += \
./Core/Src/Controller/PID.o 

C_DEPS += \
./Core/Src/Controller/PID.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Controller/%.o Core/Src/Controller/%.su Core/Src/Controller/%.cyclo: ../Core/Src/Controller/%.c Core/Src/Controller/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Controller

clean-Core-2f-Src-2f-Controller:
	-$(RM) ./Core/Src/Controller/PID.cyclo ./Core/Src/Controller/PID.d ./Core/Src/Controller/PID.o ./Core/Src/Controller/PID.su

.PHONY: clean-Core-2f-Src-2f-Controller

