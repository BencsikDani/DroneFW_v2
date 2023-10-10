################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GY-91/BMP280.c \
../Core/Src/GY-91/MPU9250.c 

OBJS += \
./Core/Src/GY-91/BMP280.o \
./Core/Src/GY-91/MPU9250.o 

C_DEPS += \
./Core/Src/GY-91/BMP280.d \
./Core/Src/GY-91/MPU9250.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/GY-91/%.o Core/Src/GY-91/%.su Core/Src/GY-91/%.cyclo: ../Core/Src/GY-91/%.c Core/Src/GY-91/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-GY-2d-91

clean-Core-2f-Src-2f-GY-2d-91:
	-$(RM) ./Core/Src/GY-91/BMP280.cyclo ./Core/Src/GY-91/BMP280.d ./Core/Src/GY-91/BMP280.o ./Core/Src/GY-91/BMP280.su ./Core/Src/GY-91/MPU9250.cyclo ./Core/Src/GY-91/MPU9250.d ./Core/Src/GY-91/MPU9250.o ./Core/Src/GY-91/MPU9250.su

.PHONY: clean-Core-2f-Src-2f-GY-2d-91

