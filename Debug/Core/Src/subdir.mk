################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Debug.c \
../Core/Src/Globals.c \
../Core/Src/LPF.c \
../Core/Src/TaskController.c \
../Core/Src/TaskDiagnostics.c \
../Core/Src/TaskMotor.c \
../Core/Src/TaskPower.c \
../Core/Src/TaskRemote.c \
../Core/Src/TaskSensorData.c \
../Core/Src/TaskTune.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/printf.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_hal_timebase_tim.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c 

OBJS += \
./Core/Src/Debug.o \
./Core/Src/Globals.o \
./Core/Src/LPF.o \
./Core/Src/TaskController.o \
./Core/Src/TaskDiagnostics.o \
./Core/Src/TaskMotor.o \
./Core/Src/TaskPower.o \
./Core/Src/TaskRemote.o \
./Core/Src/TaskSensorData.o \
./Core/Src/TaskTune.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/printf.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_hal_timebase_tim.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o 

C_DEPS += \
./Core/Src/Debug.d \
./Core/Src/Globals.d \
./Core/Src/LPF.d \
./Core/Src/TaskController.d \
./Core/Src/TaskDiagnostics.d \
./Core/Src/TaskMotor.d \
./Core/Src/TaskPower.d \
./Core/Src/TaskRemote.d \
./Core/Src/TaskSensorData.d \
./Core/Src/TaskTune.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/printf.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_hal_timebase_tim.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Debug.cyclo ./Core/Src/Debug.d ./Core/Src/Debug.o ./Core/Src/Debug.su ./Core/Src/Globals.cyclo ./Core/Src/Globals.d ./Core/Src/Globals.o ./Core/Src/Globals.su ./Core/Src/LPF.cyclo ./Core/Src/LPF.d ./Core/Src/LPF.o ./Core/Src/LPF.su ./Core/Src/TaskController.cyclo ./Core/Src/TaskController.d ./Core/Src/TaskController.o ./Core/Src/TaskController.su ./Core/Src/TaskDiagnostics.cyclo ./Core/Src/TaskDiagnostics.d ./Core/Src/TaskDiagnostics.o ./Core/Src/TaskDiagnostics.su ./Core/Src/TaskMotor.cyclo ./Core/Src/TaskMotor.d ./Core/Src/TaskMotor.o ./Core/Src/TaskMotor.su ./Core/Src/TaskPower.cyclo ./Core/Src/TaskPower.d ./Core/Src/TaskPower.o ./Core/Src/TaskPower.su ./Core/Src/TaskRemote.cyclo ./Core/Src/TaskRemote.d ./Core/Src/TaskRemote.o ./Core/Src/TaskRemote.su ./Core/Src/TaskSensorData.cyclo ./Core/Src/TaskSensorData.d ./Core/Src/TaskSensorData.o ./Core/Src/TaskSensorData.su ./Core/Src/TaskTune.cyclo ./Core/Src/TaskTune.d ./Core/Src/TaskTune.o ./Core/Src/TaskTune.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/printf.cyclo ./Core/Src/printf.d ./Core/Src/printf.o ./Core/Src/printf.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_hal_timebase_tim.cyclo ./Core/Src/stm32f7xx_hal_timebase_tim.d ./Core/Src/stm32f7xx_hal_timebase_tim.o ./Core/Src/stm32f7xx_hal_timebase_tim.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su

.PHONY: clean-Core-2f-Src

