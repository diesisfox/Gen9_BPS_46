################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Can_Processor.c \
../Src/LTC68041.c \
../Src/can.c \
../Src/freertos.c \
../Src/main.c \
../Src/mcp3909.c \
../Src/nodeMiscHelpers.c \
../Src/serial.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_hal_timebase_TIM.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/Can_Processor.o \
./Src/LTC68041.o \
./Src/can.o \
./Src/freertos.o \
./Src/main.o \
./Src/mcp3909.o \
./Src/nodeMiscHelpers.o \
./Src/serial.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_hal_timebase_TIM.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/Can_Processor.d \
./Src/LTC68041.d \
./Src/can.d \
./Src/freertos.d \
./Src/main.d \
./Src/mcp3909.d \
./Src/nodeMiscHelpers.d \
./Src/serial.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_hal_timebase_TIM.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F446xx -I"/Users/jamesliu/Development/STM32/Gen9_BPS_46/Inc" -I"/Users/jamesliu/Development/STM32/Gen9_BPS_46/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/Users/jamesliu/Development/STM32/Gen9_BPS_46/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/Users/jamesliu/Development/STM32/Gen9_BPS_46/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/Users/jamesliu/Development/STM32/Gen9_BPS_46/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/jamesliu/Development/STM32/Gen9_BPS_46/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/Users/jamesliu/Development/STM32/Gen9_BPS_46/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/Users/jamesliu/Development/STM32/Gen9_BPS_46/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


