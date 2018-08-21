################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/uart.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/uart.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103x6 -I"/home/usr/workspaceSTM32/test_stm32f103_ECM_ADV/USB_DEVICE" -I"/home/usr/workspaceSTM32/test_stm32f103_ECM_ADV/Core/Inc" -I"/home/usr/workspaceSTM32/test_stm32f103_ECM_ADV/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/usr/workspaceSTM32/test_stm32f103_ECM_ADV/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/usr/workspaceSTM32/test_stm32f103_ECM_ADV/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/usr/workspaceSTM32/test_stm32f103_ECM_ADV/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/usr/workspaceSTM32/test_stm32f103_ECM_ADV/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/usr/workspaceSTM32/test_stm32f103_ECM_ADV/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


