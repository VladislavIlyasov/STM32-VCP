################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Logic/CircBuff.c 

OBJS += \
./Core/Logic/CircBuff.o 

C_DEPS += \
./Core/Logic/CircBuff.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Logic/%.o Core/Logic/%.su: ../Core/Logic/%.c Core/Logic/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Logic

clean-Core-2f-Logic:
	-$(RM) ./Core/Logic/CircBuff.d ./Core/Logic/CircBuff.o ./Core/Logic/CircBuff.su

.PHONY: clean-Core-2f-Logic

