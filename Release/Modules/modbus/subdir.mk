################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modules/modbus/mb.c 

OBJS += \
./Modules/modbus/mb.o 

C_DEPS += \
./Modules/modbus/mb.d 


# Each subdirectory must supply rules for building sources it contributes
Modules/modbus/%.o Modules/modbus/%.su Modules/modbus/%.cyclo: ../Modules/modbus/%.c Modules/modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Modules/modbus/ascii -I../Modules/modbus/functions -I../Modules/modbus/include -I../Modules/modbus/port -I../Modules/modbus/rtu -I../Modules/modbus/tcp -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Modules-2f-modbus

clean-Modules-2f-modbus:
	-$(RM) ./Modules/modbus/mb.cyclo ./Modules/modbus/mb.d ./Modules/modbus/mb.o ./Modules/modbus/mb.su

.PHONY: clean-Modules-2f-modbus

