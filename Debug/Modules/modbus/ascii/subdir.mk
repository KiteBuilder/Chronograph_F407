################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modules/modbus/ascii/mbascii.c 

OBJS += \
./Modules/modbus/ascii/mbascii.o 

C_DEPS += \
./Modules/modbus/ascii/mbascii.d 


# Each subdirectory must supply rules for building sources it contributes
Modules/modbus/ascii/%.o Modules/modbus/ascii/%.su Modules/modbus/ascii/%.cyclo: ../Modules/modbus/ascii/%.c Modules/modbus/ascii/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Modules/modbus/ascii -I../Modules/modbus/functions -I../Modules/modbus/include -I../Modules/modbus/port -I../Modules/modbus/rtu -I../Modules/modbus/tcp -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Modules-2f-modbus-2f-ascii

clean-Modules-2f-modbus-2f-ascii:
	-$(RM) ./Modules/modbus/ascii/mbascii.cyclo ./Modules/modbus/ascii/mbascii.d ./Modules/modbus/ascii/mbascii.o ./Modules/modbus/ascii/mbascii.su

.PHONY: clean-Modules-2f-modbus-2f-ascii

