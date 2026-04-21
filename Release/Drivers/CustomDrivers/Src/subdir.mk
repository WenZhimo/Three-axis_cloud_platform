################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CustomDrivers/Src/drv_pwmMotors.c 

OBJS += \
./Drivers/CustomDrivers/Src/drv_pwmMotors.o 

C_DEPS += \
./Drivers/CustomDrivers/Src/drv_pwmMotors.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CustomDrivers/Src/%.o Drivers/CustomDrivers/Src/%.su Drivers/CustomDrivers/Src/%.cyclo: ../Drivers/CustomDrivers/Src/%.c Drivers/CustomDrivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CustomDrivers-2f-Src

clean-Drivers-2f-CustomDrivers-2f-Src:
	-$(RM) ./Drivers/CustomDrivers/Src/drv_pwmMotors.cyclo ./Drivers/CustomDrivers/Src/drv_pwmMotors.d ./Drivers/CustomDrivers/Src/drv_pwmMotors.o ./Drivers/CustomDrivers/Src/drv_pwmMotors.su

.PHONY: clean-Drivers-2f-CustomDrivers-2f-Src

