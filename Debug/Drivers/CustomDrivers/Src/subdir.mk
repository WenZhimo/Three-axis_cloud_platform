################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CustomDrivers/Src/drv_pwmMotors.c \
../Drivers/CustomDrivers/Src/mpu6050.c \
../Drivers/CustomDrivers/Src/mpu6050Calibration.c 

OBJS += \
./Drivers/CustomDrivers/Src/drv_pwmMotors.o \
./Drivers/CustomDrivers/Src/mpu6050.o \
./Drivers/CustomDrivers/Src/mpu6050Calibration.o 

C_DEPS += \
./Drivers/CustomDrivers/Src/drv_pwmMotors.d \
./Drivers/CustomDrivers/Src/mpu6050.d \
./Drivers/CustomDrivers/Src/mpu6050Calibration.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CustomDrivers/Src/%.o Drivers/CustomDrivers/Src/%.su Drivers/CustomDrivers/Src/%.cyclo: ../Drivers/CustomDrivers/Src/%.c Drivers/CustomDrivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"E:/myprojects/SZYT/Three-axis_cloud_platformV2/Drivers/CustomDrivers/Inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/SRC/Inc -I../Drivers/BGC -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CustomDrivers-2f-Src

clean-Drivers-2f-CustomDrivers-2f-Src:
	-$(RM) ./Drivers/CustomDrivers/Src/drv_pwmMotors.cyclo ./Drivers/CustomDrivers/Src/drv_pwmMotors.d ./Drivers/CustomDrivers/Src/drv_pwmMotors.o ./Drivers/CustomDrivers/Src/drv_pwmMotors.su ./Drivers/CustomDrivers/Src/mpu6050.cyclo ./Drivers/CustomDrivers/Src/mpu6050.d ./Drivers/CustomDrivers/Src/mpu6050.o ./Drivers/CustomDrivers/Src/mpu6050.su ./Drivers/CustomDrivers/Src/mpu6050Calibration.cyclo ./Drivers/CustomDrivers/Src/mpu6050Calibration.d ./Drivers/CustomDrivers/Src/mpu6050Calibration.o ./Drivers/CustomDrivers/Src/mpu6050Calibration.su

.PHONY: clean-Drivers-2f-CustomDrivers-2f-Src

