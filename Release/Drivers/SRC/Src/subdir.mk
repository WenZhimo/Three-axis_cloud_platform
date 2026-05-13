################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SRC/Src/MargAHRS.c \
../Drivers/SRC/Src/computeMotorCommands.c \
../Drivers/SRC/Src/config.c \
../Drivers/SRC/Src/evvgcCF.c \
../Drivers/SRC/Src/fastTrig.c \
../Drivers/SRC/Src/firstOrderFilter.c \
../Drivers/SRC/Src/pid.c 

OBJS += \
./Drivers/SRC/Src/MargAHRS.o \
./Drivers/SRC/Src/computeMotorCommands.o \
./Drivers/SRC/Src/config.o \
./Drivers/SRC/Src/evvgcCF.o \
./Drivers/SRC/Src/fastTrig.o \
./Drivers/SRC/Src/firstOrderFilter.o \
./Drivers/SRC/Src/pid.o 

C_DEPS += \
./Drivers/SRC/Src/MargAHRS.d \
./Drivers/SRC/Src/computeMotorCommands.d \
./Drivers/SRC/Src/config.d \
./Drivers/SRC/Src/evvgcCF.d \
./Drivers/SRC/Src/fastTrig.d \
./Drivers/SRC/Src/firstOrderFilter.d \
./Drivers/SRC/Src/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SRC/Src/%.o Drivers/SRC/Src/%.su Drivers/SRC/Src/%.cyclo: ../Drivers/SRC/Src/%.c Drivers/SRC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CustomDrivers/Inc -I../Drivers/SRC/Inc -I../Drivers/BGC -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-SRC-2f-Src

clean-Drivers-2f-SRC-2f-Src:
	-$(RM) ./Drivers/SRC/Src/MargAHRS.cyclo ./Drivers/SRC/Src/MargAHRS.d ./Drivers/SRC/Src/MargAHRS.o ./Drivers/SRC/Src/MargAHRS.su ./Drivers/SRC/Src/computeMotorCommands.cyclo ./Drivers/SRC/Src/computeMotorCommands.d ./Drivers/SRC/Src/computeMotorCommands.o ./Drivers/SRC/Src/computeMotorCommands.su ./Drivers/SRC/Src/config.cyclo ./Drivers/SRC/Src/config.d ./Drivers/SRC/Src/config.o ./Drivers/SRC/Src/config.su ./Drivers/SRC/Src/evvgcCF.cyclo ./Drivers/SRC/Src/evvgcCF.d ./Drivers/SRC/Src/evvgcCF.o ./Drivers/SRC/Src/evvgcCF.su ./Drivers/SRC/Src/fastTrig.cyclo ./Drivers/SRC/Src/fastTrig.d ./Drivers/SRC/Src/fastTrig.o ./Drivers/SRC/Src/fastTrig.su ./Drivers/SRC/Src/firstOrderFilter.cyclo ./Drivers/SRC/Src/firstOrderFilter.d ./Drivers/SRC/Src/firstOrderFilter.o ./Drivers/SRC/Src/firstOrderFilter.su ./Drivers/SRC/Src/pid.cyclo ./Drivers/SRC/Src/pid.d ./Drivers/SRC/Src/pid.o ./Drivers/SRC/Src/pid.su

.PHONY: clean-Drivers-2f-SRC-2f-Src

