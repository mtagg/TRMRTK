################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ControlSystem/Src/controlSystem.cpp 

OBJS += \
./ControlSystem/Src/controlSystem.o 

CPP_DEPS += \
./ControlSystem/Src/controlSystem.d 


# Each subdirectory must supply rules for building sources it contributes
ControlSystem/Src/%.o ControlSystem/Src/%.su: ../ControlSystem/Src/%.cpp ControlSystem/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"S:/Capstone/Firmware/Active Stabalizing Mug/MC3479/Inc" -I"S:/Capstone/Firmware/Active Stabalizing Mug/MP6543H/Inc" -I"S:/Capstone/Firmware/Active Stabalizing Mug/ControlSystem/Inc" -I"S:/Capstone/Firmware/Active Stabalizing Mug/AS5048/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ControlSystem-2f-Src

clean-ControlSystem-2f-Src:
	-$(RM) ./ControlSystem/Src/controlSystem.d ./ControlSystem/Src/controlSystem.o ./ControlSystem/Src/controlSystem.su

.PHONY: clean-ControlSystem-2f-Src

