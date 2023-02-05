################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../AS5048/Src/AS5048A.cpp 

OBJS += \
./AS5048/Src/AS5048A.o 

CPP_DEPS += \
./AS5048/Src/AS5048A.d 


# Each subdirectory must supply rules for building sources it contributes
AS5048/Src/%.o AS5048/Src/%.su: ../AS5048/Src/%.cpp AS5048/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"S:/Capstone/Firmware/Active Stabalizing Mug/MC3479/Inc" -I"S:/Capstone/Firmware/Active Stabalizing Mug/MP6543H/Inc" -I"S:/Capstone/Firmware/Active Stabalizing Mug/ControlSystem/Inc" -I"S:/Capstone/Firmware/Active Stabalizing Mug/AS5048/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-AS5048-2f-Src

clean-AS5048-2f-Src:
	-$(RM) ./AS5048/Src/AS5048A.d ./AS5048/Src/AS5048A.o ./AS5048/Src/AS5048A.su

.PHONY: clean-AS5048-2f-Src

