################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MC3479/Src/MC3479.cpp 

OBJS += \
./MC3479/Src/MC3479.o 

CPP_DEPS += \
./MC3479/Src/MC3479.d 


# Each subdirectory must supply rules for building sources it contributes
MC3479/Src/%.o MC3479/Src/%.su: ../MC3479/Src/%.cpp MC3479/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"S:/Capstone/Firmware/Active Stabalizing Mug/MC3479/Inc" -I"S:/Capstone/Firmware/Active Stabalizing Mug/MP6543H/Inc" -I"S:/Capstone/Firmware/Active Stabalizing Mug/ControlSystem/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MC3479-2f-Src

clean-MC3479-2f-Src:
	-$(RM) ./MC3479/Src/MC3479.d ./MC3479/Src/MC3479.o ./MC3479/Src/MC3479.su

.PHONY: clean-MC3479-2f-Src

