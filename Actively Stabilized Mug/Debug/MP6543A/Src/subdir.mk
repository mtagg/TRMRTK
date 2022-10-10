################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MP6543A/Src/MP6543A.cpp 

OBJS += \
./MP6543A/Src/MP6543A.o 

CPP_DEPS += \
./MP6543A/Src/MP6543A.d 


# Each subdirectory must supply rules for building sources it contributes
MP6543A/Src/%.o MP6543A/Src/%.su: ../MP6543A/Src/%.cpp MP6543A/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"S:/Capstone/Firmware/Actively Stabilized Mug/MC349/Inc" -I"S:/Capstone/Firmware/Actively Stabilized Mug/MP6543A/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MP6543A-2f-Src

clean-MP6543A-2f-Src:
	-$(RM) ./MP6543A/Src/MP6543A.d ./MP6543A/Src/MP6543A.o ./MP6543A/Src/MP6543A.su

.PHONY: clean-MP6543A-2f-Src

