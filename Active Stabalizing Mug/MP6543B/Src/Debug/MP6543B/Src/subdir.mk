################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MP6543B/Src/MP6543B.cpp 

OBJS += \
./MP6543B/Src/MP6543B.o 

CPP_DEPS += \
./MP6543B/Src/MP6543B.d 


# Each subdirectory must supply rules for building sources it contributes
MP6543B/Src/%.o MP6543B/Src/%.su: ../MP6543B/Src/%.cpp MP6543B/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MP6543B-2f-Src

clean-MP6543B-2f-Src:
	-$(RM) ./MP6543B/Src/MP6543B.d ./MP6543B/Src/MP6543B.o ./MP6543B/Src/MP6543B.su

.PHONY: clean-MP6543B-2f-Src

