################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include MP6543B/Src/Debug/MP6543B/Src/subdir.mk
-include MP6543B/Src/Debug/MC3479/Src/subdir.mk
-include MP6543B/Src/Debug/Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include MP6543B/Src/Debug/Core/Startup/subdir.mk
-include MP6543B/Src/Debug/Core/Src/subdir.mk
-include MP6543B/Src/subdir.mk
-include MC3479/Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Active Stabalizing Mug
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Active\ Stabalizing\ Mug.elf \

MAP_FILES += \
Active\ Stabalizing\ Mug.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Active\ Stabalizing\ Mug.list \


# All Target
all: main-build

# Main-build Target
main-build: Active\ Stabalizing\ Mug.elf secondary-outputs

# Tool invocations
Active\ Stabalizing\ Mug.elf Active\ Stabalizing\ Mug.map: $(OBJS) $(USER_OBJS) S:\Capstone\Firmware\Active\ Stabalizing\ Mug\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-g++ -o "Active Stabalizing Mug.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"S:\Capstone\Firmware\Active Stabalizing Mug\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Active Stabalizing Mug.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Active\ Stabalizing\ Mug.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Active Stabalizing Mug.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Active\ Stabalizing\ Mug.elf Active\ Stabalizing\ Mug.list Active\ Stabalizing\ Mug.map Active\\ Stabalizing\\ Mug.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
