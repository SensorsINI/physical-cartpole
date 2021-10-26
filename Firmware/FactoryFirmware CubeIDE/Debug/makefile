################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/USER/subdir.mk
-include Src/SYSTEM/usart/subdir.mk
-include Src/SYSTEM/sys/subdir.mk
-include Src/SYSTEM/delay/subdir.mk
-include Src/HARDWARE/TIMER/subdir.mk
-include Src/HARDWARE/OLED/subdir.mk
-include Src/HARDWARE/MOTOR/subdir.mk
-include Src/HARDWARE/LED/subdir.mk
-include Src/HARDWARE/KEY/subdir.mk
-include Src/HARDWARE/EXTI/subdir.mk
-include Src/HARDWARE/ENCODER/subdir.mk
-include Src/HARDWARE/ADC/subdir.mk
-include Src/BALANCE/show/subdir.mk
-include Src/BALANCE/DataScope_DP/subdir.mk
-include Src/BALANCE/CONTROL/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Original CartPole CubeIDE
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Original\ CartPole\ CubeIDE.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Original\ CartPole\ CubeIDE.list \

OBJCOPY_BIN += \
Original\ CartPole\ CubeIDE.bin \


# All Target
all: main-build

# Main-build Target
main-build: Original\ CartPole\ CubeIDE.elf secondary-outputs

# Tool invocations
Original\ CartPole\ CubeIDE.elf: $(OBJS) $(USER_OBJS) /Users/jeromejeannin/Dropbox/physical-cartpole/Firmware/Original\ CartPole\ CubeIDE/STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Original CartPole CubeIDE.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/Users/jeromejeannin/Dropbox/physical-cartpole/Firmware/Original CartPole CubeIDE/STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Original CartPole CubeIDE.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Original\ CartPole\ CubeIDE.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Original CartPole CubeIDE.list"
	@echo 'Finished building: $@'
	@echo ' '

Original\ CartPole\ CubeIDE.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Original CartPole CubeIDE.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SIZE_OUTPUT)$(OBJDUMP_LIST)$(EXECUTABLES)$(OBJS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS)$(OBJCOPY_BIN) "Original CartPole CubeIDE.elf"
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
