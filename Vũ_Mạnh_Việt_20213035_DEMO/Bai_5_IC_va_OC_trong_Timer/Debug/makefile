################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := Bai_5_IC_va_OC_trong_Timer
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Bai_5_IC_va_OC_trong_Timer.elf \

MAP_FILES += \
Bai_5_IC_va_OC_trong_Timer.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Bai_5_IC_va_OC_trong_Timer.list \

OBJCOPY_HEX += \
Bai_5_IC_va_OC_trong_Timer.hex \

OBJCOPY_BIN += \
Bai_5_IC_va_OC_trong_Timer.bin \


# All Target
all: main-build

# Main-build Target
main-build: Bai_5_IC_va_OC_trong_Timer.elf secondary-outputs

# Tool invocations
Bai_5_IC_va_OC_trong_Timer.elf Bai_5_IC_va_OC_trong_Timer.map: $(OBJS) $(USER_OBJS) C:\Users\FPT\OneDrive\ -\ Hanoi\ University\ of\ Science\ and\ Technology\STM32\Bai_5_IC_va_OC_trong_Timer\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Bai_5_IC_va_OC_trong_Timer.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\FPT\OneDrive - Hanoi University of Science and Technology\STM32\Bai_5_IC_va_OC_trong_Timer\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Bai_5_IC_va_OC_trong_Timer.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Bai_5_IC_va_OC_trong_Timer.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Bai_5_IC_va_OC_trong_Timer.list"
	@echo 'Finished building: $@'
	@echo ' '

Bai_5_IC_va_OC_trong_Timer.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Bai_5_IC_va_OC_trong_Timer.hex"
	@echo 'Finished building: $@'
	@echo ' '

Bai_5_IC_va_OC_trong_Timer.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Bai_5_IC_va_OC_trong_Timer.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Bai_5_IC_va_OC_trong_Timer.bin Bai_5_IC_va_OC_trong_Timer.elf Bai_5_IC_va_OC_trong_Timer.hex Bai_5_IC_va_OC_trong_Timer.list Bai_5_IC_va_OC_trong_Timer.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
