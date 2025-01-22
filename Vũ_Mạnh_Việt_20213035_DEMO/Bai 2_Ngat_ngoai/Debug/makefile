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


BUILD_ARTIFACT_NAME := Bai 2_Ngat_ngoai
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Bai\ 2_Ngat_ngoai.elf \

MAP_FILES += \
Bai\ 2_Ngat_ngoai.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Bai\ 2_Ngat_ngoai.list \

OBJCOPY_HEX += \
Bai\ 2_Ngat_ngoai.hex \

OBJCOPY_BIN += \
Bai\ 2_Ngat_ngoai.bin \


# All Target
all: main-build

# Main-build Target
main-build: Bai\ 2_Ngat_ngoai.elf secondary-outputs

# Tool invocations
Bai\ 2_Ngat_ngoai.elf Bai\ 2_Ngat_ngoai.map: $(OBJS) $(USER_OBJS) C:\Users\FPT\OneDrive\ -\ Hanoi\ University\ of\ Science\ and\ Technology\STM32\Bai\ 2_Ngat_ngoai\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Bai 2_Ngat_ngoai.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\FPT\OneDrive - Hanoi University of Science and Technology\STM32\Bai 2_Ngat_ngoai\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Bai 2_Ngat_ngoai.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Bai\ 2_Ngat_ngoai.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Bai 2_Ngat_ngoai.list"
	@echo 'Finished building: $@'
	@echo ' '

Bai\ 2_Ngat_ngoai.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Bai 2_Ngat_ngoai.hex"
	@echo 'Finished building: $@'
	@echo ' '

Bai\ 2_Ngat_ngoai.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Bai 2_Ngat_ngoai.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Bai\ 2_Ngat_ngoai.bin Bai\ 2_Ngat_ngoai.elf Bai\ 2_Ngat_ngoai.hex Bai\ 2_Ngat_ngoai.list Bai\ 2_Ngat_ngoai.map Bai\\ 2_Ngat_ngoai.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
