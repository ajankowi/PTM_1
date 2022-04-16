################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include .metadata/.plugins/org.eclipse.cdt.make.core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
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
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
AVR-testowy.lss \

FLASH_IMAGE += \
AVR-testowy.hex \

EEPROM_IMAGE += \
AVR-testowy.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: AVR-testowy.elf secondary-outputs

# Tool invocations
AVR-testowy.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,AVR-testowy.map -mmcu=atmega32 -o "AVR-testowy.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

AVR-testowy.lss: AVR-testowy.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S AVR-testowy.elf  >"AVR-testowy.lss"
	@echo 'Finished building: $@'
	@echo ' '

AVR-testowy.hex: AVR-testowy.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex AVR-testowy.elf  "AVR-testowy.hex"
	@echo 'Finished building: $@'
	@echo ' '

AVR-testowy.eep: AVR-testowy.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex AVR-testowy.elf  "AVR-testowy.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: AVR-testowy.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega32 AVR-testowy.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) AVR-testowy.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents

-include ../makefile.targets
