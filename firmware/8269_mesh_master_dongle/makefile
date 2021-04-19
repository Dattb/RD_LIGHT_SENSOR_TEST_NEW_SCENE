################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include vendor/Rang_Dong/subdir.mk
-include vendor/8267_master_kma_dongle/subdir.mk
-include proj/mcu_spec/subdir.mk
-include proj/mcu/subdir.mk
-include proj/drivers/subdir.mk
-include proj/common/subdir.mk
-include proj/app/subdir.mk
-include drivers/8258/subdir.mk
-include boot/8258/subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LST += \
8267_master_kma_dongle.lst \

FLASH_IMAGE += \

SIZEDUMMY += \
sizedummy \


# All Target
all: pre-build main-build

# Main-build Target
main-build: RD_LPN_LIGHT_SENSOR.elf secondary-outputs

# Tool invocations
RD_LPN_LIGHT_SENSOR.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: TC32 C Linker'
	tc32-elf-ld --gc-sections -L"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_LIGHT_SENSOR\firmware\proj_lib" -T ../boot.link -o"RD_LPN_LIGHT_SENSOR.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

8267_master_kma_dongle.lst: RD_LPN_LIGHT_SENSOR.elf
	@echo 'Invoking: TC32 Create Extended Listing'
	tc32-elf-objdump -x -D -l -S RD_LPN_LIGHT_SENSOR.elf  >"8267_master_kma_dongle.lst"
	@echo 'Finished building: $@'
	@echo ' '

: RD_LPN_LIGHT_SENSOR.elf
	@echo 'Create Flash image (binary format)'
	tc32-elf-objcopy RD_LPN_LIGHT_SENSOR.elf
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: RD_LPN_LIGHT_SENSOR.elf
	@echo 'Invoking: Print Size'
	tc32-elf-size -t RD_LPN_LIGHT_SENSOR.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(LST)$(SIZEDUMMY) RD_LPN_LIGHT_SENSOR.elf
	-@echo ' '

pre-build:
	-"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_LIGHT_SENSOR\firmware/getver.sh"
	-@echo ' '

post-build:
	-"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_LIGHT_SENSOR\firmware/tl_check_fw.sh"   8269_mesh_master_dongle   RD_LPN_LIGHT_SENSOR
	-@echo ' '

secondary-outputs: $(LST) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY: main-build pre-build post-build

-include ../makefile.targets
