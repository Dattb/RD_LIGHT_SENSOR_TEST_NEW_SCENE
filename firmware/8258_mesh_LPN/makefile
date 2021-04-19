################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include vendor/user_app/subdir.mk
-include vendor/mesh_lpn/subdir.mk
-include vendor/common/subdir.mk
-include vendor/common/nl_api/subdir.mk
-include vendor/common/mi_api/subdir.mk
-include vendor/common/mi_api/mijia_ble_api/subdir.mk
-include vendor/common/mi_api/mi_vendor/subdir.mk
-include vendor/common/mi_api/libs/third_party/pt/subdir.mk
-include vendor/common/mi_api/libs/third_party/micro-ecc/subdir.mk
-include vendor/common/mi_api/libs/third_party/mbedtls/subdir.mk
-include vendor/Rang_Dong/subdir.mk
-include proj/mcu_spec/subdir.mk
-include proj/mcu/subdir.mk
-include proj/drivers/subdir.mk
-include proj/common/subdir.mk
-include proj/app/subdir.mk
-include drivers/8258/subdir.mk
-include boot/8258/subdir.mk
-include 8269_mesh_master_dongle/vendor/Rang_Dong/subdir.mk
-include 8269_mesh_master_dongle/vendor/8267_master_kma_dongle/subdir.mk
-include 8269_mesh_master_dongle/subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LST += \
8258_mesh_LPN.lst \

FLASH_IMAGE += \

SIZEDUMMY += \
sizedummy \


# All Target
all: pre-build main-build

# Main-build Target
main-build: RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE.elf secondary-outputs

# Tool invocations
RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: TC32 C Linker'
	tc32-elf-ld --gc-sections -L"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_LIGHT_SENSOR_TEST_NEW_SCENE\firmware\proj_lib" -T ../boot.link -o"RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

8258_mesh_LPN.lst: RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE.elf
	@echo 'Invoking: TC32 Create Extended Listing'
	tc32-elf-objdump -x -D -l -S RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE.elf  >"8258_mesh_LPN.lst"
	@echo 'Finished building: $@'
	@echo ' '

: RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE.elf
	@echo 'Create Flash image (binary format)'
	tc32-elf-objcopy RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE.elf
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE.elf
	@echo 'Invoking: Print Size'
	tc32-elf-size -t RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(LST)$(SIZEDUMMY) RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE.elf
	-@echo ' '

pre-build:
	-"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_LIGHT_SENSOR_TEST_NEW_SCENE\firmware/getver.sh"
	-@echo ' '

post-build:
	-"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_LIGHT_SENSOR_TEST_NEW_SCENE\firmware/tl_check_fw.sh"   8258_mesh_LPN   RD_LPN_LIGHT_SENSOR_TEST_NEW_SCENE
	-@echo ' '

secondary-outputs: $(LST) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY: main-build pre-build post-build

-include ../makefile.targets
