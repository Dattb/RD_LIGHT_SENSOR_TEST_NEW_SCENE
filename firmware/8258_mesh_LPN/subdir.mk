################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../div_mod.S 

OBJS += \
./div_mod.o 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 CC/Assembler'
	tc32-elf-gcc -Xassembler"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_LIGHT_SENSOR_TEST_NEW_SCENE\firmware" -DMCU_STARTUP_8258_RET_16K -D__PROJECT_MESH_LPN__=1 -DCHIP_TYPE=CHIP_TYPE_8258 -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


