################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/Rang_Dong/RD_Light_Sensor.c \
../vendor/Rang_Dong/RD_light.c \
../vendor/Rang_Dong/Sensor_Flash.c 

OBJS += \
./vendor/Rang_Dong/RD_Light_Sensor.o \
./vendor/Rang_Dong/RD_light.o \
./vendor/Rang_Dong/Sensor_Flash.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/Rang_Dong/%.o: ../vendor/Rang_Dong/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"C:\Users\Dat UTC\Documents\Rang_Dong\CODE_LIGHT_SENSOR\firmware" -D__PROJECT_8267_MASTER_KMA_DONGLE__=1 -DCHIP_TYPE=CHIP_TYPE_8269 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


