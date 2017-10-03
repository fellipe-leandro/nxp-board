################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Static_Code/System/PE_low_level_init.c 

OBJS += \
./Static_Code/System/PE_low_level_init.o 

C_DEPS += \
./Static_Code/System/PE_low_level_init.d 


# Each subdirectory must supply rules for building sources it contributes
Static_Code/System/%.o: ../Static_Code/System/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -D"FSL_OSA_BM_TIMER_CONFIG=2" -D"CPU_MKL43Z256VLH4" -I"/home/fellipe/workspace.kds/t1/SDK/platform/hal/inc" -I"/home/fellipe/workspace.kds/t1/SDK/platform/hal/src/sim/MKL43Z4" -I"/home/fellipe/workspace.kds/t1/SDK/platform/system/src/clock/MKL43Z4" -I"/home/fellipe/workspace.kds/t1/SDK/platform/system/inc" -I"/home/fellipe/workspace.kds/t1/SDK/platform/osa/inc" -I"/home/fellipe/workspace.kds/t1/SDK/platform/CMSIS/Include" -I"/home/fellipe/workspace.kds/t1/SDK/platform/devices" -I"/home/fellipe/workspace.kds/t1/SDK/platform/devices/MKL43Z4/include" -I"/home/fellipe/workspace.kds/t1/SDK/platform/utilities/src" -I"/home/fellipe/workspace.kds/t1/SDK/platform/utilities/inc" -I"/home/fellipe/workspace.kds/t1/SDK/platform/devices/MKL43Z4/startup" -I"/home/fellipe/workspace.kds/t1/Generated_Code/SDK/platform/devices/MKL43Z4/startup" -I"/home/fellipe/workspace.kds/t1/Sources" -I"/home/fellipe/workspace.kds/t1/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


