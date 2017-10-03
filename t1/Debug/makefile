################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Static_Code/System/subdir.mk
-include Sources/subdir.mk
-include SDK/platform/utilities/src/subdir.mk
-include SDK/platform/system/src/interrupt/subdir.mk
-include SDK/platform/system/src/clock/MKL43Z4/subdir.mk
-include SDK/platform/system/src/clock/subdir.mk
-include SDK/platform/osa/src/subdir.mk
-include SDK/platform/hal/src/uart/subdir.mk
-include SDK/platform/hal/src/sim/MKL43Z4/subdir.mk
-include SDK/platform/hal/src/rtc/subdir.mk
-include SDK/platform/hal/src/port/subdir.mk
-include SDK/platform/hal/src/osc/subdir.mk
-include SDK/platform/hal/src/mcglite/subdir.mk
-include SDK/platform/hal/src/lpuart/subdir.mk
-include SDK/platform/drivers/src/rtc/subdir.mk
-include SDK/platform/devices/MKL43Z4/startup/gcc/subdir.mk
-include SDK/platform/devices/MKL43Z4/startup/subdir.mk
-include SDK/platform/devices/subdir.mk
-include Generated_Code/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_SIZE += \
t1.siz \


# All Target
all: t1.elf secondary-outputs

# Tool invocations
t1.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m0plus -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -T "/home/fellipe/workspace.kds/t1/Project_Settings/Linker_Files/ProcessorExpert.ld" -Xlinker --gc-sections -L"/home/fellipe/workspace.kds/t1/Project_Settings/Linker_Files" -Wl,-Map,"t1.map" -specs=nosys.specs -specs=nano.specs -Xlinker -z -Xlinker muldefs -o "t1.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

t1.siz: t1.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "t1.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(C++_DEPS)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(CC_DEPS)$(CPP_DEPS)$(CXX_DEPS)$(C_UPPER_DEPS)$(S_UPPER_DEPS) t1.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
