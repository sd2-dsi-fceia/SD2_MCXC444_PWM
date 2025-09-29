################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../device/system_MCXC444.c 

C_DEPS += \
./device/system_MCXC444.d 

OBJS += \
./device/system_MCXC444.o 


# Each subdirectory must supply rules for building sources it contributes
device/%.o: ../device/%.c device/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_MCXC444VLH -DCPU_MCXC444VLH_cm0plus -DMCUXPRESSO_SDK -DPRINTF_ADVANCED_ENABLE=1 -DFRDM_MCXC444 -DFREEDOM -DSDK_DEBUGCONSOLE=1 -DMCUX_META_BUILD -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\drivers" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\CMSIS" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\CMSIS\m-profile" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\device" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\device\periph2" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\utilities" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\utilities\str" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\utilities\debug_console_lite" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\component\uart" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\source" -I"C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\board" -O0 -fno-common -g3 -gdwarf-4 -c -ffunction-sections -fdata-sections -fno-builtin -imacros "C:\Users\guido\Documents\MCUXpressoIDE_24.12.148\workspace\SD2_MCXC444_PWM\source\mcux_config.h" -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-device

clean-device:
	-$(RM) ./device/system_MCXC444.d ./device/system_MCXC444.o

.PHONY: clean-device

