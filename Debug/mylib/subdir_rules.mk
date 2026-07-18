################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
mylib/%.o: ../mylib/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/TI_CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"F:/TI_CCS/TI_Project/A1_UpdataMotor" -I"F:/TI_CCS/TI_Project/A1_UpdataMotor/Debug" -I"F:/TI_CCS/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/TI_CCS/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"mylib/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


