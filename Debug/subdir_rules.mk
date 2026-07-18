################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/TI_CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"F:/TI_CCS/TI_Project/A1_UpdataMotor" -I"F:/TI_CCS/TI_Project/A1_UpdataMotor/Debug" -I"F:/TI_CCS/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/TI_CCS/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-498547290: ../empty.syscfg
	@echo 'SysConfig - building file: "$<"'
	"F:/TI_CCS/sysconfig_1.26.2/sysconfig_cli.bat" -s "F:/TI_CCS/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "F:/TI_CCS/TI_Project/A1_UpdataMotor/empty.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-498547290 ../empty.syscfg
device.opt: build-498547290
device.cmd.genlibs: build-498547290
ti_msp_dl_config.c: build-498547290
ti_msp_dl_config.h: build-498547290
Event.dot: build-498547290

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/TI_CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"F:/TI_CCS/TI_Project/A1_UpdataMotor" -I"F:/TI_CCS/TI_Project/A1_UpdataMotor/Debug" -I"F:/TI_CCS/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/TI_CCS/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: F:/TI_CCS/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/TI_CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"F:/TI_CCS/TI_Project/A1_UpdataMotor" -I"F:/TI_CCS/TI_Project/A1_UpdataMotor/Debug" -I"F:/TI_CCS/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/TI_CCS/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


