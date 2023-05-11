################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mlittle-endian -mthumb -I"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/include/c" -I"C:/ti/mcu_plus_sdk_am263x_08_03_01_05/source" -DSOC_AM263X -D_DEBUG_=1 -gstrict-dwarf -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/isaac/workspace_v12/demo_position/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-751579601: ../example.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1200/ccs/utils/sysconfig/sysconfig_cli.bat" -s "C:/ti/mcu_plus_sdk_am263x_08_03_01_05/.metadata/product.json" --script "C:/Users/isaac/workspace_v12/demo_position/example.syscfg" --context "r5fss0-0" -o "syscfg" --part AM263x --package ZCZ --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_dpl_config.c: build-751579601 ../example.syscfg
syscfg/ti_dpl_config.h: build-751579601
syscfg/ti_drivers_config.c: build-751579601
syscfg/ti_drivers_config.h: build-751579601
syscfg/ti_drivers_open_close.c: build-751579601
syscfg/ti_drivers_open_close.h: build-751579601
syscfg/ti_pinmux_config.c: build-751579601
syscfg/ti_power_clock_config.c: build-751579601
syscfg/ti_board_config.c: build-751579601
syscfg/ti_board_config.h: build-751579601
syscfg/ti_board_open_close.c: build-751579601
syscfg/ti_board_open_close.h: build-751579601
syscfg/ti_enet_config.c: build-751579601
syscfg/ti_enet_config.h: build-751579601
syscfg/ti_enet_open_close.c: build-751579601
syscfg/ti_enet_open_close.h: build-751579601
syscfg/: build-751579601

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mlittle-endian -mthumb -I"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/include/c" -I"C:/ti/mcu_plus_sdk_am263x_08_03_01_05/source" -DSOC_AM263X -D_DEBUG_=1 -gstrict-dwarf -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/isaac/workspace_v12/demo_position/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


