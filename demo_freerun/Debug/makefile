################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ti_cgt_tiarmclang_1.3.1.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./demo_freerun.o" \
"./syscfg/ti_dpl_config.o" \
"./syscfg/ti_drivers_config.o" \
"./syscfg/ti_drivers_open_close.o" \
"./syscfg/ti_pinmux_config.o" \
"./syscfg/ti_power_clock_config.o" \
"./syscfg/ti_board_config.o" \
"./syscfg/ti_board_open_close.o" \
"./syscfg/ti_enet_config.o" \
"./syscfg/ti_enet_open_close.o" \
"./main.o" \
"../linker.cmd" \
$(GEN_CMDS__FLAG) \
-Wl,-lnortos.am263x.r5f.ti-arm-clang.debug.lib \
-Wl,-ldrivers.am263x.r5f.ti-arm-clang.debug.lib \
-Wl,-lboard.am263x.r5f.ti-arm-clang.debug.lib \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
demo_freerun.out \

EXE_OUTPUTS__QUOTED += \
"demo_freerun.out" \

BIN_OUTPUTS += \
demo_freerun.hex \

BIN_OUTPUTS__QUOTED += \
"demo_freerun.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "demo_freerun.out"

# Tool invocations
demo_freerun.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/bin/tiarmclang.exe" -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mlittle-endian -mthumb -DSOC_AM263X -D_DEBUG_=1 -gstrict-dwarf -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -Wl,-m"epwm_hr_duty_cycle.Debug.map" -Wl,-i"C:/ti/mcu_plus_sdk_am263x_08_03_01_05/source/kernel/nortos/lib" -Wl,-i"C:/ti/mcu_plus_sdk_am263x_08_03_01_05/source/drivers/lib" -Wl,-i"C:/ti/mcu_plus_sdk_am263x_08_03_01_05/source/board/lib" -Wl,-i"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/lib" -Wl,--reread_libs -Wl,--diag_suppress=10063 -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="demo_freerun_linkInfo.xml" -Wl,--ram_model -o "demo_freerun.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

demo_freerun.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: Arm Hex Utility'
	"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/bin/tiarmhex.exe" --diag_wrap=off -o "demo_freerun.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "demo_freerun.o" "syscfg\ti_dpl_config.o" "syscfg\ti_drivers_config.o" "syscfg\ti_drivers_open_close.o" "syscfg\ti_pinmux_config.o" "syscfg\ti_power_clock_config.o" "syscfg\ti_board_config.o" "syscfg\ti_board_open_close.o" "syscfg\ti_enet_config.o" "syscfg\ti_enet_open_close.o" "main.o" 
	-$(RM) "demo_freerun.d" "syscfg\ti_dpl_config.d" "syscfg\ti_drivers_config.d" "syscfg\ti_drivers_open_close.d" "syscfg\ti_pinmux_config.d" "syscfg\ti_power_clock_config.d" "syscfg\ti_board_config.d" "syscfg\ti_board_open_close.d" "syscfg\ti_enet_config.d" "syscfg\ti_enet_open_close.d" "main.d" 
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-$(MAKE) -C C:\Users\isaac\workspace_v12\demo_freerun -f makefile_ccs_bootimage_gen OUTNAME=demo_freerun PROFILE=Debug MCU_PLUS_SDK_PATH=C:/ti/mcu_plus_sdk_am263x_08_03_01_05 CG_TOOL_ROOT=C:/ti/ti_cgt_tiarmclang_1.3.1.LTS CCS_INSTALL_DIR=C:\ti\ccs1200\ccs CCS_IDE_MODE=desktop
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

