/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Auto generated file 
 */

#include "ti_drivers_open_close.h"
#include <kernel/dpl/DebugP.h>

void Drivers_open(void)
{

    Drivers_adcOpen();
    Drivers_epwmOpen();
    Drivers_eqepOpen();
    Drivers_gpioIntXbarOpen();
    Drivers_intXbarOpen();
    Drivers_uartOpen();
}

void Drivers_close(void)
{
    Drivers_uartClose();
}

void Drivers_adcOpen()
{
	/* CONFIG_ADC3 initialization */

	/* Configures the analog-to-digital converter module prescaler. */
	ADC_setPrescaler(CONFIG_ADC3_BASE_ADDR, ADC_CLK_DIV_4_0);
	/* Configures the analog-to-digital converter resolution and signal mode. */
	ADC_setMode(CONFIG_ADC3_BASE_ADDR, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	/* Sets the priority mode of the SOCs. */
	ADC_setSOCPriority(CONFIG_ADC3_BASE_ADDR, ADC_PRI_ALL_ROUND_ROBIN);

	/* Start of Conversion 0 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 0, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 1 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 1, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 2 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 2, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 3 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 3, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 4 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 4, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 4, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 5 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 5, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 5, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 6 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 6, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 6, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 7 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 7, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 7, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 8 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 8, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 8, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 9 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 9, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 9, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 10 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 10, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 10, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 11 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 11, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 11, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 12 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 12, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 12, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 13 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 13, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 13, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 14 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 14, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 14, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 15 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 15, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 15, ADC_INT_SOC_TRIGGER_NONE);

	/* Powers up the analog-to-digital converter core. */
	ADC_enableConverter(CONFIG_ADC3_BASE_ADDR);
    /* Delay for ADC to power up. */
    ClockP_usleep(500);
	/* Sets the timing of the end-of-conversion pulse */
	ADC_setInterruptPulseMode(CONFIG_ADC3_BASE_ADDR, ADC_PULSE_END_OF_CONV);


	/* ADC Interrupt 1 Configuration */
	/* Enables an ADC interrupt source. */
	ADC_enableInterrupt(CONFIG_ADC3_BASE_ADDR, 0);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC3_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC3_BASE_ADDR, 0);

	/* ADC Interrupt 2 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC3_BASE_ADDR, 1);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC3_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC3_BASE_ADDR, 1);

	/* ADC Interrupt 3 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC3_BASE_ADDR, 2);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC3_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC3_BASE_ADDR, 2);

	/* ADC Interrupt 4 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC3_BASE_ADDR, 3);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC3_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC3_BASE_ADDR, 3);


	/* Post Processing Block 1 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC3_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC3_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC3_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC3_BASE_ADDR, 0, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC3_BASE_ADDR, 0, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC3_BASE_ADDR, 0);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC3_BASE_ADDR, 0, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC3_BASE_ADDR, 0);

	/* Post Processing Block 2 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC3_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC3_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC3_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC3_BASE_ADDR, 1, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC3_BASE_ADDR, 1, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC3_BASE_ADDR, 1);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC3_BASE_ADDR, 1, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC3_BASE_ADDR, 1);

	/* Post Processing Block 3 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC3_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC3_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC3_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC3_BASE_ADDR, 2, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC3_BASE_ADDR, 2, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC3_BASE_ADDR, 2);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC3_BASE_ADDR, 2, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC3_BASE_ADDR, 2);

	/* Post Processing Block 4 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC3_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC3_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC3_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC3_BASE_ADDR, 3, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC3_BASE_ADDR, 3, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC3_BASE_ADDR, 3);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC3_BASE_ADDR, 3, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC3_BASE_ADDR, 3);

	/* Set SOC burst mode. */
	ADC_setBurstModeConfig(CONFIG_ADC3_BASE_ADDR, ADC_TRIGGER_SW_ONLY, 1);
	/* Disables SOC burst mode. */
	ADC_disableBurstMode(CONFIG_ADC3_BASE_ADDR);
}

void Drivers_epwmOpen(void)
{
	/* PWM13_B_INHA initialization */

	/* Time Base */
	EPWM_setEmulationMode(PWM13_B_INHA_BASE_ADDR, EPWM_EMULATION_FREE_RUN);
	EPWM_setClockPrescaler(PWM13_B_INHA_BASE_ADDR, EPWM_CLOCK_DIVIDER_4, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(PWM13_B_INHA_BASE_ADDR, 25000);
	EPWM_disableGlobalLoadRegisters(PWM13_B_INHA_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(PWM13_B_INHA_BASE_ADDR, EPWM_COUNTER_MODE_UP_DOWN);
	EPWM_setCountModeAfterSync(PWM13_B_INHA_BASE_ADDR, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
	EPWM_disablePhaseShiftLoad(PWM13_B_INHA_BASE_ADDR);
	EPWM_setPhaseShift(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_enableSyncOutPulseSource(PWM13_B_INHA_BASE_ADDR, EPWM_SYNC_OUT_PULSE_ON_ALL);
	EPWM_setSyncInPulseSource(PWM13_B_INHA_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_DISABLE);
	EPWM_setOneShotSyncOutTrigger(PWM13_B_INHA_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(PWM13_B_INHA_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 0);
	EPWM_disableGlobalLoadRegisters(PWM13_B_INHA_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(PWM13_B_INHA_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 25000);
	EPWM_disableGlobalLoadRegisters(PWM13_B_INHA_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(PWM13_B_INHA_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(PWM13_B_INHA_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(PWM13_B_INHA_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(PWM13_B_INHA_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(PWM13_B_INHA_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(PWM13_B_INHA_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(PWM13_B_INHA_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
	EPWM_setTripZoneAction(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZB_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZB_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZA_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZA_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(PWM13_B_INHA_BASE_ADDR);
	EPWM_enableTripZoneSignals(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(PWM13_B_INHA_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(PWM13_B_INHA_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(PWM13_B_INHA_BASE_ADDR);
	EPWM_setDigitalCompareBlankingEvent(PWM13_B_INHA_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, EPWM_DC_WINDOW_START_TBCTR_PERIOD);
	EPWM_setDigitalCompareWindowOffset(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(PWM13_B_INHA_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(PWM13_B_INHA_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(PWM13_B_INHA_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(PWM13_B_INHA_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(PWM13_B_INHA_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(PWM13_B_INHA_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(PWM13_B_INHA_BASE_ADDR);
	EPWM_setValleyTriggerSource(PWM13_B_INHA_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(PWM13_B_INHA_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(PWM13_B_INHA_BASE_ADDR);
	EPWM_setValleySWDelayValue(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(PWM13_B_INHA_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(PWM13_B_INHA_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(PWM13_B_INHA_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(PWM13_B_INHA_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(PWM13_B_INHA_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(PWM13_B_INHA_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(PWM13_B_INHA_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(PWM13_B_INHA_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(PWM13_B_INHA_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(PWM13_B_INHA_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(PWM13_B_INHA_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(PWM13_B_INHA_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(PWM13_B_INHA_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(PWM13_B_INHA_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(PWM13_B_INHA_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(PWM13_B_INHA_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayMode(PWM13_B_INHA_BASE_ADDR, EPWM_DB_RED, false);
	EPWM_setDeadBandDelayMode(PWM13_B_INHA_BASE_ADDR, EPWM_DB_FED, false);
	EPWM_setDeadBandOutputSwapMode(PWM13_B_INHA_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(PWM13_B_INHA_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(PWM13_B_INHA_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(PWM13_B_INHA_BASE_ADDR, 0);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(PWM13_B_INHA_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_setDeadBandCounterClock(PWM13_B_INHA_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(PWM13_B_INHA_BASE_ADDR);
	EPWM_setChopperDutyCycle(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_setChopperFreq(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(PWM13_B_INHA_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_enableInterrupt(PWM13_B_INHA_BASE_ADDR);
	EPWM_setInterruptSource(PWM13_B_INHA_BASE_ADDR, EPWM_INT_TBCTR_ZERO, EPWM_INT_TBCTR_ZERO);
	EPWM_setInterruptEventCount(PWM13_B_INHA_BASE_ADDR, 1);
	EPWM_disableInterruptEventCountInit(PWM13_B_INHA_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(PWM13_B_INHA_BASE_ADDR, 0);
	
	EPWM_disableADCTrigger(PWM13_B_INHA_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerSource(PWM13_B_INHA_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(PWM13_B_INHA_BASE_ADDR, EPWM_SOC_A, 0);
	EPWM_disableADCTriggerEventCountInit(PWM13_B_INHA_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(PWM13_B_INHA_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(PWM13_B_INHA_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerSource(PWM13_B_INHA_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(PWM13_B_INHA_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(PWM13_B_INHA_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(PWM13_B_INHA_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(PWM13_B_INHA_BASE_ADDR);
    EPWM_disableSplitXCMP(PWM13_B_INHA_BASE_ADDR);
	EPWM_allocAXCMP(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    EPWM_setXCMPLoadMode(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XMINMAX_ACTIVE, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XMINMAX_SHADOW1, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XMINMAX_SHADOW2, 0);
    EPWM_setXCMPRegValue(PWM13_B_INHA_BASE_ADDR, EPWM_XMINMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(PWM13_B_INHA_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(PWM13_B_INHA_BASE_ADDR);
    EPWM_setDiodeEmulationMode(PWM13_B_INHA_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(PWM13_B_INHA_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(PWM13_B_INHA_BASE_ADDR, EPWM_DE_TRIPL, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0);
    EPWM_configureDiodeEmulationTripSources(PWM13_B_INHA_BASE_ADDR, EPWM_DE_TRIPH, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0);
    EPWM_selectDiodeEmulationPWMsignal(PWM13_B_INHA_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(PWM13_B_INHA_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(PWM13_B_INHA_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(PWM13_B_INHA_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(PWM13_B_INHA_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(PWM13_B_INHA_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(PWM13_B_INHA_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(PWM13_B_INHA_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(PWM13_B_INHA_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(PWM13_B_INHA_BASE_ADDR);
    HRPWM_setMEPControlMode(PWM13_B_INHA_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(PWM13_B_INHA_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(PWM13_B_INHA_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(PWM13_B_INHA_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(PWM13_B_INHA_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(PWM13_B_INHA_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(PWM13_B_INHA_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(PWM13_B_INHA_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(PWM13_B_INHA_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(PWM13_B_INHA_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(PWM13_B_INHA_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(PWM13_B_INHA_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(PWM13_B_INHA_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(PWM13_B_INHA_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(PWM13_B_INHA_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(PWM13_B_INHA_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(PWM13_B_INHA_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(PWM13_B_INHA_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(PWM13_B_INHA_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(PWM13_B_INHA_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

	/* Global Load */
	EPWM_disableGlobalLoad(PWM13_B_INHA_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(PWM13_B_INHA_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(PWM13_B_INHA_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(PWM13_B_INHA_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(PWM13_B_INHA_BASE_ADDR, 0);
}

void Drivers_eqepOpen()
{
	/* CONFIG_EQEP0 initialization */
	/* Disable, clear all flags and interrupts */
	EQEP_disableInterrupt(CONFIG_EQEP0_BASE_ADDR,
		(EQEP_INT_GLOBAL     		|
		EQEP_INT_POS_CNT_ERROR		|
		EQEP_INT_PHASE_ERROR    	|
		EQEP_INT_DIR_CHANGE    		|
		EQEP_INT_WATCHDOG          	|
		EQEP_INT_UNDERFLOW         	|
		EQEP_INT_OVERFLOW        	|
		EQEP_INT_POS_COMP_READY    	|
		EQEP_INT_POS_COMP_MATCH   	|
		EQEP_INT_STROBE_EVNT_LATCH	|
		EQEP_INT_INDEX_EVNT_LATCH 	|
		EQEP_INT_UNIT_TIME_OUT   	|
		EQEP_INT_QMA_ERROR));
	EQEP_clearInterruptStatus(CONFIG_EQEP0_BASE_ADDR,
		(EQEP_INT_GLOBAL     		|
		EQEP_INT_POS_CNT_ERROR		|
		EQEP_INT_PHASE_ERROR    	|
		EQEP_INT_DIR_CHANGE    		|
		EQEP_INT_WATCHDOG          	|
		EQEP_INT_UNDERFLOW         	|
		EQEP_INT_OVERFLOW        	|
		EQEP_INT_POS_COMP_READY    	|
		EQEP_INT_POS_COMP_MATCH   	|
		EQEP_INT_STROBE_EVNT_LATCH	|
		EQEP_INT_INDEX_EVNT_LATCH 	|
		EQEP_INT_UNIT_TIME_OUT   	|
		EQEP_INT_QMA_ERROR));
	/* Set the strobe input source of the eQEP module */
	EQEP_setStrobeSource(CONFIG_EQEP0_BASE_ADDR,EQEP_STROBE_FROM_GPIO);
	/* Sets the polarity of the eQEP module's input signals */
	EQEP_setInputPolarity(CONFIG_EQEP0_BASE_ADDR,false,false,false,false);
	/* Configures eQEP module's quadrature decoder unit */
	EQEP_setDecoderConfig(CONFIG_EQEP0_BASE_ADDR, (EQEP_CONFIG_UP_COUNT | EQEP_CONFIG_1X_RESOLUTION | EQEP_CONFIG_NO_SWAP));
	/* Set the emulation mode of the eQEP module */
	EQEP_setEmulationMode(CONFIG_EQEP0_BASE_ADDR,EQEP_EMULATIONMODE_RUNFREE);
	/* Configures eQEP module position counter unit */
	EQEP_setPositionCounterConfig(CONFIG_EQEP0_BASE_ADDR,EQEP_POSITION_RESET_MAX_POS,116000);
	/* Sets the current encoder position */
	EQEP_setPosition(CONFIG_EQEP0_BASE_ADDR,0);
	/* Enables the eQEP module unit timer */
	EQEP_enableUnitTimer(CONFIG_EQEP0_BASE_ADDR,2000000);
	/* Disables the eQEP module watchdog timer */
	EQEP_disableWatchdog(CONFIG_EQEP0_BASE_ADDR);
	/* Configures the quadrature modes in which the position count can be latched */
	EQEP_setLatchMode(CONFIG_EQEP0_BASE_ADDR,(EQEP_LATCH_UNIT_TIME_OUT|EQEP_LATCH_RISING_STROBE|EQEP_LATCH_RISING_INDEX));
	/* Set the quadrature mode adapter (QMA) module mode */
	EQEP_setQMAModuleMode(CONFIG_EQEP0_BASE_ADDR,EQEP_QMA_MODE_BYPASS);
	/* Disable Direction Change During Index */
	EQEP_disableDirectionChangeDuringIndex(CONFIG_EQEP0_BASE_ADDR);
	/* Configures the mode in which the position counter is initialized */
	EQEP_setPositionInitMode(CONFIG_EQEP0_BASE_ADDR,(EQEP_INIT_DO_NOTHING));
	/* Sets the software initialization of the encoder position counter */
	EQEP_setSWPositionInit(CONFIG_EQEP0_BASE_ADDR,true);
	/* Sets the init value for the encoder position counter */
	EQEP_setInitialPosition(CONFIG_EQEP0_BASE_ADDR,0);
	/* Enables the eQEP module */
	EQEP_enableModule(CONFIG_EQEP0_BASE_ADDR);
	/* Configures eQEP module edge-capture unit */
	EQEP_setCaptureConfig(CONFIG_EQEP0_BASE_ADDR,EQEP_CAPTURE_CLK_DIV_128,EQEP_UNIT_POS_EVNT_DIV_8);
	/* Enables the eQEP module edge-capture unit */
	EQEP_enableCapture(CONFIG_EQEP0_BASE_ADDR);
	/* Configures eQEP module position-compare unit */
	EQEP_setCompareConfig(CONFIG_EQEP0_BASE_ADDR,(EQEP_COMPARE_NO_SYNC_OUT | EQEP_COMPARE_NO_SHADOW), 0, 0);
	/* Enables the eQEP module position-compare unit */
	EQEP_enableCompare(CONFIG_EQEP0_BASE_ADDR);

	EQEP_SourceSelect sourceSelect;
	sourceSelect.sourceA = EQEP_SOURCE_DEVICE_PIN;
	sourceSelect.sourceB = EQEP_SOURCE_DEVICE_PIN;
	sourceSelect.sourceIndex = EQEP_SOURCE_ZERO;
	EQEP_selectSource(CONFIG_EQEP0_BASE_ADDR, sourceSelect);

}
void Drivers_gpioIntXbarOpen()
{
    /*
    * GPIO INTERRUPT XBAR
    */
    SOC_xbarSelectGpioIntrXbarInputSource(CSL_GPIO_INTR_XBAR_U_BASE, GPIO_INT_XBAR_VIM_MODULE0_0, GPIO_INT_XBAR_GPIO_0_BANK_INTR_2);
    SOC_xbarSelectGpioIntrXbarInputSource(CSL_GPIO_INTR_XBAR_U_BASE, GPIO_INT_XBAR_VIM_MODULE0_1, GPIO_INT_XBAR_GPIO_0_BANK_INTR_4);
}
void Drivers_intXbarOpen()
{
    /* INT XBAR */
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, 0, ( INT_XBAR_EPWM13_INT ), 0, 0, 0, 0, 0, 0);
}
/*
 * UART
 */

/* UART Driver handles */
UART_Handle gUartHandle[CONFIG_UART_NUM_INSTANCES];

#include <drivers/uart/v0/dma/uart_dma.h>

/* UART Driver Parameters */
UART_Params gUartParams[CONFIG_UART_NUM_INSTANCES] =
{
    {
        .baudRate           = 115200,
        .dataLength         = UART_LEN_8,
        .stopBits           = UART_STOPBITS_1,
        .parityType         = UART_PARITY_NONE,
        .readMode           = UART_TRANSFER_MODE_BLOCKING,
        .readReturnMode     = UART_READ_RETURN_MODE_FULL,
        .writeMode          = UART_TRANSFER_MODE_BLOCKING,
        .readCallbackFxn    = NULL,
        .writeCallbackFxn   = NULL,
        .hwFlowControl      = FALSE,
        .hwFlowControlThr   = UART_RXTRIGLVL_16,
        .transferMode       = UART_CONFIG_MODE_INTERRUPT,
        .skipIntrReg         = FALSE,
        .uartDmaIndex = -1,
        .intrNum            = 38U,
        .intrPriority       = 4U,
        .operMode           = UART_OPER_MODE_16X,
        .rxTrigLvl          = UART_RXTRIGLVL_8,
        .txTrigLvl          = UART_TXTRIGLVL_32,
        .rxEvtNum           = 0U,
        .txEvtNum           = 0U,

    },
};

void Drivers_uartOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = UART_open(instCnt, &gUartParams[instCnt]);
        if(NULL == gUartHandle[instCnt])
        {
            DebugP_logError("UART open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_uartClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_uartClose(void)
{
    uint32_t instCnt;
    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        if(gUartHandle[instCnt] != NULL)
        {
            UART_close(gUartHandle[instCnt]);
            gUartHandle[instCnt] = NULL;
        }
    }

    return;
}

