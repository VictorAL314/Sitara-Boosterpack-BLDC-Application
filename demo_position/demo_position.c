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

// from epwm_hr_duty_cycle.c
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

// from adc_soc_software.c
#include <kernel/dpl/ClockP.h>
#include <drivers/adc.h>

// From gpio_led_blink.c
#include <kernel/dpl/AddrTranslateP.h>

/*
 * INSTRUCTIONS
 * Connect J8 80 (PWM) to EQEP0A
 * Run and observe EQEP counter incrementing
 *
 */

/* APP run time in seconds */
#define APP_EPWM_RUN_TIME    (60U)
/* FIXME : To be removed after syscfg integration */
#define APP_INT_IS_PULSE    (1U)

#define ADC_NUM_BITS    12
#define ADC_MAX_VAL     ((1 << ADC_NUM_BITS) - 1)


/* Global variables and objects */
static HwiP_Object  gEpwmHwiObject;
static SemaphoreP_Object  gEpwmSyncSemObject;

/* Function Prototypes */
static void App_epwmIntrISR(void *handle);
static void App_eqepIntrISR(void *handle); // Not working, interrupts not triggering or getting caught (not sure which)
static void App_rotate_degrees_unsigned_poll(uint32_t degrees); // Untested
static void App_rotate_degrees_signed_poll(int degrees); // Works
static void App_rotate_degrees_signed_precalc(int degrees); // Not working

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))


// PWM base address
uint32_t pwm13BaseAddress = PWM13_B_INHA_BASE_ADDR;
uint32_t time_base_period = 25000;
uint32_t cmpB_value = 0;

// ADC base addresses
// Potentiometer: ADC3_AIN3
uint32_t adc3BaseAddress = CONFIG_ADC3_BASE_ADDR;
uint32_t adc_pot = ADC_SOC_NUMBER3;
uint32_t adc3ResultBaseAddress = CONFIG_ADC3_RESULT_BASE_ADDR;

// EQEP base address
uint32_t gEqepBaseAddr = CONFIG_EQEP0_BASE_ADDR;
#define EQEP_COUNT_PER_REV 116000;

// GPIO base addresses and direction
#define DIR_CW (1U)
#define DIR_CCW (0U)
uint32_t enableBaseAddress;
uint32_t brakeBaseAddress;
uint32_t directionBaseAddress;




int end_flag = 0;
uint32_t end_pos = 0;
void demo_position_main(void *args)
{
    int32_t  status;
    volatile uint32_t  numIsrCnt = (600);
    HwiP_Params  hwiPrms;
    HwiP_Params  eqepPrms;


    float duty_cycle;
    //uint32_t ADC_NUM_BITS = 12;
    //uint32_t ADC_MAX_VAL = (1 << ADC_NUM_BITS) - 1;
    uint32_t adc_in; // Value read in from the ADC

    /* GPIO addresses and initialization */
    // GPIO base addresses
    enableBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ENABLE_BASE_ADDR);
    brakeBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_INLC_BRAKE_BASE_ADDR);
    directionBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_INHC_DIRECTION_BASE_ADDR);

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Starting ...\r\n");

    /* Configure GPIO directions and values */
    /* Set enable high */
    GPIO_pinWriteHigh(enableBaseAddress, GPIO_ENABLE_PIN);
    GPIO_setDirMode(enableBaseAddress, GPIO_ENABLE_PIN, GPIO_ENABLE_DIR);
    /* Set brake high */
    GPIO_pinWriteHigh(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
    GPIO_setDirMode(brakeBaseAddress, GPIO_INLC_BRAKE_PIN, GPIO_INLC_BRAKE_DIR);
    /* Set direction high */
    GPIO_pinWriteHigh(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
    GPIO_setDirMode(directionBaseAddress, GPIO_INHC_DIRECTION_PIN, GPIO_INHC_DIRECTION_DIR);

    status = SemaphoreP_constructCounting(&gEpwmSyncSemObject, 0, numIsrCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    /* Integrate with Syscfg */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_epwmIntrISR;
    /* Integrate with Syscfg */
    hwiPrms.isPulse     = APP_INT_IS_PULSE;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    HwiP_Params_init(&eqepPrms);
    /* Integrate with Syscfg */
    eqepPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    eqepPrms.callback    = &App_eqepIntrISR;
    /* Integrate with Syscfg */
    eqepPrms.isPulse     = APP_INT_IS_PULSE;

    EPWM_clearEventTriggerInterruptFlag(pwm13BaseAddress);

    //while(numIsrCnt > 0)

    while(!end_flag)
    {

        char str[20];
        float angle_float;
        int angle;

        DebugP_log("Enter an integer angle in degrees. Positive for counter-clockwise, negative for clockwise. Type \"done\" to stop. \r\n");
        scanf("%[^\n]", str);

        while ((getchar()) != '\n');

        if(strcmp(str, "done") == 0) {

            end_flag = 1;
        }
        else {

            angle = atoi(str);
            App_rotate_degrees_signed_poll(angle);
        }


    }


    // Reset PWM duty cycle to 0
    EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, time_base_period);
    EPWM_disableInterrupt(pwm13BaseAddress);
    EPWM_clearEventTriggerInterruptFlag(pwm13BaseAddress);     /* Clear any pending interrupts if any */
    HwiP_destruct(&gEpwmHwiObject);
    SemaphoreP_destruct(&gEpwmSyncSemObject);


    /* Clear and disable interrupt */
    ADC_disableInterrupt(adc3BaseAddress, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(adc3BaseAddress, ADC_INT_NUMBER1);
    /* Power down the ADC */
    ADC_disableConverter(adc3BaseAddress);

    DebugP_log("Closing drivers.\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_epwmIntrISR(void *handle)
{
    volatile bool status;

    status = EPWM_getEventTriggerInterruptStatus(pwm13BaseAddress);
    if(status == true)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
        EPWM_clearEventTriggerInterruptFlag(pwm13BaseAddress);
    }

    return;
}

static void App_eqepIntrISR(void *handle)
{
    end_flag = 1;
    end_pos = EQEP_getPositionLatch(gEqepBaseAddr);
    if((EQEP_getInterruptStatus(gEqepBaseAddr) & (EQEP_INT_POS_COMP_MATCH | EQEP_INT_POS_COMP_READY)) != 0U)
        {
            end_flag = 1;
            end_pos = EQEP_getPositionLatch(gEqepBaseAddr);

            EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_POS_COMP_MATCH | EQEP_INT_POS_COMP_READY);
        }

    return;
}


static void App_rotate_degrees_unsigned_poll(uint32_t degrees)
{
    // For now, ignore direction.
    // Poll the EQEP position, wait for it to reach desired position without overhead
    // (Next attempt, generate an interrupt when desired position is met. That way
    // ADC can still adjust speed during the movement)
    //
    // EQEP_count / EQEP_COUNT_PER_REV = percentage of a complete rotation
    // EQEP_count / EQEP_COUNT_PER_REV * 360 = degrees of rotation
    //
    // => (degrees / 360) * EQEP_COUNT_PER_REV = target_count

    // Set EQEP compare value
    uint32_t target_count = (float) degrees / 360 * EQEP_COUNT_PER_REV;

    // Set PWM duty cycle to 1
    EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, 1);

    // Wait for EQEP compare value
    while(EQEP_getPositionLatch(gEqepBaseAddr) < target_count);

    // Set PWM duty cycle to 0
    EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, time_base_period);

}

static void App_rotate_degrees_signed_poll(int degrees)
{
    // Consider direction
    // Poll the EQEP position, wait for it to reach desired position without overhead
    // (Next attempt, generate an interrupt when desired position is met. That way
    // ADC can still adjust speed during the movement)
    //
    // EQEP_count / EQEP_COUNT_PER_REV = percentage of a complete rotation
    // EQEP_count / EQEP_COUNT_PER_REV * 360 = degrees of rotation
    //
    // => (degrees / 360) * EQEP_COUNT_PER_REV = target_count

    // Let's arbitrarily define positive direction (CCW) as 0, negative direction (CW) as 1
    // Direction HIGH = Clockwise = negative
    // It seems that in UP count mode, counter only increases.

    // Define positive direction as counter clockwise (CCW) and negative as clockwise (CW)
    uint32_t direction = (degrees > 0) ? DIR_CCW : DIR_CW;
    uint32_t degrees_pos;
    uint32_t end_position;

    if(direction == DIR_CCW) {
        GPIO_pinWriteLow(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
        degrees_pos = degrees;
    }
    else {
        // Clockwise (negative) direction
        GPIO_pinWriteHigh(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
        degrees_pos = -1 * degrees;
    }

    DebugP_log("RESETTING EQEP POSITION\r\n");
    // Reset EQEP counter
    EQEP_setPosition(gEqepBaseAddr, 0);

    // Set EQEP compare value
    uint32_t target_count = (float) degrees_pos / 360 * EQEP_COUNT_PER_REV;


    DebugP_log("Starting position: %d\r\n", EQEP_getPositionLatch(gEqepBaseAddr));
    DebugP_log("Target angle: %d, position = %d\r\n", degrees, target_count);


    // Set PWM duty cycle to 1
    EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, 0);

    // Wait for EQEP compare value
    while(EQEP_getPosition(gEqepBaseAddr) < target_count);

    // Set PWM duty cycle to 0
    EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, time_base_period);


    int error_signed;
    int error_unsigned;
    float error_rev;

    end_position = EQEP_getPositionLatch(gEqepBaseAddr);
    error_unsigned = (end_position > target_count) ? (end_position - target_count) : (target_count - end_position);
    error_signed = end_position - target_count;
    error_rev = (float) (error_unsigned) / EQEP_COUNT_PER_REV;
    DebugP_log("Ending position: %d\r\n", end_position);
    DebugP_log("Error = %d encoder counts\r\n", error_signed);
    DebugP_log("Error = %.4f%% revolution\r\n\r\n", 100*error_rev);


    // Reset EQEP counter
    EQEP_setPosition(gEqepBaseAddr, 0);

}

static void App_rotate_degrees_signed_precalc(int degrees)
{
    // Consider direction
    // Poll the EQEP position, wait for it to reach desired position without overhead
    // Pre-calculate the list of PWM compare values to make a smooth ramp-up, ramp-down in speed
    //
    // EQEP_count / EQEP_COUNT_PER_REV = percentage of a complete rotation
    // EQEP_count / EQEP_COUNT_PER_REV * 360 = degrees of rotation
    //
    // => (degrees / 360) * EQEP_COUNT_PER_REV = target_count

    // Let's arbitrarily define positive direction (CCW) as 0, negative direction (CW) as 1
    // Direction HIGH = Clockwise = negative
    // It seems that in UP count mode, counter only increases.

    // Define positive direction as counter clockwise (CCW) and negative as clockwise (CW)
    uint32_t direction = (degrees > 0) ? DIR_CCW : DIR_CW;
    uint32_t degrees_pos;
    uint32_t end_position;

    if(direction == DIR_CCW) {
        GPIO_pinWriteLow(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
        degrees_pos = degrees;
    }
    else {
        // Clockwise (negative) direction
        GPIO_pinWriteHigh(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
        degrees_pos = -1 * degrees;
    }


    // Set EQEP compare value
    uint32_t target_count = (float) degrees_pos / 360 * EQEP_COUNT_PER_REV;


    // Reset EQEP counter
    EQEP_setPosition(gEqepBaseAddr, 0);

    DebugP_log("RESETTING EQEP POSITION\r\n");
    DebugP_log("Starting position: %d\r\n", EQEP_getPositionLatch(gEqepBaseAddr));
    DebugP_log("Target angle: %d, position = %d\r\n", degrees, target_count);






    DebugP_log("RESETTING EQEP POSITION\r\n");
    DebugP_log("Starting position: %d\r\n", EQEP_getPositionLatch(gEqepBaseAddr));
    DebugP_log("Target angle: %d, target position = %d\r\n", degrees, target_count);

    // Define 3 regions: speed up, constant, slow down
    // Speed may be varied based on load, but we know position counts for sure.
    // So, for the first 1/4, increase the speed for every counted position
    // For the middle 1/2, go at full speed
    // Then, for the last 1/4, decrease speed
    uint32_t end_speedup = target_count / 4;
    uint32_t end_constant = 3*target_count / 4;
    uint32_t ramp_duration = target_count / 4;

    // Remember, 1 - cmp_val/time_base_counter = actual duty_cycle of the motor
    uint32_t cmp_value = time_base_period; // Start out at duty_cycle = 0


    uint32_t count;

    //EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, time_base_period / 2);

    // Speed up
    count = 0;
    while(EQEP_getPositionLatch(gEqepBaseAddr) < end_speedup) {
        cmp_value = MAX(0, cmp_value - end_speedup/8 * count++);
        EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, cmp_value);
    }


    // Constant speed
    // Set PWM duty cycle to 1
    EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, 0);
    while(EQEP_getPositionLatch(gEqepBaseAddr) < end_constant);

    // Slow down
    count = 0;
    while(EQEP_getPositionLatch(gEqepBaseAddr) < target_count) {
        cmp_value = MIN(time_base_period, cmp_value + end_speedup/8 * count++);
        EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, MIN(cmp_value, time_base_period));
    }


    // Set PWM duty cycle to 0
    EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, time_base_period);

    int error;
    float error_rev;

    end_position = EQEP_getPositionLatch(gEqepBaseAddr);
    error = end_position - target_count;
    error_rev = (float) (end_position - target_count) / EQEP_COUNT_PER_REV;
    DebugP_log("Ending position: %d\r\n", end_position);
    DebugP_log("Error = %d encoder counts\r\n", error);
    DebugP_log("Error = %.4f\% revolution\r\n\r\n", error_rev*100);


    // Reset EQEP counter
    EQEP_setPosition(gEqepBaseAddr, 0);

}
