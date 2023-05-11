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
 * Connect DRV8323RH to the LP-AM263. Ensure that the DRV board is configured
 * for 1xPWM mode.
 *
 * Connect the A channel of the encoder to J24, Pin 1, and make sure the encoder
 * is receiving the necessary supply voltage and ground connections.
 *
 * Connect the direction signal that will be read in from the switch debouncer to J2.11 on the LP
 *
 * Connect the break signal that will be read in from the switch debouncer to FILL PIN HERE on the LP
 *
 */

/* Frequency of PWM output signal in Hz - 1 KHz is selected */
#define APP_EPWM_OUTPUT_FREQ    (10U * 1000U)
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
static void App_gpioDirIntrISR(void *handle);
static void App_gpioBrakeIntrISR(void *handle);
void App_timerIntrISR(void *args);

// PWM base address
uint32_t pwm13BaseAddress = PWM13_B_INHA_BASE_ADDR;
#define TIME_BASE_PERIOD (25000U)

// ADC base addresses
// Potentiometer: ADC3_AIN3
uint32_t adc3BaseAddress = CONFIG_ADC3_BASE_ADDR;
uint32_t adc_pot = ADC_SOC_NUMBER0;
uint32_t adc3ResultBaseAddress = CONFIG_ADC3_RESULT_BASE_ADDR;

/* GPIO addresses and initialization */
// GPIO base addresses
uint32_t enableBaseAddress;
uint32_t brakeBaseAddress;
uint32_t directionBaseAddress;
uint32_t directionIntBaseAddress;
uint32_t brakeIntBaseAddress;
uint32_t directionIntBankNum;
uint32_t brakeIntBankNum;
HwiP_Object         GpioHwiObject;
HwiP_Object         GpioHwiObjectBrake;
/* EQEP BASE ADDR AND CONFIG*/
uint32_t gEqepBaseAddr = CONFIG_EQEP0_BASE_ADDR;
const uint32_t EQEP_MAX_POS = 116000;


// Hardware timer
volatile uint32_t timer_count = 0;
/******************************************
 * eqep_delta is the number of eqep counts since last calculation
 * timer_delta is the number of timer periods since last calculation
 *
 * RPM =
 *
 *   eqep_delta       1             1          10^9 ns    60 s
 * = ----------- *---------*---------------  * ------- * ----------
 *   timer_delta  EQEP_MAX  timer_period (ns)  1 s        1 min
 *
 *   eqep_delta
 * = ---------- * timer_scaler
 *   timer_delta
 *
 * timer_scaler = 10^9 * 60 / (EQEP_MAX_POS * CONFIG_TIMER0_NSEC_PER_TICK_ACTUAL)
 ******************************************/
double timer_scaler_rpm = (double)(1e9 * 60) / (EQEP_MAX_POS * CONFIG_TIMER0_NSEC_PER_TICK_ACTUAL);
double timer_scaler_rps = (double)(1e9) / (EQEP_MAX_POS * CONFIG_TIMER0_NSEC_PER_TICK_ACTUAL);

uint32_t isr_trigger_count = 0;
void demo_freerun_main(void *args)
{
    uint32_t timer_delta, timer_prev = 0;
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);



    int32_t  status;
    volatile uint32_t  runtime_count = (10000);
    volatile uint32_t eqep_count = 0;
    HwiP_Params  hwiPrms; // hardware interrupt for
    HwiP_Params  directionInterrupt; // Hardware interrupt for direction switching

    HwiP_Params  brakeInterrupt; // Hardware interrupt for brake switching

    uint32_t cmpB_value = TIME_BASE_PERIOD;
    float duty_cycle;
    //uint32_t ADC_NUM_BITS = 12;
    //uint32_t ADC_MAX_VAL = (1 << ADC_NUM_BITS) - 1;
    uint32_t adc_in; // Value read in from the ADC


    // Set GPIO addresses
    enableBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ENABLE_BASE_ADDR);
    brakeBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_INLC_BRAKE_BASE_ADDR);
    directionBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_INHC_DIRECTION_BASE_ADDR);
    directionIntBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_DIRECTION_INTERRUPT_BASE_ADDR);
    brakeIntBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_BRAKE_INTERRUPT_BASE_ADDR);


    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EPWM Duty Cycle Test Started ...\r\n");
    DebugP_log("App will wait for 60 seconds (using PWM period ISR) ...\r\n");


    /* Configure GPIO directions and values */
    /* Set enable high */
    GPIO_setDirMode(enableBaseAddress, GPIO_ENABLE_PIN, GPIO_ENABLE_DIR);
    GPIO_pinWriteHigh(enableBaseAddress, GPIO_ENABLE_PIN);

//    /* Set brake high */
//    GPIO_setDirMode(brakeBaseAddress, GPIO_INLC_BRAKE_PIN, GPIO_INLC_BRAKE_DIR);
//    GPIO_pinWriteHigh(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);

    /* Set brake */
    GPIO_setDirMode(brakeBaseAddress, GPIO_INLC_BRAKE_PIN, GPIO_INLC_BRAKE_DIR);
    // Read direction level from the switched pin (interrupt pin)
    // Set the direction on the direction pin
    if(GPIO_pinRead(brakeIntBaseAddress, GPIO_BRAKE_INTERRUPT_PIN) == GPIO_PIN_LOW) {
        GPIO_pinWriteLow(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
    }
    else {
        // curr_direction = GPIO_PIN_HIGH
        GPIO_pinWriteHigh(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
    }



    /* Set direction interrupt as input */
    GPIO_setDirMode(directionIntBaseAddress, GPIO_DIRECTION_INTERRUPT_PIN, GPIO_DIRECTION_INTERRUPT_DIR);
    directionIntBankNum = GPIO_GET_BANK_INDEX(GPIO_DIRECTION_INTERRUPT_PIN);

    /* Set brake interrupt as input */
    GPIO_setDirMode(brakeIntBaseAddress, GPIO_BRAKE_INTERRUPT_PIN, GPIO_BRAKE_INTERRUPT_DIR);
    brakeIntBankNum = GPIO_GET_BANK_INDEX(GPIO_DIRECTION_INTERRUPT_PIN);


    /* Set direction */
    GPIO_setDirMode(directionBaseAddress, GPIO_INHC_DIRECTION_PIN, GPIO_INHC_DIRECTION_DIR);
    // Read direction level from the switched pin (interrupt pin)
    // Set the direction on the direction pin
    if(GPIO_pinOutValueRead(directionIntBaseAddress, GPIO_DIRECTION_INTERRUPT_PIN) == GPIO_PIN_LOW) {
        GPIO_pinWriteLow(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
    }
    else {
        // curr_direction = GPIO_PIN_HIGH
        GPIO_pinWriteHigh(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
    }


    status = SemaphoreP_constructCounting(&gEpwmSyncSemObject, 0, runtime_count);
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

    /* Enable interrupt for direction switch */
    HwiP_Params_init(&directionInterrupt);
    // This interrupt comes from Table 10-15 in the AM263 Technical Reference Manual.
    // Taken from the gpio_input_interrupt example in the SDK, the example uses an alias
    // for this interrupt number. The XBAR is outputting the GPIO43 (pin J2-11 on AM263 LP)
    directionInterrupt.intNum = CSLR_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_14;
    directionInterrupt.callback = &App_gpioDirIntrISR;
    directionInterrupt.isPulse     = APP_INT_IS_PULSE;
    status              = HwiP_construct(&GpioHwiObject, &directionInterrupt);
    DebugP_assert(status == SystemP_SUCCESS );

    /* Enable interrupt for direction switch */
    HwiP_Params_init(&brakeInterrupt);
    // This interrupt comes from Table 10-15 in the AM263 Technical Reference Manual.
    // Taken from the gpio_input_interrupt example in the SDK, the example uses an alias
    // for this interrupt number. The XBAR is outputting the GPIO43 (pin J2-11 on AM263 LP)
    brakeInterrupt.intNum = CSLR_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_15;
    brakeInterrupt.callback = &App_gpioBrakeIntrISR;
    brakeInterrupt.isPulse     = APP_INT_IS_PULSE;
    status              = HwiP_construct(&GpioHwiObjectBrake, &brakeInterrupt);
    DebugP_assert(status == SystemP_SUCCESS );


    EPWM_clearEventTriggerInterruptFlag(pwm13BaseAddress);
    GPIO_bankIntrEnable(directionIntBaseAddress, directionIntBankNum);
    GPIO_bankIntrEnable(brakeIntBaseAddress, brakeIntBankNum);

    uint32_t old_eqep = EQEP_getPositionLatch(gEqepBaseAddr);
    uint32_t new_eqep = 0;
    uint32_t eqep_delta =0;
    double speed_rpm = 0;
    double speed_rps = 0;
    while(runtime_count > 0)
    {

        // Clear ADC interrupt status,
        // then force ADC start-of-conversion
        ADC_clearInterruptStatus(adc3BaseAddress, ADC_INT_NUMBER1);
        ADC_forceSOC(adc3BaseAddress, adc_pot);
        while(ADC_getInterruptStatus(adc3BaseAddress, ADC_INT_NUMBER1) == false)
        {
            /* Wait for the SOC conversion to complete */
        }

        /*
         Read in the ADC result from ADC3, input 3 (potentiometer)
         Calculate the duty cycle. For 12-bit ADC, result is
         between [0, 4095].
         Divide the ADC result by 4095 to get duty cycle.
         PWM signal goes high when the EPWM counter reaches the
         counter compare value, so we need to invert the duty cycle.
         Then, (1-duty_cycle) represents the percentage of the way
         through the entire PWM period that the signal should go high.
         */
        adc_in = ADC_readResult(adc3ResultBaseAddress, adc_pot);
        //DebugP_log("ADC IN: %d\r\n", adc_in);
        duty_cycle = (float)adc_in / ADC_MAX_VAL;
        //DebugP_log("Duty Cycle: %.3f\r\n", duty_cycle);
        cmpB_value = (1 - duty_cycle) * TIME_BASE_PERIOD;
        //DebugP_log("cmpB_value: %d\r\n\r\n", cmpB_value);


        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
        runtime_count--;

        // Update the counter compare value, setting the duty cycle
        EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, cmpB_value);

//        // POLL THE BRAKE PIN
//        if(GPIO_pinRead(brakeIntBaseAddress, GPIO_BRAKE_INTERRUPT_PIN) == GPIO_PIN_LOW) {
//            GPIO_pinWriteLow(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
//        }
//        else {
//            // curr_direction = GPIO_PIN_HIGH
//            GPIO_pinWriteHigh(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
//        }

        if((EQEP_getInterruptStatus(gEqepBaseAddr) & EQEP_INT_UNIT_TIME_OUT) != 0)
            {
                eqep_count++;
                // Get new values
                new_eqep = EQEP_getPositionLatch(gEqepBaseAddr);
                timer_delta = timer_count - timer_prev;


                if (new_eqep >= old_eqep) {
                    eqep_delta = new_eqep - old_eqep;
                }
                else {
                    eqep_delta = EQEP_MAX_POS - old_eqep + new_eqep;
                }
                /*******************************************************
                 * timer_scaler (defined above)
                 * contains timer tick period, ns -> min conversion,
                 * and number of eqep ticks per revolution
                 * Pre-calculating it saves time here.
                 *******************************************************/
                //speed_rpm = timer_scaler_rpm * eqep_delta / timer_delta;
                speed_rps = timer_scaler_rps * eqep_delta / timer_delta;


                /* Print (slow), not counted in timers */
                //DebugP_log("ISR COUNT: %d\r\n", isr_trigger_count);
                //DebugP_log("EQEP delta: %u\r\n", eqep_delta);
                //DebugP_log("Timer delta (10us periods): %u\r\n", timer_delta);
                //DebugP_log("Timer count: %u\r\n", timer_count);
                //DebugP_log("Speed (RPM): %.2f\r\n", speed_rpm);

                if(eqep_count % 10 == 0) {
                    DebugP_log("Speed (RPS): %.2f\r\n\r\n", speed_rps);
                }


                // Update old values
                old_eqep = EQEP_getPositionLatch(gEqepBaseAddr);
                timer_prev = timer_count;
                EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_UNIT_TIME_OUT);
            }



    }


    EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, TIME_BASE_PERIOD);
    EPWM_disableInterrupt(pwm13BaseAddress);
    EPWM_clearEventTriggerInterruptFlag(pwm13BaseAddress);     /* Clear any pending interrupts if any */
    HwiP_destruct(&gEpwmHwiObject);
    HwiP_destruct(&GpioHwiObject);
    HwiP_destruct(&GpioHwiObjectBrake);
    SemaphoreP_destruct(&gEpwmSyncSemObject);


    DebugP_log("All tests have passed!!\r\n");



    /* Clear and disable interrupt */
    ADC_disableInterrupt(adc3BaseAddress, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(adc3BaseAddress, ADC_INT_NUMBER1);
    /* Power down the ADC */
    ADC_disableConverter(adc3BaseAddress);

    DebugP_log("All tests have passed!!\r\n");

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


/***********************************************
 * GPIO Direction input ISR
 *
 * On a level change on the direction pin when the brake is not active,
 * 1. Get the current status of the motor
 *      - Duty cycle
 * 2. Slow down/stop the motor
 * 3. Toggle the direction
 * 4. Resume motor operation
 *
 ***********************************************/
static void App_gpioDirIntrISR(void *handle) {
    /* Get and clear bank interrupt status */
    uint32_t intrStatus = GPIO_getBankIntrStatus(directionIntBaseAddress, directionIntBankNum);
    GPIO_clearBankIntrStatus(directionIntBaseAddress, directionIntBankNum, intrStatus);

    // Disable interrupts
    GPIO_bankIntrDisable(directionIntBaseAddress, directionIntBankNum);

    volatile uint32_t count = 0;
    int curr_direction;

    // ONLY RUN ISR CODE IF CURRENT OUTPUT DIRECTION IS DIFFERENT THAN
    // INPUT INTERRUPT DIRECTION
    if(GPIO_pinOutValueRead(directionBaseAddress, GPIO_INHC_DIRECTION_PIN) != GPIO_pinRead(directionIntBaseAddress, GPIO_DIRECTION_INTERRUPT_PIN)) {
        //Stop if brake is not on, otherwise can just toggle
        if(GPIO_pinOutValueRead(brakeBaseAddress, GPIO_INLC_BRAKE_PIN) == GPIO_PIN_LOW) {
        // Read direction level from the switched pin (interrupt pin)
               curr_direction = GPIO_pinRead(directionIntBaseAddress, GPIO_DIRECTION_INTERRUPT_PIN);
               // Set the direction on the direction pin
               if(curr_direction == GPIO_PIN_LOW) {
                   GPIO_pinWriteLow(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
               }
               else {
                   // curr_direction = GPIO_PIN_HIGH
                   GPIO_pinWriteHigh(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
               }
        }
        else {
        /* Set brake high */
               GPIO_pinWriteLow(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);

               // Wait for motor to full stop
               //while(EQEP_getPositionLatch(gEqepBaseAddr) != EQEP_getPositionLatch(gEqepBaseAddr));
               while(count++ < 50000000);
               //DebugP_log("Changing direction\r\n");

               // Read direction level from the switched pin (interrupt pin)
               curr_direction = GPIO_pinRead(directionIntBaseAddress, GPIO_DIRECTION_INTERRUPT_PIN);
               // Set the direction on the direction pin
               if(curr_direction == GPIO_PIN_LOW) {
                   GPIO_pinWriteLow(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
               }
               else {
                   // curr_direction = GPIO_PIN_HIGH
                   GPIO_pinWriteHigh(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
               }
               GPIO_pinWriteHigh(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
        }

    }

    // Enable interrupts
    GPIO_bankIntrEnable(directionIntBaseAddress, directionIntBankNum);

    isr_trigger_count++;
}
/***********************************************
 * GPIO Brake input ISR
 *
 * On a level change on the brake pin,
 * 1. Read in Brake switch voltage
 * 2. If low, slow down and stop motor
 * 3. If high, start moving motor again
 *
 ***********************************************/
static void App_gpioBrakeIntrISR(void *handle) {
    uint32_t intrStatus = GPIO_getBankIntrStatus(brakeIntBaseAddress, brakeIntBankNum);
    GPIO_clearBankIntrStatus(brakeIntBaseAddress, brakeIntBankNum, intrStatus);

    // Disable interrupts
    GPIO_bankIntrDisable(brakeIntBaseAddress, brakeIntBankNum);
    int brake_status;
    brake_status = GPIO_pinRead(brakeIntBaseAddress, GPIO_BRAKE_INTERRUPT_PIN);
    // Set the brake status on the brake pin
    if(GPIO_pinOutValueRead(brakeBaseAddress, GPIO_INLC_BRAKE_PIN) != GPIO_pinRead(brakeIntBaseAddress, GPIO_BRAKE_INTERRUPT_PIN)) {
        if(brake_status == GPIO_PIN_LOW) {
            GPIO_pinWriteLow(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
        }
        else {
            // curr_direction = GPIO_PIN_HIGH
            GPIO_pinWriteHigh(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
        }
    }

    // Enable interrupts
    GPIO_bankIntrEnable(directionIntBaseAddress, directionIntBankNum);
    isr_trigger_count++;
}
/***********************************************
 * Timer interrupt ISR
 *
 ***********************************************/
void App_timerIntrISR(void *args) {
    timer_count++;
}





//
//
//
//
//
//
///*
// *  FOR COMMENTING OUT
// */
//
//// from epwm_hr_duty_cycle.c
//#include <kernel/dpl/DebugP.h>
//#include <kernel/dpl/SemaphoreP.h>
//#include <kernel/dpl/HwiP.h>
//#include <drivers/epwm.h>
//#include "ti_drivers_config.h"
//#include "ti_drivers_open_close.h"
//#include "ti_board_open_close.h"
//
//// from adc_soc_software.c
//#include <kernel/dpl/ClockP.h>
//#include <drivers/adc.h>
//
//// From gpio_led_blink.c
//#include <kernel/dpl/AddrTranslateP.h>
//
///*
// * INSTRUCTIONS
// * Connect J8 80 (PWM) to EQEP0A
// * Run and observe EQEP counter incrementing
// *
// */
//
///* Frequency of PWM output signal in Hz - 1 KHz is selected */
//#define APP_EPWM_OUTPUT_FREQ    (10U * 1000U)
///* APP run time in seconds */
//#define APP_EPWM_RUN_TIME    (60U)
///* FIXME : To be removed after syscfg integration */
//#define APP_INT_IS_PULSE    (1U)
//
//#define ADC_NUM_BITS    12
//#define ADC_MAX_VAL     ((1 << ADC_NUM_BITS) - 1)
//
//
///* Global variables and objects */
//static HwiP_Object  gEpwmHwiObject;
//static SemaphoreP_Object  gEpwmSyncSemObject;
//
///* Function Prototypes */
//static void App_epwmIntrISR(void *handle);
//static void App_gpioDirIntrISR(void *handle);
//void App_timerIntrISR(void *args);
//
//// PWM base address
//uint32_t pwm13BaseAddress = PWM13_B_INHA_BASE_ADDR;
//#define TIME_BASE_PERIOD (25000U)
//
//// ADC base addresses
//// Potentiometer: ADC3_AIN3
//uint32_t adc3BaseAddress = CONFIG_ADC3_BASE_ADDR;
//uint32_t adc_pot = ADC_SOC_NUMBER3;
//uint32_t adc3ResultBaseAddress = CONFIG_ADC3_RESULT_BASE_ADDR;
//
///* GPIO addresses and initialization */
//// GPIO base addresses
//uint32_t enableBaseAddress;
//uint32_t brakeBaseAddress;
//uint32_t directionBaseAddress;
//uint32_t directionIntBaseAddress;
//uint32_t directionIntBankNum;
//HwiP_Object         GpioHwiObject;
//
///* EQEP BASE ADDR AND CONFIG*/
//uint32_t gEqepBaseAddr = CONFIG_EQEP0_BASE_ADDR;
//const uint32_t EQEP_MAX_POS = 116000;
//
//
//// Hardware timer
//volatile uint32_t timer_count = 0;
///******************************************
// * eqep_delta is the number of eqep counts since last calculation
// * timer_delta is the number of timer periods since last calculation
// *
// * RPM =
// *
// *   eqep_delta       1             1          10^9 ns    60 s
// * = ----------- *---------*---------------  * ------- * ----------
// *   timer_delta  EQEP_MAX  timer_period (ns)  1 s        1 min
// *
// *   eqep_delta
// * = ---------- * timer_scaler
// *   timer_delta
// *
// * timer_scaler = 10^9 * 60 / (EQEP_MAX_POS * CONFIG_TIMER0_NSEC_PER_TICK_ACTUAL)
// ******************************************/
//double timer_scaler_rpm = (double)(1e9 * 60) / (EQEP_MAX_POS * CONFIG_TIMER0_NSEC_PER_TICK_ACTUAL);
//double timer_scaler_rps = (double)(1e9) / (EQEP_MAX_POS * CONFIG_TIMER0_NSEC_PER_TICK_ACTUAL);
//
//uint32_t isr_trigger_count = 0;
//void mount_1x_eqep_newboard_adc_gpioint_speed_main(void *args)
//{
//    uint32_t timer_delta, timer_prev = 0;
//    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
//
//
//
//    int32_t  status;
//    volatile uint32_t  runtime_count = (600);
//    HwiP_Params  hwiPrms; // hardware interrupt for
//    HwiP_Params  directionInterrupt; // Hardware interrupt for direction switching
//
//    uint32_t cmpB_value = TIME_BASE_PERIOD;
//    float duty_cycle;
//    //uint32_t ADC_NUM_BITS = 12;
//    //uint32_t ADC_MAX_VAL = (1 << ADC_NUM_BITS) - 1;
//    uint32_t adc_in; // Value read in from the ADC
//
//
//    // Set GPIO addresses
//    enableBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ENABLE_BASE_ADDR);
//    brakeBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_INLC_BRAKE_BASE_ADDR);
//    directionBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_INHC_DIRECTION_BASE_ADDR);
//    directionIntBaseAddress = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_DIRECTION_INTERRUPT_BASE_ADDR);
//
//
//    /* Open drivers to open the UART driver for console */
//    Drivers_open();
//    Board_driversOpen();
//
//    DebugP_log("EPWM Duty Cycle Test Started ...\r\n");
//    DebugP_log("App will wait for 60 seconds (using PWM period ISR) ...\r\n");
//
//
//    /* Configure GPIO directions and values */
//    /* Set enable high */
//    GPIO_setDirMode(enableBaseAddress, GPIO_ENABLE_PIN, GPIO_ENABLE_DIR);
//    GPIO_pinWriteHigh(enableBaseAddress, GPIO_ENABLE_PIN);
//
//    /* Set brake high */
//    GPIO_setDirMode(brakeBaseAddress, GPIO_INLC_BRAKE_PIN, GPIO_INLC_BRAKE_DIR);
//    GPIO_pinWriteHigh(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
//
//
//
//    /* Set direction interrupt as input */
//    GPIO_setDirMode(directionIntBaseAddress, GPIO_DIRECTION_INTERRUPT_PIN, GPIO_DIRECTION_INTERRUPT_DIR);
//    directionIntBankNum = GPIO_GET_BANK_INDEX(GPIO_DIRECTION_INTERRUPT_PIN);
//
//
//    /* Set direction */
//    GPIO_setDirMode(directionBaseAddress, GPIO_INHC_DIRECTION_PIN, GPIO_INHC_DIRECTION_DIR);
//    // Read direction level from the switched pin (interrupt pin)
//    // Set the direction on the direction pin
//    if(GPIO_pinOutValueRead(directionIntBaseAddress, GPIO_DIRECTION_INTERRUPT_PIN) == GPIO_PIN_LOW) {
//        GPIO_pinWriteLow(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
//    }
//    else {
//        // curr_direction = GPIO_PIN_HIGH
//        GPIO_pinWriteHigh(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
//    }
//
//
////    status = SemaphoreP_constructCounting(&gEpwmSyncSemObject, 0, runtime_count);
////    DebugP_assert(SystemP_SUCCESS == status);
////    /* Register & enable interrupt */
////    HwiP_Params_init(&hwiPrms);
////    /* Integrate with Syscfg */
////    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
////    hwiPrms.callback    = &App_epwmIntrISR;
////    /* Integrate with Syscfg */
////    hwiPrms.isPulse     = APP_INT_IS_PULSE;
////    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
////    DebugP_assert(status == SystemP_SUCCESS);
//
//    /* Enable interrupt for direction switch */
//    HwiP_Params_init(&directionInterrupt);
//    // This interrupt comes from Table 10-15 in the AM263 Technical Reference Manual.
//    // Taken from the gpio_input_interrupt example in the SDK, the example uses an alias
//    // for this interrupt number. The XBAR is outputting the GPIO43 (pin J2-11 on AM263 LP)
//    directionInterrupt.intNum = CSLR_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_14;
//    directionInterrupt.callback = &App_gpioDirIntrISR;
//    directionInterrupt.isPulse     = APP_INT_IS_PULSE;
//    status              = HwiP_construct(&GpioHwiObject, &directionInterrupt);
//    DebugP_assert(status == SystemP_SUCCESS );
//
//
////    EPWM_clearEventTriggerInterruptFlag(pwm13BaseAddress);
//    GPIO_bankIntrEnable(directionIntBaseAddress, directionIntBankNum);
//    uint32_t intrStatus = GPIO_getBankIntrStatus(directionIntBaseAddress, directionIntBankNum);
//    GPIO_clearBankIntrStatus(directionIntBaseAddress, directionIntBankNum, intrStatus);
//
//    uint32_t old_eqep = EQEP_getPositionLatch(gEqepBaseAddr);
//    uint32_t new_eqep = 0;
//    uint32_t eqep_delta =0;
//    // float unit_time_period = (float)2000000/(8460000); // unit time period counter value / clock rate (Hz)
//    double speed_rpm = 0;
//    double speed_rps = 0;
//    while(runtime_count > 0)
//    {
//
////        // Clear ADC interrupt status,
////        // then force ADC start-of-conversion
////        ADC_clearInterruptStatus(adc3BaseAddress, ADC_INT_NUMBER1);
////        ADC_forceSOC(adc3BaseAddress, adc_pot);
////        while(ADC_getInterruptStatus(adc3BaseAddress, ADC_INT_NUMBER1) == false)
////        {
////            /* Wait for the SOC conversion to complete */
////        }
////
////        /*
////         Read in the ADC result from ADC3, input 3 (potentiometer)
////         Calculate the duty cycle. For 12-bit ADC, result is
////         between [0, 4095].
////         Divide the ADC result by 4095 to get duty cycle.
////         PWM signal goes high when the EPWM counter reaches the
////         counter compare value, so we need to invert the duty cycle.
////         Then, (1-duty_cycle) represents the percentage of the way
////         through the entire PWM period that the signal should go high.
////         */
////        adc_in = ADC_readResult(adc3ResultBaseAddress, adc_pot);
////        //DebugP_log("ADC IN: %d\r\n", adc_in);
////        duty_cycle = (float)adc_in / ADC_MAX_VAL;
////        //DebugP_log("Duty Cycle: %.3f\r\n", duty_cycle);
////        cmpB_value = (1 - duty_cycle) * TIME_BASE_PERIOD;
////        //DebugP_log("cmpB_value: %d\r\n\r\n", cmpB_value);
//
//
////        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
//        runtime_count--;
//        DebugP_log("Number of ISR triggers: %d\r\n", isr_trigger_count);
//        // Update the counter compare value, setting the duty cycle
////        EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, cmpB_value);
//
//
////
////        if((EQEP_getInterruptStatus(gEqepBaseAddr) & EQEP_INT_UNIT_TIME_OUT) != 0)
////            {
////                // Get new values
////                new_eqep = EQEP_getPositionLatch(gEqepBaseAddr);
////                timer_delta = timer_count - timer_prev;
////
////
////                if (new_eqep >= old_eqep) {
////                    eqep_delta = new_eqep - old_eqep;
////                }
////                else {
////                    eqep_delta = EQEP_MAX_POS - old_eqep + new_eqep;
////                }
////                /*******************************************************
////                 * timer_scaler (defined above)
////                 * contains timer tick period, ns -> min conversion,
////                 * and number of eqep ticks per revolution
////                 * Pre-calculating it saves time here.
////                 *******************************************************/
////                //speed_rpm = timer_scaler_rpm * eqep_delta / timer_delta;
////                speed_rps = timer_scaler_rps * eqep_delta / timer_delta;
////
////
////                /* Print (slow), not counted in timers */
////                DebugP_log("ISR COUNT: %d\r\n", isr_trigger_count);
////                //DebugP_log("EQEP delta: %u\r\n", eqep_delta);
////                //DebugP_log("Timer delta (10us periods): %u\r\n", timer_delta);
////                //DebugP_log("Timer count: %u\r\n", timer_count);
////                //DebugP_log("Speed (RPM): %.2f\r\n", speed_rpm);
////                DebugP_log("Speed (RPS): %.2f\r\n\r\n", speed_rps);
////
////                // Update old values
////                old_eqep = EQEP_getPositionLatch(gEqepBaseAddr);
////                timer_prev = timer_count;
////                EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_UNIT_TIME_OUT);
////            }
//    }
//
//
////    EPWM_setCounterCompareValue(pwm13BaseAddress, EPWM_COUNTER_COMPARE_B, TIME_BASE_PERIOD);
////    EPWM_disableInterrupt(pwm13BaseAddress);
////    EPWM_clearEventTriggerInterruptFlag(pwm13BaseAddress);     /* Clear any pending interrupts if any */
////    HwiP_destruct(&gEpwmHwiObject);
////    SemaphoreP_destruct(&gEpwmSyncSemObject);
//    HwiP_destruct(&GpioHwiObject);
//
//
//    DebugP_log("All tests have passed!!\r\n");
//
//
////
////    /* Clear and disable interrupt */
////    ADC_disableInterrupt(adc3BaseAddress, ADC_INT_NUMBER1);
////    ADC_clearInterruptStatus(adc3BaseAddress, ADC_INT_NUMBER1);
////    /* Power down the ADC */
////    ADC_disableConverter(adc3BaseAddress);
//
//    DebugP_log("ADC Software Triggered Conversion Test Passed!!\r\n");
//    DebugP_log("All tests have passed!!\r\n");
//
//    Board_driversClose();
//    Drivers_close();
//}
//
////static void App_epwmIntrISR(void *handle)
////{
////    volatile bool status;
////
////    status = EPWM_getEventTriggerInterruptStatus(pwm13BaseAddress);
////    if(status == true)
////    {
////        SemaphoreP_post(&gEpwmSyncSemObject);
////        EPWM_clearEventTriggerInterruptFlag(pwm13BaseAddress);
////    }
////
////    return;
////}
//
//
///***********************************************
// * GPIO Direction input ISR
// *
// * On a level change on the direction pin,
// * 1. Get the current status of the motor
// *      - Duty cycle
// * 2. Slow down/stop the motor
// * 3. Toggle the direction
// * 4. Resume motor operation
// *
// ***********************************************/
//static void App_gpioDirIntrISR(void *handle) {
//    /* Get and clear bank interrupt status */
//    uint32_t intrStatus = GPIO_getBankIntrStatus(directionIntBaseAddress, directionIntBankNum);
//    GPIO_clearBankIntrStatus(directionIntBaseAddress, directionIntBankNum, intrStatus);
//
//    // Disable interrupts
//    GPIO_bankIntrDisable(directionIntBaseAddress, directionIntBankNum);
//
//    volatile uint32_t count = 0;
//    int curr_direction;
//
//    // ONLY RUN ISR CODE IF CURRENT OUTPUT DIRECTION IS DIFFERENT THAN
//    // INPUT INTURRUP DIRECTION
//    if(GPIO_pinOutValueRead(directionBaseAddress, GPIO_INHC_DIRECTION_PIN) != GPIO_pinRead(directionIntBaseAddress, GPIO_DIRECTION_INTERRUPT_PIN)) {
//        /* Set brake high */
//       GPIO_pinWriteLow(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
//
//       // Wait for motor to full stop
//       //while(EQEP_getPositionLatch(gEqepBaseAddr) != EQEP_getPositionLatch(gEqepBaseAddr));
//       while(count++ < 50000000);
//       //DebugP_log("Changing direction\r\n");
//
//       // Read direction level from the switched pin (interrupt pin)
//       curr_direction = GPIO_pinRead(directionIntBaseAddress, GPIO_DIRECTION_INTERRUPT_PIN);
//       // Set the direction on the direction pin
//       if(curr_direction == GPIO_PIN_LOW) {
//           GPIO_pinWriteLow(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
//       }
//       else {
//           // curr_direction = GPIO_PIN_HIGH
//           GPIO_pinWriteHigh(directionBaseAddress, GPIO_INHC_DIRECTION_PIN);
//       }
//       GPIO_pinWriteHigh(brakeBaseAddress, GPIO_INLC_BRAKE_PIN);
//    }
//
//    // Enable interrupts
//    GPIO_bankIntrEnable(directionIntBaseAddress, directionIntBankNum);
//
//    isr_trigger_count++;
//}
//
///***********************************************
// * Timer interrupt ISR
// *
// ***********************************************/
//void App_timerIntrISR(void *args) {
//    timer_count++;
//}
