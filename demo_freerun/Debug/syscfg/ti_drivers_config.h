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

#ifndef TI_DRIVERS_CONFIG_H_
#define TI_DRIVERS_CONFIG_H_

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>
#include "ti_dpl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Common Functions
 */
void System_init(void);
void System_deinit(void);

/*
 * ADC
 */
#include <drivers/adc.h>
#include <drivers/soc.h>

/* ADC Instance Macros */
#define CONFIG_ADC3_BASE_ADDR (CSL_CONTROLSS_ADC3_U_BASE)
#define CONFIG_ADC3_RESULT_BASE_ADDR (CSL_CONTROLSS_ADC3_RESULT_U_BASE)
#define CONFIG_ADC_NUM_INSTANCES (1U)

/*
 * EPWM
 */
#include <drivers/epwm.h>
#include <drivers/soc.h>

/* EPWM Instance Macros */
#define PWM13_B_INHA_BASE_ADDR (CSL_CONTROLSS_G0_EPWM13_U_BASE)
#define CONFIG_EPWM_NUM_INSTANCES (1U)

/*
 * EQEP
 */
#include <drivers/eqep.h>
#include <drivers/soc.h>

/* EQEP Instance Macros */
#define CONFIG_EQEP0_BASE_ADDR (CSL_CONTROLSS_EQEP0_U_BASE)
#define CONFIG_EQEP_NUM_INSTANCES (1U)

/*
 * GPIO
 */
#include <drivers/gpio.h>

/* GPIO PIN Macros */
#define GPIO_ENABLE_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_ENABLE_PIN (74)
#define GPIO_ENABLE_DIR (GPIO_DIRECTION_OUTPUT)
#define GPIO_ENABLE_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define GPIO_INHC_DIRECTION_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_INHC_DIRECTION_PIN (62)
#define GPIO_INHC_DIRECTION_DIR (GPIO_DIRECTION_OUTPUT)
#define GPIO_INHC_DIRECTION_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define GPIO_INLC_BRAKE_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_INLC_BRAKE_PIN (61)
#define GPIO_INLC_BRAKE_DIR (GPIO_DIRECTION_OUTPUT)
#define GPIO_INLC_BRAKE_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define GPIO_DIRECTION_INTERRUPT_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_DIRECTION_INTERRUPT_PIN (43)
#define GPIO_DIRECTION_INTERRUPT_DIR (GPIO_DIRECTION_INPUT)
#define GPIO_DIRECTION_INTERRUPT_TRIG_TYPE (GPIO_TRIG_TYPE_BOTH_EDGE)
#define GPIO_BRAKE_INTERRUPT_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_BRAKE_INTERRUPT_PIN (75)
#define GPIO_BRAKE_INTERRUPT_DIR (GPIO_DIRECTION_INPUT)
#define GPIO_BRAKE_INTERRUPT_TRIG_TYPE (GPIO_TRIG_TYPE_BOTH_EDGE)
#define CONFIG_GPIO_NUM_INSTANCES (5U)




/*
 * UART
 */
#include <drivers/uart.h>

/* UART Instance Macros */
#define CONFIG_UART0 (0U)
#define CONFIG_UART_NUM_INSTANCES (1U)
#define CONFIG_UART_NUM_DMA_INSTANCES (0U)


#ifdef __cplusplus
}
#endif

#endif /* TI_DRIVERS_CONFIG_H_ */
