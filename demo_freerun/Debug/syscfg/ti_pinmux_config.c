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
#include "ti_drivers_config.h"
#include <drivers/pinmux.h>

static Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {
            /* EPWM13 pin config */
    /* EPWM13_B -> EPWM13_B (K3) */
    {
        PIN_EPWM13_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

            /* EQEP0 pin config */
    /* EQEP0_A -> EQEP0_A (B14) */
    {
        PIN_EQEP0_A,
        ( PIN_MODE(8) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC )
    },
    /* EQEP0_B -> EQEP0_B (A14) */
    {
        PIN_EQEP0_B,
        ( PIN_MODE(8) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC )
    },
    /* EQEP0_STROBE -> EQEP0_STROBE (C12) */
    {
        PIN_EQEP0_STROBE,
        ( PIN_MODE(8) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC )
    },
    /* EQEP0_INDEX -> EQEP0_INDEX (D11) */
    {
        PIN_EQEP0_INDEX,
        ( PIN_MODE(8) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC )
    },

            /* GPIO0 pin config */
    /* GPIO74 -> EPWM15_B (R16) */
    {
        PIN_EPWM15_B,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO62 -> EPWM9_B (J2) */
    {
        PIN_EPWM9_B,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO61 -> EPWM9_A (G1) */
    {
        PIN_EPWM9_A,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO43 -> EPWM0_A (B2) */
    {
        PIN_EPWM0_A,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO75 -> UART1_RXD (L3) */
    {
        PIN_UART1_RXD,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },

            /* UART0 pin config */
    /* UART0_RXD -> UART0_RXD (A7) */
    {
        PIN_UART0_RXD,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* UART0_TXD -> UART0_TXD (A6) */
    {
        PIN_UART0_TXD,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

    {PINMUX_END, PINMUX_END}
};


/*
 * Pinmux
 */
void Pinmux_init(void)
{
    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);
}

