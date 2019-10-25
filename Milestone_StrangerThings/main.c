/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * --/COPYRIGHT--*/
//******************************************************************************

#include <msp430.h>
#include <stdint.h>

/*
 * Milestone 1:
 * Stranger Things
 *
 * Author: Douglas Fisher, Nick Papas
 */

int count;

void SendUCA0Data(uint8_t data)
{
    while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
    UCA0TXBUF = data;
}

//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

void initUART()
{
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 104;                            // 1MHz 9600
    UCA0BR1 = 0;                              // 1MHz 9600
    UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IFG2 &= ~(UCA0RXIFG);
    IE2 |= UCA0RXIE;
}

void initGPIO()
{
    P1SEL = BIT1 + BIT2;                      // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2;
    P1DIR |= BIT6;                            //set P1.6 to output
    P1SEL |= BIT6;                            //set P1.6 to use PWM
    P2DIR |= BIT1;                            //set P2.1 to output
    P2SEL |= BIT1;                            //set P2.1 to use PWM
    P2DIR |= BIT5;                            //set P2.5 to output
    P2SEL |= BIT5;                            //set P2.5 to use PWM

    count = 0;
}

void initTimer()
{
    TA0CTL |= TASSEL_2;                       //select SMCLK as clk src
    TA0CTL |= MC_1;                           //set to up mode
    TA0CCTL1 |= OUTMOD_7;                     //Sets outmode to reset/set
    TA0CCR0 = 0xFF;                           //Sets timer max
    TA0CCR1 = 0x00;                           //Set Red Duty Cycle to zero

    TA1CTL |= TASSEL_2;                       //select SMCLK as clk src
    TA1CTL |= MC_1;                           //set to up mode
    TA1CCTL1 |= OUTMOD_7;                     //Sets outmode to reset/set
    TA1CCTL2 |= OUTMOD_7;                     //Sets outmode to reset/set
    TA1CCR0 = 0xFF;                           //Sets timer max
    TA1CCR1 = 0x00;                           //Sets Green Duty Cycle
    TA1CCR2 = 0x00;                           //Sets Blue Duty Cycle to zero
}

//******************************************************************************
// Main ************************************************************************
//******************************************************************************

void main()
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
    DCOCTL = CALDCO_1MHZ;

    initUART();
    initGPIO();
    initTimer();

    __bis_SR_register(LPM0_bits + GIE);       //Enter LPM0, interrupts enabled

    while(1);
}

//******************************************************************************
// UART RX Interrupt ***********************************************************
//******************************************************************************
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
{
    if (IFG2 & UCA0RXIFG)
    {
        uint8_t rx_val = UCA0RXBUF; //Must read UCxxRXBUF to clear the flag

        switch (count) {            //Switch for position in EUART
        case 0:                     //Case length
            SendUCA0Data(rx_val - 3);
            break;
        case 1:                     //Case Red
            TA0CCR1 = rx_val;
            break;
        case 2:                     //Case Green
            TA1CCR1 = rx_val;
            break;
        case 3:                     //Case Blue
            TA1CCR2 = rx_val;
            break;
        default:
            SendUCA0Data(rx_val);
        }

        if(rx_val == 0x0D) {
            count = 0;
            return;
        }
        count++;
    }
}
