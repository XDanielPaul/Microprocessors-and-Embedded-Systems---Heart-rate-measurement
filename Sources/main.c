/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "MK60D10.h"
#include <stdio.h>
#include <ctype.h>
#include <string.h>

/* Macros for bit-level registers manipulation */
#define GPIO_PIN_MASK 0x1Fu
#define GPIO_PIN(x) (((1)<<(x & GPIO_PIN_MASK)))

/* Defining value for each digit in display */
#define Ce1 ~0x1000
#define Ce2 ~0x4000
#define Ce3 ~0x8000
#define Ce4 ~0x2000

/* Defining values for segements to show numbers on LED display */
#define Num0 0x0E8C
#define Num1 0x0404
#define Num2 ~0x047B
#define Num3 ~0x00FB
#define Num4 0x050C
#define Num5 0x0F08
#define Num6 0x0F88
#define Num7 0x0604
#define Num8 0x0F8C
#define Num9 0x0F0C


char result[10] = "xxxx"; // Result is being shown on LED display
unsigned int index; // Index to result array
int heartbeats = 0; // Number of heartbeats
int write_to_display = 0; // Boolean that either forbids/allows showing result to display


/* A delay function */
void delay(long long bound) {
  long long i;
  for(i=0;i<bound;i++);
}

/* Initialize the MCU - basic clock settings, turning the watchdog off */
void MCUInit(void)  {
    MCG_C4 |= ( MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x01) );
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(0x00);
    WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;
}

/* Initialize ports */
void PortsInit(void)
{

    /* Turn on port clocks */
	WDOG_STCTRLH_WDOGEN(0);
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK;

    /* Set corresponding PTA/PTD pins for GPIO functionality */
    PORTA->PCR[2] = PORT_PCR_MUX(0x01); // P1 - 30 B
    PORTA->PCR[3] = PORT_PCR_MUX(0x01); // P1 - 29 F

    PORTA->PCR[6] = PORT_PCR_MUX(0x01); // P1 - 25 DP
    PORTA->PCR[7] = PORT_PCR_MUX(0x01); // P1 - 27 E
    PORTA->PCR[8] = PORT_PCR_MUX(0x01); // P1 - 23 G
    PORTA->PCR[9] = PORT_PCR_MUX(0x01); // P1 - 28 A
    PORTA->PCR[10] = PORT_PCR_MUX(0x01); // P1 - 24 C
    PORTA->PCR[11] = PORT_PCR_MUX(0x01); // P1 - 26 D

    PORTD->PCR[12] = PORT_PCR_MUX(0x01); // P1 - 19 C1
    PORTD->PCR[13] = PORT_PCR_MUX(0x01); // P1 - 20 C4
    PORTD->PCR[14] = PORT_PCR_MUX(0x01); // P1 - 22 C2
    PORTD->PCR[15] = PORT_PCR_MUX(0x01); // P1 - 21 C3


    /* Change corresponding PTA/PTD port pins as outputs */
    PTA->PDDR = GPIO_PDDR_PDD(0x0FCC);     // Segments as output
    PTD->PDDR = GPIO_PDDR_PDD(0xF000);	  // Number as output
    PTA->PDOR &= GPIO_PDOR_PDO(~0x0FCC);
    }

/* Initialize ADC */
void ADC0_Init(void)
{
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; // ADC0 Clock
	ADC0_CFG1 = (ADC_CFG1_MODE(3) | ADC_CFG1_ADIV(2)); // 16bit mode
	ADC0_SC1A = ADC_SC1_ADCH(31); // Turns off ADC
}

/* Initialize LPTMR */
void LPTMR0Init()
{
    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK; // Enable clock to LPTMR
    LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;   // Turn OFF LPTMR to perform setup
    LPTMR0_PSR = ( LPTMR_PSR_PRESCALE(0)
                 | LPTMR_PSR_PBYP_MASK   // LPO feeds directly to LPT
                 | LPTMR_PSR_PCS(1)) ;   // use the choice of clock - LP0 - 1kHz
    LPTMR0_CMR = 1000;                  // Set initial value to 1000
    LPTMR0_CSR =(  LPTMR_CSR_TCF_MASK    // Clear any pending interrupt (now)
                 | LPTMR_CSR_TIE_MASK    // LPT interrupt enabled
                );
    NVIC_EnableIRQ(LPTMR0_IRQn);         // enable interrupts from LPTMR0
    LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;    // Turn ON LPTMR0 and start counting
}


/* Handle LPTMR interrupt */
void LPTMR0_IRQHandler(void)
{
	/* Write heartrate to result
	 * (*2 because we are measuring in 15 second intervals and
	 * there are two peaks in one heartbeat [opening/closing of heart valve])
	 * */
	sprintf(result, "%d", (heartbeats*2));
	write_to_display = 1; // allow display
	heartbeats = 0; // reset heartbeats
    LPTMR0_CMR = 15000;                // set timer to interrupt after 15 seconds
    LPTMR0_CSR |=  LPTMR_CSR_TCF_MASK;   // write 1 to TCF tclear the flag

}

/* Show value on LED display */
void display_val(char* val_str) {

	/* Don't show any undesirable results */
	if(index >= strlen(val_str) || index > 4){
		PTA->PDOR = GPIO_PDOR_PDO(~0x0FCC);
		return;
	}

	if(strlen(val_str) > 3){
		PTA->PDOR = GPIO_PDOR_PDO(~0x0FCC);
		return;
	}

	/* Show number on chosen digit */
	switch (val_str[index]){
	case '0':
			PTA->PDOR = GPIO_PDOR_PDO(Num0);
			index++;
			break;
	case '1':
			PTA->PDOR = GPIO_PDOR_PDO(Num1);
			index++;
			break;
	case '2':
			PTA->PDOR = GPIO_PDOR_PDO(Num2);
			index++;
			break;
	case '3':
			PTA->PDOR = GPIO_PDOR_PDO(Num3);
			index++;
			break;
	case '4':
			PTA->PDOR = GPIO_PDOR_PDO(Num4);
			index++;
			break;
	case '5':
			PTA->PDOR = GPIO_PDOR_PDO(Num5);
			index++;
			break;
	case '6':
			PTA->PDOR = GPIO_PDOR_PDO(Num6);
			index++;
			break;
	case '7':
			PTA->PDOR = GPIO_PDOR_PDO(Num7);
			index++;
			break;
	case '8':
			PTA->PDOR = GPIO_PDOR_PDO(Num8);
			index++;
			break;
	case '9':
			PTA->PDOR = GPIO_PDOR_PDO(Num9);
			index++;
			break;
	}
}

/* Choose digit and display given value */
void show(void){
	/* Don't show anything unless there is a result (interrupt has been registered) */
	if (write_to_display == 1){
	   index = 0;
	   /* Display value to first digit */
	   display_val(result);
	   PTD->PDOR |= GPIO_PDOR_PDO(Ce1);
	   delay(2000);
	   PTD->PDOR &= GPIO_PDOR_PDO(~Ce1);
	   /* Display value to second digit */
	   display_val(result);
	   PTD->PDOR |= GPIO_PDOR_PDO(Ce2);
	   delay(2000);
	   PTD->PDOR &= GPIO_PDOR_PDO(~Ce2);
	   /* Display value to third digit */
	   display_val(result);
	   PTD->PDOR |= GPIO_PDOR_PDO(Ce3);
	   delay(2000);
	   PTD->PDOR &= GPIO_PDOR_PDO(~Ce3);
	   /* Display value to fourth digit */
	   display_val(result);
	   PTD->PDOR |= GPIO_PDOR_PDO(Ce4);
	   delay(2000);
	   PTD->PDOR &= GPIO_PDOR_PDO(~Ce4);
	 }
}

int main(void)
{

	/* Initialize everything */
    MCUInit();
    PortsInit();
    ADC0_Init();
    LPTMR0Init();

    /* Define required variables */
    int rising = 0; // 0 - signal decreasing, 1 - signal rising
    int prev_num = 0; // previous averaged value
    int act_num = 0; // actual averaged value
    int array_size = 30; // averaging sample size
    int hb_array[array_size]; // array for averaging value
    int array_counter = 0; // array_index
    int average; // Average of values
    int signal; // Value from ADC0_RA


    /* Variables for calculating high pass filter result */
    float curr_y;
    float prev_x = 0;
    float prev_y = 0;
    float alpha = 1000/(53.0516477+1000);


    while (1) {

    	ADC0_SC1A = 0x0 & ADC_SC1_ADCH_MASK; // Writing to ADCSC1 to start conversion, choose channel DP0
    	while((ADC0_SC2 & 0x80)); // Waiting if conversion is in progress
    	while(!(ADC0_SC1A & 0x80)); // Waiting until conversion has been completed
    	signal = ADC0_RA; // Writing output to signal

    	/* Calculating high pass filter value */
    	curr_y = alpha * (prev_y + signal - prev_x);
    	prev_x = signal;
    	prev_y = curr_y;

    	hb_array[array_counter] = (int)(curr_y*3); // Adding value to array (*3 - Amplifying difference)
    	array_counter++;

    	/* If required sample size has been collected */
    	if(array_counter >= array_size){

    		array_counter = 0;

    		/* Calculate average */
    		for (int i = 0; i < array_size;i++){
    			average += hb_array[i];
    		}

    		act_num = average / array_size;
    		average = 0;

    		/* If the signal was rising */
    		if(rising == 1){
    					/* And is now decreasing */
    		    		if (prev_num > act_num){
    		    			rising = 0; // Give information about decreasing
    		    			heartbeats++; // Increment heartbeat
    		    		} else {
    		    			/* Otherwise write set current value as previous */
    		    			prev_num = act_num;
    		    		}
    		    	} else {
    		    		/* If the signal is decreasing and started rising */
    		    		if(prev_num < act_num){
    		    			rising = 1; // Give information about rising
    		    		} else {
    		    			/* Otherwise write set current value as previous */
    		    			prev_num = act_num;
    		    		}
    		   }
    	}

    	/* Show results to LED display */
    	show();

    }

    return 0;
}
