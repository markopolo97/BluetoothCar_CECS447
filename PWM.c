// PWM.c
// Runs on TM4C123
// Use PWM0/PB6 to generate pulse-width modulated output.
// Mark Aquiapao and TrieuVy Le
// May 4, 2019
//*******************************************************************************
#include <stdint.h>
//#include "inc/tm4c123gh6pm.h"
#include "tm4c123gh6pm.h"
//*******************************************************************************
// period is 16-bit number of PWM clock cycles in one period (3<=period)
// period for PB6 and PB7 must be the same
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
// PWM clock rate = processor clock rate/SYSCTL_RCC_PWMDIV
//                = BusClock/2 
//                = 80 MHz/2 = 40 MHz (in this example)
// Output on PB6/M0PWM0
//*******************************************************************************
void PWM0A_Init(uint16_t period, uint16_t duty){
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B
  while((SYSCTL_PRGPIO_R&0x02) == 0){};
  GPIO_PORTB_AFSEL_R |= 0x40;           // enable alt funct on PB6
  GPIO_PORTB_PCTL_R &= ~0x0F000000;     // configure PB6 as PWM0
  GPIO_PORTB_PCTL_R |= 0x04000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;          // disable analog functionality on PB6
  GPIO_PORTB_DEN_R |= 0x40;             // enable digital I/O on PB6
  SYSCTL_RCC_R = 0x00100000 |           // 3) use PWM divider
      (SYSCTL_RCC_R & (~0x000E0000));   //    configure for /2 divider
  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_0_LOAD_R = period - 1;           // 5) cycles needed to count down to 0
  PWM0_0_CMPA_R = duty - 1;             // 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000001;          // enable PB6/M0PWM0
}
//*******************************************************************************
// change duty cycle of PB6
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM0A_Duty(uint16_t duty){
  PWM0_0_CMPA_R = duty - 1;             // 6) count value when output rises
}
//*******************************************************************************
void Oscill_Init(void){
	unsigned volatile long delay;
  SYSCTL_RCGC2_R |= 0x00000002; 		// activate clock for port B
	delay = SYSCTL_RCGC2_R;
  GPIO_PORTB_AMSEL_R &= ~0x04;      // disable analog functionality on PB2
  GPIO_PORTB_PCTL_R &= ~0x00000F00; // configure PB2 as GPIO
  GPIO_PORTB_DIR_R |= 0x04;     		// make PB2 output
  GPIO_PORTB_AFSEL_R &= ~0x04;  		// disable alt funct on PB2
  GPIO_PORTB_DEN_R |= 0x04;     		// enable digital I/O on PB2
  GPIO_PORTB_DATA_R &= ~0x04;   		// make PB2 low
}


