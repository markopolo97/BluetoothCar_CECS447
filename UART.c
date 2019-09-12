// UART.c
// Runs on TM4C123 or LM4F120
// Lab 11 involves switching this from UART1 to UART0.
//                 switching from PC5,PC4 to PA1,PA0
// Mark Aquiapao and TrieuVy Le
// May 1, 2019

// This connection occurs in the Bluetooth Serial Terminal 
// U1Rx (PB0) connected to serial port on PC
// U1Tx (PB1) connected to serial port on PC
// Ground connected ground in the USB cable

#include "tm4c123gh6pm.h"
#include "UART.h"

//------------UART1_Init------------
// Initialize the UART for 57600 baud rate (assuming 80 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none

void UART1_Init(void){
// as part of Lab 11, modify this program to use UART0 instead of UART1
//                 switching from PC5,PC4 to PA1,PA0
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1; // activate UART1
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; // activate port B
  UART1_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART1_IBRD_R = 86;                  	// IBRD = int(80,000,000 / (16 * 57600)) 
																				//			= int(86.80555556)
  UART1_FBRD_R = 51;                  	// FBRD = round(0.80555556 * 64 + 0.05) = 52
																				// 8 bit word length (odd parity bits, one stop bit)
  UART1_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_PEN);		
  UART1_CTL_R |= UART_CTL_UARTEN;       // enable UART
  GPIO_PORTB_AFSEL_R |= 0x03;           // enable alt funct on PB1,PB0
  GPIO_PORTB_DEN_R |= 0x03;             // enable digital I/O on PB1,PB0
  GPIO_PORTB_PCTL_R &= ~0x000000FF; 		// configure PB0 and PB1 as U1Rx and U1Tx
	GPIO_PORTB_PCTL_R |= 0x00000011;
  GPIO_PORTB_AMSEL_R &= ~0x03;          // disable analog functionality on PB1,PB0
}

//------------UART1_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
unsigned char UART1_InChar(void){
  while((UART1_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART1_DR_R&0xFF));
}


//------------UART1_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART1_OutChar(unsigned char data){
// as part of Lab 11, modify this program to use UART0 instead of UART1
  while((UART1_FR_R&UART_FR_TXFF) != 0);
  UART1_DR_R = data;
}

//------------UART1_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART1_OutString(unsigned char str[]){
	unsigned int i = 0;
	while(str[i] != 0x0){
		UART1_OutChar(str[i]);
		i++;
	}
}
