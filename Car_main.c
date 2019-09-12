// main.c
// Runs on LM4F120/TM4C123
// Car for Project Bluetooth
// CECS 447 with John Yu
// Mark Aquaipao and TrieuVy Le
// May 5, 2019
//-------------------------------------------------------------------
/*
    PB0 UART1(RX)  ->  BT(TX) for HC-05 module
    PB1 UART1(TX)  ->  BT(RX) for HC-05 module
    PC4 GPIO       ->  Right wheel control
    PC5 GPIO       ->  Right wheel control
    PC6 GPIO       ->  Left wheel control
    PC7 GPIO       ->  Left wheel control
    PB4 GPIO       ->  IR Transmitter(IR LED)
    PF1 GPIO (out) ->  On-board RED LED
    PF2 GPIO (out) ->  On-board BLUE LED
    PF3 GPIO (out) ->  On-board GREEN LED
*/
//-------------------------------------------------------------------
// NPN Transistor 2222A 
// Base 			: 1K resistor PB2
// Emitter 		: Ground
// Collector 	: IR LED, 100 ohms, VCC
//-------------------------------------------------------------------
// PWM Output on PB6/M0PWM0
//-------------------------------------------------------------------
// Functions in other files
#include "UART.h"
#include "PLL.h"
#include "PWM.h"
#include <stdint.h>	
#include "tm4c123gh6pm.h"
//-------------------------------------------------------------------
#define LED  						 		(*((volatile unsigned long *)0x400253FC))	
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOB			0x00000002	// port B Clock Gating Control
#define SYSCTL_TCGC2_FPIOA			0x00000001	// port A Clock Gating Control
//-------------------------------------------------------------------
// COLORS of LED:
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define YELLOW    0x0A
#define SKYBLUE   0x0C
#define WHITE     0x0E
#define PINK			0x06
#define OFF			0x00
//-------------------------------------------------------------------
// Command Signals for Bluetooth
#define w					0x77 // Forward
#define s					0x73 // Reverse
#define a					0x61 // Left
#define d					0x64 // Right 
#define q					0x71 // Stop
#define c					0x63 // Change Address
#define zero			0x30
#define one				0x31
#define two				0x32
#define three			0x33
#define four			0x34
#define NULL			0x00

//-------------------------------------------------------------------
#define forward   
//-------------------------------------------------------------------
// Subroutines
void EnableInterrupts(void);  // Enable interrupts
void Delay(int);
void LED_Init(void);
void PWM1A_Init(void);
void Motor_Init(void);
void Timer1A_Init(uint32_t);
void Start(void);
void Logic(uint8_t);
void Address(uint8_t);
//-------------------------------------------------------------------
// Global Varibales for UART and IR 
unsigned int   BT_DATA=0;					// key on keyboard for BT UART
unsigned char  UART_BUSY=0;         // BT UART1 busy flag
unsigned char  IR_ADDR=0;           // IR transmission request
//-------------------------------------------------------------------
int main(void){ 
	
  PLL_Init();								// set system clock to 80 MHz
	Timer1A_Init(1000);			  // initilize timer for 1us
	PWM1A_Init();							// initialize IR LED and 40KHz signal
	Motor_Init();							// Turns on the motor
	LED_Init();								// Setup LEDs
  UART1_Init();             // initialize UART
  EnableInterrupts();       // needed for TExaS
	LED = WHITE;
	// Remove comment below to TEST SIGNAL
	//PWM0_1_CMPA_R = 1000;	// set bit high 
 
  while(1){
		BT_DATA = UART1_InChar();
		
		// FORWARD
		if(BT_DATA == w){ // FORWARD
			LED = GREEN;
			UART1_OutString("Forward\r\n");
			GPIO_PORTC_DATA_R = 0xC0;  
			Delay(2);
			GPIO_PORTC_DATA_R = 0x00;
		}
			
		// REVERSE
		else if(BT_DATA == s){
			LED = RED;
			UART1_OutString("Reverse\r\n");
			GPIO_PORTC_DATA_R = 0x30;   
			Delay(2);
			GPIO_PORTC_DATA_R = 0x00;
		}
			
		// RIGHT
		else if(BT_DATA == d){ 
			LED = BLUE;
			UART1_OutString("Right Turn\r\n");
			Delay(1);
			GPIO_PORTC_DATA_R = 0x60; 
			Delay(1);
			GPIO_PORTC_DATA_R = 0x00;
		}
			
		// LEFT
		else if(BT_DATA == a){ 
			LED = YELLOW;
			UART1_OutString("Left Turn\r\n");
			Delay(1);
			GPIO_PORTC_DATA_R = 0x90;  
			Delay(1);
			GPIO_PORTC_DATA_R = 0x00;
		}
			
		// STOP
		else if(BT_DATA == q){ 
			LED = OFF;
			UART1_OutString("Stop\r\n");
			GPIO_PORTC_DATA_R = 0x00;
		}
		
		// CHANGE ADDRESS
		else if(BT_DATA == c){
			if(IR_ADDR>=3) 	IR_ADDR = 0;
			else						IR_ADDR++;
			UART1_OutString("Change Address\r\n");
			if(IR_ADDR==0)			UART1_OutString("Address @ Device 0\r\n");
			else if(IR_ADDR==1)	UART1_OutString("Address @ Device 1\r\n");
			else if(IR_ADDR==2)	UART1_OutString("Address @ Device 2\r\n");
			else if(IR_ADDR==3)	UART1_OutString("Address @ Device 3\r\n");
			else								UART1_OutString("Invalid Address\r\n");
		} 
		
		// COMMAND 0 : ANIMATION 1
		else if(BT_DATA == zero){	
			Start(); Address(IR_ADDR); 
			Logic(0); Logic(0); Logic(0); Logic(0); 
			UART1_OutString("Trasmited Command 0 \n\r"); 
		}
		
		// COMMAND 1 : ANIMATION 2
		else if(BT_DATA == one){	
			Start(); Address(IR_ADDR); 
			Logic(0); Logic(0); Logic(0); Logic(1); 
			UART1_OutString("Transmitted Command 1 \n\r"); 
		}
		
		// COMMAND 2 : ANIMATION 3
		else if(BT_DATA == two){	
			Start(); Address(IR_ADDR); 
			Logic(0); Logic(0); Logic(1); Logic(0); 
			UART1_OutString("Transmitted Command 2 \n\r"); 
		}
		
		// COMMAND 3 : ANIMATION 4
		else if(BT_DATA == three){	
			Start(); Address(IR_ADDR); 
			Logic(0); Logic(0); Logic(1); Logic(1); 
			UART1_OutString("Transmitted Command 3 \n\r"); 
		}
		
		// COMMAND 4 : ANIMATION 5
		else if(BT_DATA == four){	
			Start(); Address(IR_ADDR); 
			Logic(0); Logic(1); Logic(0); Logic(0); 
			UART1_OutString("Transmitted Command 4 \n\r"); 
		}
		
		// DEFAULT
		else{
			Delay(1);
			GPIO_PORTC_DATA_R = 0x00;
		}
		
		
  } // while(1)
} // main


//******************* Initialize GPIO Ports *************************
//*******************************************************************
// Port F Initilization
// Subroutine to initialize port F pins for output
// PF3(BLUE),PF2(GREEN),PF1(RED) are outputs to the LEDs
//*******************************************************************
void LED_Init(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R 	|= 0x00000020; 		// 1) F clock
	delay = SYSCTL_RCGC2_R; 					// dummy delay
	GPIO_PORTF_LOCK_R = 0x4C4F434B; 	// 2) unlock PortF PF4
	GPIO_PORTF_CR_R |= 0x0E; 					// allow changes to PF4 
	GPIO_PORTF_AMSEL_R &= ~0x0E; 			// 3) disable analog function on PF4 
	GPIO_PORTF_PCTL_R &= ~0x0000FFF0; // 4) GPIO clear bit PCTL
	GPIO_PORTF_DIR_R |= 0x0E; 				// 5) PF1,PF2,PF3 output
	GPIO_PORTF_AFSEL_R &= ~0x0E; 			// 6) no alternate function
	GPIO_PORTF_DEN_R |= 0x0E; 				// 7) enable digital pins PF4
	GPIO_PORTF_PUR_R &= ~0x11; 				// disable pullup resistors on PF4/0
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00A00000; // (g) priority 5
	NVIC_EN0_R = 0x40000000; 					// (h) enable interrupt 30 in NVIC
}

//*******************************************************************
// Port C Initialization for Motors
//*******************************************************************
void Motor_Init(void){
	unsigned volatile long delay;
  SYSCTL_RCGC2_R |= 0x00000004; 		// activate clock for port C
	delay = SYSCTL_RCGC2_R;
  GPIO_PORTC_AMSEL_R &= ~0xF0;      // disable analog functionality on PC7-4
  GPIO_PORTC_PCTL_R &= ~0xFFFF0000; // configure PC7-4 as GPIO
  GPIO_PORTC_DIR_R |= 0xF0;     		// make PC7-4 out
  GPIO_PORTC_DR8R_R |= 0xF0;    		// enable 8 mA drive on PC7-4
  GPIO_PORTC_AFSEL_R &= ~0xF0;  		// disable alt funct on PC7-4
  GPIO_PORTC_DEN_R |= 0xF0;     		// enable digital I/O on PC7-4
  GPIO_PORTC_DATA_R &= ~0xF0;   		// make PC7-4 low
}

//*******************************************************************
// Delay function to produce x/2 seconds 
//*******************************************************************
void Delay(int sec){
	unsigned long volatile time;
  time = (727240*200/91)*sec;  
  while(time){
		time--;
  }
}

//*********************** PWM Initialization *************************
void PWM1A_Init(){ // PWM 2 Gen 1A                                              
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B
  while((SYSCTL_PRGPIO_R&0x02) == 0){}; //delay
  GPIO_PORTB_AFSEL_R |= 0x10;           // enable alt funct on PB4
  GPIO_PORTB_PCTL_R &= ~0x000F0000;     // configure PB4 as PWM0
  GPIO_PORTB_PCTL_R |= 0x00040000;
  GPIO_PORTB_AMSEL_R &= ~0x10;          // disable analog functionality on PB4
  GPIO_PORTB_DEN_R |= 0x10;             // enable digital I/O on PB4
	GPIO_PORTB_DIR_R |= 0x10;							// set  PB4 as OUTPUT

  PWM0_1_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_1_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_1_LOAD_R = 2000;                 // 5) 80Mhz/40k = 2000    
  PWM0_1_CMPA_R = 0;                 		// 6) count value when output rises
  PWM0_1_CTL_R |= 0x00000001;           // 7) start PWM0
	PWM0_ENABLE_R |= 0x00000004;          // enable PB6/M0PWM0
}

//*********************** Timing Initialization *************************
void Timer1A_Init(uint32_t delay){//micro seconds
  uint32_t i = 0;
  // Timer1A configuration
  SYSCTL_RCGCTIMER_R |= 0x02; // enable clock to timer Block 1
  TIMER1_CTL_R = 0;           // disable Timer1 during configuration
  TIMER1_CFG_R = 0x04;        // 16-bit timer
  TIMER1_TAMR_R = 0x02;       // periodic countdown mode
  TIMER1_TAILR_R = 80 - 1;    // 80E6/80 = 1E6    period = 1MicroSecond     
  TIMER1_ICR_R |= 0x1;        // clear Timer1A timeout flag
  TIMER1_CTL_R |= 0x01;       // enable Timer1A
  NVIC_EN0_R |= 0x00200000;   // enable IRQ21

  while(i < delay){
    while((TIMER1_RIS_R & 0x01) == 0); // wait for timeout
    TIMER1_ICR_R = 0x01;    // clear timerA timeout flag  
    i++;
  }

}

//*********************** IR Protocol***************************
void Start(){
// - Start Pulse: 1ms high, 500us low  
  PWM0_1_CMPA_R = 1000;	// set bit high 
  Timer1A_Init(1000);  	// count for 1ms 
  PWM0_1_CMPA_R = 0;		// set bit low
  Timer1A_Init(500);		// count for 500us 
}

void Logic(uint8_t bit){            
	// - Logical 1: 1.1ms high, 400us low 
	// - Logical 0: 400us high, 400us low  
	switch(bit){
		case '0': PWM0_1_CMPA_R = 1000;	// set high
							Timer1A_Init(400); 		// count 400us
							PWM0_1_CMPA_R = 0;		// set low
							Timer1A_Init(400);		// count 400us 
							break;
		case '1': PWM0_1_CMPA_R = 1000; // set high
							Timer1A_Init(1100); 	// count 1.1ms
							PWM0_1_CMPA_R = 0;	  // set low
							Timer1A_Init(400);    // count 400us
							break;
	}
}

void Address(uint8_t addr){
	uint8_t bit1, bit0; 
  switch(addr){
    case '0' : bit1 = 0; bit0 = 0; break;
    case '1' : bit1 = 0; bit0 = 1; break;
    case '2' : bit1 = 1; bit0 = 0; break;
    case '3' : bit1 = 1; bit0 = 1; break;
  }
  Logic(bit1);
  Logic(bit0);
}
