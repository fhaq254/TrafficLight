// TableTrafficLight.c solution to edX lab 10, EE319KLab 5
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// October 12, 2017

/* 
 Copyright 2017 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// See TExaS.h for other possible hardware connections that can be simulated
// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "SysTick.h"
#include "TExaS.h"

struct state{           // *not const, but don't change them in code
  uint32_t LED_Walk;    // PF3 (green),1 (red) 
	uint32_t LED_Road;    // Port B5-0, South{Red,Yellow,Green} and West{R,Y,G}
  uint32_t Wait;        // 10 ms wait
  uint32_t Next[8];		  // the next states (listed in the array in the order that the inputs correspond to)	
};
typedef struct state crosswalk; 
// FSM State Indices 
#define S_ON      0   // South On
#define S_CD      1   // South Change Direction
#define W_ON      2
#define W_CD      3
#define S_Walk    4
#define W_Walk    5
#define Walk      6
#define Warn_H1   7   // Warning Light High
#define Warn_L1   8   // Warning Light Low
#define Warn_H2   9
#define Warn_L2   10  
#define Warn_H3   11
#define Warn_L3   12

// Output Labels
   // Walk LEDs
#define WalkOn       0x08 
#define DontWalk_H   0x02
#define DontWalk_L   0x00
   // Road LEDs
#define South_G      0x0C  // South Green light
#define South_Y      0x14  // South Yellow light
#define West_G       0x21
#define West_Y       0x22
#define Both_R       0x24  // Stop Traffic, both Red

// Port Labels
#define RoadLight (*((volatile uint32_t *)0x400053FC)) // Port B
#define WalkLight (*((volatile uint32_t *)0x400253FC)) // Port F
#define Buttons   (*((volatile uint32_t *)0x400243FC)) // Port E


crosswalk FSM[13] = {   // {LED_Walk, LED_Road, Wait, Next[ ]}
	{DontWalk_H, South_G, 150, {S_CD,S_CD,S_ON,S_CD,S_Walk,S_Walk,S_Walk,S_Walk}},  
	{DontWalk_H, South_Y, 50,  {W_ON,W_ON,W_ON,W_ON,W_ON,W_ON,W_ON,W_ON}},
	{DontWalk_H, West_G,  150, {W_CD,W_ON,W_CD,W_CD,W_Walk,W_Walk,W_Walk,W_CD}},
	{DontWalk_H, West_Y,  50,  {S_ON,S_ON,S_ON,S_ON,S_ON,S_ON,S_ON,S_ON}},
  {DontWalk_H, South_Y, 50,  {Walk,Walk,Walk,Walk,Walk,Walk,Walk,Walk}},
	{DontWalk_H, West_Y,  50,  {Walk,Walk,Walk,Walk,Walk,Walk,Walk,Walk}},
	{WalkOn,     Both_R,  150, {Warn_H1,Warn_H1,Warn_H1,Warn_H1,Warn_H1,Warn_H1,Warn_H1,Warn_H1}},
	{DontWalk_H, Both_R,  50,  {Warn_L1,Warn_L1,Warn_L1,Warn_L1,Warn_L1,Warn_L1,Warn_L1,Warn_L1}},
	{DontWalk_L, Both_R,  50,  {Warn_H2,Warn_H2,Warn_H2,Warn_H2,Warn_H2,Warn_H2,Warn_H2,Warn_H2}},
	{DontWalk_H, Both_R,  50,  {Warn_L2,Warn_L2,Warn_L2,Warn_L2,Warn_L2,Warn_L2,Warn_L2,Warn_L2}},
	{DontWalk_L, Both_R,  50,  {Warn_H3,Warn_H3,Warn_H3,Warn_H3,Warn_H3,Warn_H3,Warn_H3,Warn_H3}},
	{DontWalk_H, Both_R,  50,  {Warn_L3,Warn_L3,Warn_L3,Warn_L3,Warn_L3,Warn_L3,Warn_L3,Warn_L3}},
	{DontWalk_L, Both_R,  50,  {S_ON,W_ON,S_ON,S_ON,S_ON,W_ON,S_ON,W_ON}}
};

void EnableInterrupts(void);

int main(void){ volatile unsigned long delay;
  uint32_t CS;        // index of current state
	uint32_t Input;   

	TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210); // activate traffic simulation and set system clock to 80 MHz
  SysTick_Init();     
  
	// Initialize ports and FSM you write this as part of Lab 5
	SYSCTL_RCGC2_R |= 0x00000032;  //Activate Ports F,E,B
	delay = SYSCTL_RCGC2_R;        // Allow time for clock to start
	//F
	GPIO_PORTF_LOCK_R = 0x4C4F434B; //Unlock GPIO Port F
	GPIO_PORTF_CR_R = 0x0A;        //Allow changes to PF 1,3
	GPIO_PORTF_AMSEL_R = 0x00;     //Disable analog on PF
	GPIO_PORTF_PCTL_R = 0x0A;      //PCTL GPIO on PF 1,3
	GPIO_PORTF_DIR_R = 0x0A;       //PF 1, 3 outputs
	GPIO_PORTF_DEN_R = 0x0A;       //Enable digital I/O on PF 1,3
	
	//E
	GPIO_PORTE_LOCK_R = 0x4C4F434B; //Unlock GPIO Port E
	GPIO_PORTE_CR_R = 0x07;        //Allow changes to PE 1,2,3
	GPIO_PORTE_AMSEL_R = 0x00;     //Disable analog on PE
	GPIO_PORTE_PCTL_R = 0x07;      //PCTL GPIO on PE 1,2,3
	GPIO_PORTE_DIR_R = 0x00;       //PE 1,2,3 inputs
	GPIO_PORTE_DEN_R = 0x07;       //Enable digital I/O on PE 1,2,3
	
	//B
	GPIO_PORTB_LOCK_R = 0x4C4F434B; //Unlock GPIO Port B
	GPIO_PORTB_CR_R = 0x3F;        //Allow changes to PB 0-5
	GPIO_PORTB_AMSEL_R = 0x00;     //Disable analog on PB
	GPIO_PORTB_PCTL_R = 0x3F;      //PCTL GPIO on PB 0-5
	GPIO_PORTB_DIR_R = 0x3F;       //PB 0-5 outputs
	GPIO_PORTB_DEN_R = 0x3F;       //Enable digital I/O on PB 0-5
	
	
	
	
	
	
  EnableInterrupts(); // TExaS uses interrupts
  
	CS = S_ON;  // start state
	while(1){   // FSM Engine 
		WalkLight = FSM[CS].LED_Walk;
		RoadLight = FSM[CS].LED_Road;
		SysTick_Wait10ms(FSM[CS].Wait);
    Input = Buttons;
    CS = FSM[CS].Next[Input]; 
  }
}

