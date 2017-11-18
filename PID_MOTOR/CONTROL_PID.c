/*
*************************** C SOURCE FILE ************************************

project   :
filename  : CONTROL_PID.C
version   : 2
date      :

******************************************************************************

Copyright (c) 20xx
All rights reserved.

******************************************************************************

VERSION HISTORY:
----------------------------
Version      : 1
Date         :
Revised by   :
Description  :

Version      : 2
Date         :
Revised by   :
Description  : *
               *
               *

******************************************************************************
*/

#define CONTROL_PID_C_SRC

/****************************************************************************/
/**                                                                        **/
/**                             MODULES USED                               **/
/**                                                                        **/
/****************************************************************************/

#include "CONTROL_PID.h"
#include "BITWISE_LIB.h"

/****************************************************************************/
/**                                                                        **/
/**                        DEFINITIONS AND MACROS                          **/
/**                                                                        **/
/****************************************************************************/
#define START_DUTY 0
#define Xaxis 0
#define Yaxis 1
#define Zaxis 2
#define STOP 0
#define PLUS 1
#define MINUS 2

/****************************************************************************/
/**                                                                        **/
/**                        TYPEDEFS AND STRUCTURES                         **/
/**                                                                        **/
/****************************************************************************/


/****************************************************************************/
/**                                                                        **/
/**                      PROTOTYPES OF LOCAL FUNCTIONS                     **/
/**                                                                        **/
/****************************************************************************/
static void HardwareInit (void);

/****************************************************************************/
/**                                                                        **/
/**                           EXPORTED VARIABLES                           **/
/**                                                                        **/
/****************************************************************************/


/****************************************************************************/
/**                                                                        **/
/**                            GLOBAL VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/
float Kp = 0.0f,Kp_2 = 0.0f;
float Ki = 0.0f,Ki_2 = 0.0f;
float Kd = 0.0f,Kd_2 = 0.0f;
int16s e = 0,e_2 = 0;
float int_e = 0,int_e_2 = 0;
float u = 0,u_2 = 0;
float control = 0,control_2 = 0;
float p_e = 0,p_e_2 = 0;
float tor = 0.5;
static int16u duty1;
int32s count_pulseX = 0; 
int32s count_pulseY = 0; 
float thetaX = 0;
float thetaY = 0;
int target_posX = 180;
int target_posY = 0;
int h;
static float current_posX = 0;
static float current_posY = 0;
float current_pos_show = 0;
int16u ReadPortB,B0,B1,B2,B3;
float radius = 11.46;

int home_state = 1;

/****************************************************************************/
/**                                                                        **/
/**                           EXPORTED FUNCTIONS                           **/
/**                                                                        **/
/****************************************************************************/
void drive_motor(int axis,int direction,int speed){
/***************************
	MotorLeft(CW) >> IN1  H
				  >> IN2  L
	MotorRight(CW) >> IN3  H
	               >> IN4  L
	Y_PLUS = Left(CW) + Right(CCW)
	X_PLUS = Left(CW) + Right(CW)
****************************/
	
	if(speed == 0){
			output_high(IN1);
			output_high(IN2);
			output_high(IN3);
			output_high(IN4);
	}
	else if(axis == Xaxis){
		if(direction == MINUS){
			output_low(IN1);    
			output_high(IN2);
			output_low(IN3);
			output_high(IN4);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);		
		}
		else if(direction == PLUS){	
			output_high(IN1);    
			output_low(IN2);	    
			output_high(IN3);
			output_low(IN4);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}
	}

	else if(axis == Yaxis){
		if(direction == PLUS){
			output_low(IN1);
			output_high(IN2);
			output_high(IN3);
			output_low(IN4);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}
		else if(direction == MINUS){
			output_high(IN1);
			output_low(IN2);
			output_low(IN3);
			output_high(IN4);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}	
	}	
}

void control_position(int axis, float target, float currentPosition) {
	if(axis == Xaxis){
		Kp = 3.10588;
		Ki = 0.00001;
		Kd = 1.5;
		
		e = target - currentPosition;
		int_e = int_e + e;
		u = Kp*e + Ki*int_e + Kd*(e-p_e);
		p_e = e;  
		
		if (abs(e)>tor){
			if(u > 255){
				control = 255;	
			}
			else{
				control = abs(u);
			}
		}
		if (u>0){
			drive_motor(axis,PLUS,control);
		}
		else{
			drive_motor(axis,MINUS,control);
		}
	}
	else if(axis == Yaxis){
		Kp_2 = 3.07811;
		Ki_2 = 0.00001;
		Kd_2 = 1.5;
		
		e_2 = target - currentPosition;
		int_e_2 = int_e_2 + e_2;
		u_2 = Kp_2*e_2 + Ki_2*int_e_2 + Kd_2*(e_2-p_e_2);
		p_e_2 = e_2;  
		
		if (abs(e_2)>tor){
			if(u_2 > 255){
				control_2 = 255;	
			}
			else{
				control_2 = abs(u_2);
			}
		}
		if (u_2>0){
			drive_motor(axis,PLUS,control_2);
		}
		else{
			drive_motor(axis,MINUS,control_2);
		}
	}
}

/****************************************************************************/
/**                                                                        **/
/**                             LOCAL FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/
int main (void)
 {		
	disable_interrupts(INTR_GLOBAL);
	HardwareInit ();	
	enable_interrupts(INTR_GLOBAL);
	for (;;)
	{
	}
	return 0;
}

static void HardwareInit (void)
{
	setup_adc_ports(NO_ANALOGS);
	set_tris_a (get_tris_a () & 0xffe8); //1111 1111 1110 1000
	set_tris_b (get_tris_b () & 0xff3f); //1111 1111 0011 1111
	B0 = (ReadPortB & 0x0001);
	B1 = (ReadPortB & 0x0002) >> 1;
	B2 = (ReadPortB & 0x0004) >> 2;
	B3 = (ReadPortB & 0x0008) >> 3;

	setup_capture(1,CAPTURE_EE | INTERRUPT_EVERY_CAPTURE |CAPTURE_TIMER2);
	clear_interrupt(INT_IC1);
	enable_interrupts(INT_IC1);
	
	setup_capture(2,CAPTURE_EE | INTERRUPT_EVERY_CAPTURE |CAPTURE_TIMER2);
	clear_interrupt(INT_IC2);
	enable_interrupts(INT_IC2);

	setup_capture(3,CAPTURE_EE | INTERRUPT_EVERY_CAPTURE |CAPTURE_TIMER2);
	clear_interrupt(INT_IC3);
	enable_interrupts(INT_IC3);

	setup_capture(4,CAPTURE_EE | INTERRUPT_EVERY_CAPTURE |CAPTURE_TIMER2);
	clear_interrupt(INT_IC4);
	enable_interrupts(INT_IC4);

	clear_interrupt(INT_EXT1);
	enable_interrupts(INT_EXT1);
	
	clear_interrupt(INT_EXT2);
	enable_interrupts(INT_EXT2);
	
	/*clear_interrupt(INT_TIMER1);
	enable_interrupts(INT_TIMER1);*/

	/*clear_interrupt(INT_TIMER4);
	enable_interrupts(INT_TIMER4);*/

	duty1 = START_DUTY;
	set_compare_time(1, duty1, duty1);
	setup_compare(1, COMPARE_PWM | COMPARE_TIMER3);

	set_compare_time(2, duty1, duty1);
	setup_compare(2, COMPARE_PWM | COMPARE_TIMER3);
	
	set_timer1(0);
	setup_timer1(TMR_INTERNAL | TMR_DIV_BY_256, 6249); 	
	
	set_timer2(0);
	setup_timer2(TMR_INTERNAL | TMR_DIV_BY_1, 65535);

	set_timer3(0);
	setup_timer3(TMR_INTERNAL | TMR_DIV_BY_1, 400);

	set_timer4(0);
	setup_timer4 (TMR_INTERNAL | TMR_DIV_BY_256, 6249);

	set_timer5(0);
	setup_timer5 (TMR_INTERNAL | TMR_DIV_BY_256, 6249);
	return;
}

#INT_TIMER1
void Home(void){
	if(home_state == 1){
		drive_motor(Xaxis,MINUS,80);
	}
	else if(home_state == 2){
		drive_motor(Yaxis,PLUS,80);
	}
	return;
}

#INT_EXT1
void Initial_X_axis(void){
	clear_interrupt(INT_TIMER4);
	enable_interrupts(INT_TIMER4);
	return;
}

#INT_EXT2
void Initial_Y_axis(void){
	clear_interrupt(INT_TIMER5);
	enable_interrupts(INT_TIMER5);
	return;
}	

#INT_IC1
void Encoder1_A(void){
	ReadPortB = input_b() & 0x0003;  
	B0 = (ReadPortB & 0x0001);
	if((B0 ^ B1) == 0){
		count_pulseX--;
	}
	else{
		count_pulseX++;
	}
	//printf("\ncount_pulseX = %d",count_pulseX);
	return;
}

#INT_IC2
void Encoder1_B(void){
	ReadPortB = input_b() & 0x0003;  
	B1 = (ReadPortB & 0x0002 ) >> 1;
	if((B0 ^ B1) == 0){
		count_pulseX++;
	}
	else{
		count_pulseX--;
	}
	//printf("\ncount_pulseX = %d",count_pulseX);
	return;
}
	
#INT_IC3
void Encoder2_A(void){
	ReadPortB = input_b() & 0x000c; 
	B2 = (ReadPortB & 0x0004) >> 2; 
	if((B2 ^ B3) == 0){
		count_pulseY++;
	}
	else{
		count_pulseY--;
	}
	//printf("\ncount_pulseY = %d",count_pulseY);
	return;
}

#INT_IC4
void Encoder2_B(void){
	ReadPortB = input_b() & 0x000c;  
	B3 = (ReadPortB & 0x0008) >> 3;
	if((B2 ^ B3) == 0){
		count_pulseY--;
	}
	else{
		count_pulseY++;
	}
	//printf("\ncount_pulseY = %d",count_pulseY);
	return;
}

#INT_TIMER4
void Run_Motor(void){
	thetaX = (count_pulseX*7.5)/64;
	thetaY = (count_pulseY*7.5)/64;
	current_posX = (thetaX*(22/7)*radius)/180;
	current_posY = (thetaY*(22/7)*radius)/180;
	int h = thetaX;
	printf("\n\r%d",h);
	//current_pos_show = (current_posX*(22/7)) * radius * 10;
	//control_position(Yaxis, target_posY, current_posY);
	control_position(Xaxis, 360 , thetaY);
	//drive_motor(Yaxis,MINUS,100);
	disable_interrupts(INT_TIMER5);
	return;
}

#INT_TIMER5
void Run_Motor2(void){
	thetaX = (count_pulseX*7.5)/64;
	thetaY = (count_pulseY*7.5)/64;
	current_posX = (thetaX*(22/7)*radius)/180;
	current_posY = (thetaY*(22/7)*radius)/180;
	int h = thetaX;
	printf("\n\r%d",h);
	//current_pos_show = (current_posX*(22/7)) * radius * 10;
	//control_position(Yaxis, target_posY, current_posY);
	control_position(Xaxis, -360 , thetaY);
	//drive_motor(Yaxis,MINUS,100);
	disable_interrupts(INT_TIMER4);
	return;
}


/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/**                                 EOF                                    **/
/**                                                                        **/
/****************************************************************************/