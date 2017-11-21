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
#define PI 3.14159265

#define START_DUTY 0
#define Xaxis 0
#define Yaxis 1
#define Zaxis 2
#define STOP 0
#define PLUS 1
#define MINUS 2
#define CW 1 
#define CCW 2
#define MOTOR_L 1
#define MOTOR_R 2

#define RX1Q_LN 8
#define RX_CMND_FRM_LN 25
#define START_CHR   '['
#define END_CHR     ']'
#define CONDITION_S '('
#define CONDITION_E ')'
#define CONDITION_M ','
//UART Queue
#define TX1Q_LN 128

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
static void GlobalVarInit (void);
static void DynamicMemInit (void);
static void UARTQueueInit (void);
static int8u SendTx1 (int8u *strPtr);

//My Function
static void create_position_array(void);
static int8u ConvertStr2Int (int8u num);
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
	float r, dr, ddr;
	float x_d, dx_d, ddx_d;
	float y_d, dy_d, ddy_d;
	float e_x, e_y;
	float int_e_x = 0.0f;
	float int_e_y = 0.0f;
	float p_x, p_y;
	float dx, dy;

float Kp = 0.0f,Kp_2 = 0.0f;
float Ki = 0.0f,Ki_2 = 0.0f;
float Kd = 0.0f,Kd_2 = 0.0f;
int16s e = 0,e_2 = 0;
float int_e = 0,int_e_2 = 0;
float u = 0,u_2 = 0;
float control = 0,control_2 = 0;
float p_e = 0,p_e_2 = 0;
float tor = 3;
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
int16u ReadPortB,B0,B1,B2,B3;
float radius = 11.46;

int home_state = 1;
char move_state;

static volatile Q8UX_STRUCT Tx1QCB;
static volatile int8u Tx1QArray[TX1Q_LN];
static volatile int8u *Tx1BuffPtr;
static volatile int16u TxBuffIdx;
static volatile TX1_STATUS Tx1Flag;
static volatile int16u Tx1FrameIn, Tx1FrameOut, Rx1FrameCount, RxCount,
                        Tx1QFullCount, Rx1QFullCount;

static volatile int8u *RxBuffPtr;
static volatile QPTRX_STRUCT Rx1QCB;
static volatile PTR_STRUCT Rx1BuffPtrArray[RX1Q_LN];
static volatile PTR_STRUCT DestPtrStruct;

static volatile int16u MemFail, MemCount;

static int position_x[50];
static int position_y[50];

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
	
	if(speed == 0 || direction == STOP){
		move_state = 'z';
		if(axis == MOTOR_L){
			output_low(IN3);
			output_low(IN4);
		}
		else if(axis == MOTOR_R){
			output_low(IN1);
			output_low(IN2);
		}	
			set_pwm_duty(1, 400);
			set_pwm_duty(2, 400);
	}
	else if(axis == MOTOR_L){
		if(direction == CW){
			output_high(IN3);
			output_low(IN4);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}
		else if(direction == CCW){
			output_low(IN3);
			output_high(IN4);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}
	}
	else if(axis == MOTOR_R){
		if(direction == CW){
			output_high(IN1);    
			output_low(IN2);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}
		else if(direction == CCW){
			output_low(IN1);    
			output_high(IN2);
			set_pwm_duty(1, speed);
			set_pwm_duty(2, speed);
		}
	}
	
}

void trajectory(float currentX, float currenty, float R, float theta)
{
	A_x = 0;
	Kp_x = 0;
	Ki_x = 0;
	Kd_x = 0;

	A_y = 0;
	Kp_y = 0;
	Ki_y = 0;
	Kd_y = 0;

	a = PI/180;
	T = 5;

	r = (R/powf(T,3))*(10*powf(t,3) - 15*powf(t,4)/T + 6*powf(t,5)/powf(T,2));
	dr = (R/powf(T,3))*(30*powf(t,2) - 60*powf(t,3)/T + 30*powf(t,4)/powf(T,2));
	ddr = (R/powf(T,3))*(60*powf(t,1) - 180*powf(t,2)/T + 120*powf(t,3)/powf(T,2));
	x_d = (1/(2*a))*r*(cos(theta)+sin(theta));
	dx_d = (1/(2*a))*dr*(cos(theta)+sin(theta));
	ddx_d = (1/(2*a))*ddr*(cos(theta)+sin(theta));
	y_d = (1/(2*a))*r*(cos(theta)-sin(theta));
	dy_d = (1/(2*a))*dr*(cos(theta)-sin(theta));
	ddy_d = (1/(2*a))*ddr*(cos(theta)-sin(theta));

	e_x = dx_d - currentX;
	int_e_x = int_e_x + e_x;
	p_x = currentX;
	dx = currentX - p_x;
	e_y = dy_d - currentY;
	int_e_y = int_e_y + e_y;
	p_y = currentY;
	dy = currentY - p_y;

	u_x = A_x*(ddx_d) + Kp_x*e + Ki_x*int_e_x + Kd_x*(dx_d-dx);
	u_y = A_y*(ddy_d) + Kp_y*e + Ki_y*int_e_y + Kd_y*(dy_d-dy);

	
}

void control_position(int axis, float target, float currentPosition) {
	if(axis == MOTOR_L){
		float targetX = target;
		float currentX = currentPosition;
		Kp = 8;         //old value = 8
		Ki = 0.001;     //old value = 0.001
		Kd = 2.5;       //old value = 2.5

		e = targetX - currentX;
		int_e = int_e + e;
		u = Kp*e + Ki*int_e + Kd*(e-p_e);
		int h = u;
		printf("\n%d",h);
		p_e = e;  
		
		if (abs(e)>tor){
			if(abs(u) > 255){
				control = 255;	
			}
			else if(abs(u)<50){
				control = abs((50/255)*400);
			}else{
				control = abs((u/255)*400);
			}
			
			if (u>0){
				drive_motor(axis,CW,control);
			}
			else{
				drive_motor(axis,CCW,control);
			}
		}else{
				drive_motor(Xaxis,STOP,0);
		}
	}
	else if(axis == MOTOR_R){
		float targetY = target;
		float currentY = currentPosition;
		Kp_2 = 9.5;          //old value = 9.5
		Ki_2 = 0.000000006;  //old value = 0.000000006
		Kd_2 = 2;			 //old value = 2

		e_2 = targetY - currentY;
		int_e_2 = int_e_2 + e_2;
		u_2 = Kp_2*e_2 + Ki_2*int_e_2 + Kd_2*(e_2-p_e_2);
		int h = u_2;
		printf("\n%d",h);
		p_e_2 = e_2;  
		
		if (abs(e_2)>tor){
			if(abs(u_2) > 255){
				control_2 = 255;
			}
			else if(abs(u_2) < 50){
				control_2 = abs((50/255)*400);
			}else{
				control_2 = abs((u_2/255)*400);
			}
			if (u_2>0){
				drive_motor(axis,CW,control_2);
			}
			else{
				drive_motor(axis,CCW,control_2);
			}
			
		}else{
			drive_motor(Yaxis,STOP,0);
		}
		
	}
	
}
void Encoder(void){
	thetaX = (count_pulseX*7.5)/64;
	thetaY = (count_pulseY*7.5)/64;
	current_posX = (thetaX*(22/7)*radius)/180;
	current_posY = (thetaY*(22/7)*radius)/180;
}

void move_motor(char state){
	if(state == 'w'){
		control_position(MOTOR_R,-20,current_posY);
        control_position(MOTOR_L,20,current_posX);
	}
	else if (state == 's'){
		control_position(MOTOR_R,20,current_posY);
        control_position(MOTOR_L,-20,current_posX);
	}
	else if (state == 'a'){
		control_position(MOTOR_R,-20,current_posY);
        control_position(MOTOR_L,-20,current_posX);
	}
	else if (state == 'd'){
		control_position(MOTOR_R,20,current_posY);
        control_position(MOTOR_L,20,current_posX);
	}
	else if (state == '1'){
		control_position(MOTOR_L,50,current_posX);
	}
	else if (state == '2'){
		control_position(MOTOR_L,-50,current_posX);
	}
	else if (state == '3'){
		control_position(MOTOR_R,50,current_posY);
	}
	else if (state == '4'){
		control_position(MOTOR_R,-50,current_posY);
	}
	else if (state == 'z'){
		disable_interrupts(INT_TIMER4);
	}
}
/****************************************************************************/
/**                                                                        **/
/**                             LOCAL FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/
int main (void)
 {		
	int8u errCode, sendTx1Count;

    DisableIntr ();
    HardwareInit ();
    GlobalVarInit ();
    DynamicMemInit ();
    UARTQueueInit ();
    EnableIntr ();
    for(;;){
        DisableIntr();
        QPtrXGet(&Rx1QCB, &DestPtrStruct, &errCode);
        if (errCode == Q_OK)
        {
            //create_position_array();
        	EnableIntr ();
            if (DestPtrStruct.blockPtr[0] == 'w')
            {
                /* Move forward */
                move_state = 'w';
                clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);

            }
            else if (DestPtrStruct.blockPtr[0] == 's')
            {
                /* Move backward */
                move_state = 's';
                clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);
            }
            else if (DestPtrStruct.blockPtr[0] == 'a')
            {
                /* Move left */
                move_state = 'a';
                clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);   
				
    	    }
            else if (DestPtrStruct.blockPtr[0] == 'd')
            {
                /* Move right */
                move_state = 'd';
                clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);
            }
            else if (DestPtrStruct.blockPtr[0] == '1')
            {
                /* Move motor left cw */
                move_state = '1';
                clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);
            }
            else if (DestPtrStruct.blockPtr[0] == '2')
            {
                /* Move motor left ccw */
                move_state = '2';
                clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);
            }
            else if (DestPtrStruct.blockPtr[0] == '3')
            {
                /* Move motor right cw */
                move_state = '3';
                clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);
            }
            else if (DestPtrStruct.blockPtr[0] == '4')
            {
                /* Move motor right ccw */
                move_state = '4';
                clear_interrupt(INT_TIMER4);
				enable_interrupts(INT_TIMER4);
            }
            else if (DestPtrStruct.blockPtr[0] == 'h')
            {
            	/* Home */
            	clear_interrupt(INT_TIMER1);
            	enable_interrupts(INT_TIMER1);
            }

            //sendTx1Count = SendTx1 (((int8u *)DestPtrStruct.blockPtr));
            //putc('F');
            free ((void *)DestPtrStruct.blockPtr);
            MemCount--;
            //EnableIntr();
        }
        else
        {
            EnableIntr();
        }
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
	
	/*clear_interrupt(INT_TIMER5);
	enable_interrupts(INT_TIMER5);*/

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

static void GlobalVarInit (void)
{
    Tx1Flag = TX1_READY;
    TxBuffIdx = 0;
    Tx1FrameIn = 0;
    Tx1QFullCount = 0;
    Rx1FrameCount = 0;
    RxCount = 0;
    Rx1QFullCount = 0;
    MemFail = 0;
    MemCount = 0;
    return;
}
static void DynamicMemInit (void)
{
    //get mem block for first Rx1 buffer
    RxBuffPtr = (int8u *)malloc ((sizeof (int8u)) * RX_CMND_FRM_LN);
    if (RxBuffPtr != (int8u *)NULL)
    {
        MemCount++;
        clear_interrupt(INT_RDA);
        enable_interrupts (INT_RDA);
    }
    else
    {
        MemFail++;
    }
    return;
}
static void UARTQueueInit (void)
{
    QPtrXInit(&Rx1QCB, Rx1BuffPtrArray, RX1Q_LN);
    Q8UXInit(&Tx1QCB, Tx1QArray, TX1Q_LN);
    return;
}
static int8u SendTx1 (int8u *strPtr)//critical section
{
    int8u strLn;
    int8u strIdx;
    int8u qSpace;
    int8u errCode;
    int8u count;
    count = 0;
    strLn = strlen(strPtr);
    if (strLn != 0)
    {
        qSpace = TX1Q_LN - Q8UXCount (&Tx1QCB);
        if (qSpace >= (int16u)strLn)
        {
            for (strIdx = 0; strIdx < strLn; strIdx++)
            {
                Q8UXPut (&Tx1QCB, strPtr[strIdx], &errCode);
                count++;
            }
            if (Tx1Flag == TX1_READY)
            {
                Tx1Flag = TX1_BUSY; // Set Rx1 to Busy.
                TX1IF = 1;// Start Tx1 Interupt. 
                enable_interrupts(INT_TBE);
            }
        }
    }
    return count;
}

/****************************************************************************/

/****************************************************************************/
/**                               My Function                              **/
/****************************************************************************/
static void create_position_array(void)
{
    int8u c_state = 0;
    int8u s_state = 0;
    int8u sum_a = 0; 
    int8u sum_b = 0;
    int8u sum_c = 0;
    int8u pos_r = 0;
    int *posY;
    int *posX;
    int8u Arr_state = 0;

    posY = position_y;
    posX = position_x;

    for(int8u i = 0; i < strlen(DestPtrStruct.blockPtr); i++)
    {
        if(DestPtrStruct.blockPtr[i] == CONDITION_S)
        {
            c_state = 1;
        }
        else if(DestPtrStruct.blockPtr[i] == CONDITION_M)
        {
            if(c_state == 1)
            {
                //Found x
                if(s_state==1){pos_r = sum_a;}
                if(s_state==2){pos_r = sum_a*10+sum_b;}
                if(s_state==3){pos_r = sum_a*100+sum_b*10+sum_c;}
                //printf("\r\n x == %d", pos_r);
                *(posX+Arr_state) = pos_r;
                s_state = 0;
                c_state = 0;
            }
        }
        else if(DestPtrStruct.blockPtr[i] == CONDITION_E)
        {
            //Found y
            if(s_state==1){pos_r = sum_a;}
            if(s_state==2){pos_r = sum_a*10+sum_b;}
            if(s_state==3){pos_r = sum_a*100+sum_b*10+sum_c;}
            //printf("\r\n y == %d", pos_r);
            *(posY+Arr_state) = pos_r;
            s_state = 0;
            Arr_state++;
        }
        else
        {
            //Found numeric
            //printf("\r\nDestPtrStruct.blockPtr[i] = %c", DestPtrStruct.blockPtr[i]);
            if(s_state == 0){sum_a = ConvertStr2Int(DestPtrStruct.blockPtr[i]);}
            if(s_state == 1){sum_b = ConvertStr2Int(DestPtrStruct.blockPtr[i]);}
            if(s_state == 2){sum_c = ConvertStr2Int(DestPtrStruct.blockPtr[i]);}
            s_state++;
        }
    }
    return;
}        

static int8u ConvertStr2Int (int8u num)
{
	return num - '0';
}
/****************************************************************************/

/****************************************************************************/
/**                           Interrupt Functions                          **/
/****************************************************************************/

#INT_RDA //Rx1 interupt
void RDA1 (void)
{
    static FRAME_STATE FrameState = FRAME_WAIT;
    static int16u FrmIdx = 0;
    int8u Chr;
    int8u *errCode;

    Chr = getc(); // Read data from Rx1 register
    RxCount++;
    switch (FrameState) // State machine for build frame
    {
        case FRAME_WAIT: // waits for start char of new frame
            if (Chr == START_CHR)
            {
                //Get a start char
                //RxBuffPtr[FrmIdx] = Chr; // Save start char to frame
                //FrmIdx++;
                FrameState = FRAME_PROGRESS; // Change state to build frame
            }
            break;
        case FRAME_PROGRESS: // Build frame
            if ((FrmIdx == (RX_CMND_FRM_LN - 2)) && (Chr != END_CHR))
            {
                //Characters exceed frame lenght. Frame error.
                // Rejects data and black to wait new frame.
                FrmIdx = 0;
                FrameState = FRAME_WAIT;
            }
            else if (Chr == END_CHR)
            {
                //Get and char. Frame completes.
                //RxBuffPtr[FrmIdx] = Chr;
                //FrmIdx++;
                RxBuffPtr[FrmIdx] = 0;
                FrmIdx = 0;
                Rx1FrameCount++;
                //Sends Rx1 event to event queue.
                QPtrXPut (&Rx1QCB, (void *)RxBuffPtr, &errCode);
                if (errCode == Q_FULL)
                {
                    //EvQ full error. Free mem of data block.
                    free ((void *)RxBuffPtr);
                    MemCount--;
                    Rx1QFullCount++;
                }
                FrameState = FRAME_WAIT; // Back to wait for start char of new frame.
                // GEt mem block for better of new frame
                RxBuffPtr = (int8u *)malloc ((sizeof (int8u)) * RX_CMND_FRM_LN);
                if (RxBuffPtr == (int8u *)NULL)
                {
                    //Can not get mem block. Disable Rx1 interupt.
                    disable_interrupts(INT_RDA);
                    MemFail++;
                }
                else
                {
                    MemCount++;
                }
            }
            else
            {
                RxBuffPtr[FrmIdx] = Chr;
                FrmIdx++;
            }
            break;
        default:
            break;
    }
    return;
}

#INT_TBE // Tx1 interupt
void TBE1ISR (void)
{
    int8u destChr;
    Q_ERR errCode;
    Q8UXGet (&Tx1QCB, &destChr, &errCode);
    if (errCode == Q_OK)
    {
        putc(destChr);
    }
    else
    {
        disable_interrupts (INT_TBE);
        Tx1Flag = TX1_READY;
    }
    return;
}

#INT_TIMER1
void Home(void){
	printf("\nINTERRUPTS TIMER1");
	if(home_state == 1){
		drive_motor(MOTOR_R,CCW,100);
		drive_motor(MOTOR_L,CCW,100);
	}
	else if(home_state == 2){
		drive_motor(MOTOR_R,CCW,100);
		drive_motor(MOTOR_L,CW,100);
	}
	return;
}

#INT_EXT1
void Initial_Y_axis(void){
	move_state = 'z';
	count_pulseY = 0;
	thetaY = 0;
	current_posY = 0;
	drive_motor(MOTOR_R,STOP,0);
	drive_motor(MOTOR_L,STOP,0);
	disable_interrupts(INT_TIMER1);
	//disable_interrupts(INT_EXT2);
	
	/*clear_interrupt(INT_TIMER4);
	enable_interrupts(INT_TIMER4);*/
	return;
}
#INT_EXT2
void Initial_X_axis(void){
	count_pulseX = 0;
	thetaX = 0;
	current_posX = 0;
	home_state = 2;
	//drive_motor(Yaxis,PLUS,0);
	disable_interrupts(INT_EXT2);
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
		count_pulseY--;
	}
	else{
		count_pulseY++;
	}
	//printf("\ncount_pulseY = %d",count_pulseY);
	return;
}

#INT_IC4
void Encoder2_B(void){
	ReadPortB = input_b() & 0x000c;  
	B3 = (ReadPortB & 0x0008) >> 3;
	if((B2 ^ B3) == 0){
		count_pulseY++;
	}
	else{
		count_pulseY--;
	}
	//printf("\ncount_pulseY = %d",count_pulseY);
	return;
}

#INT_TIMER4
void Run_Motor(void){
	disable_interrupts(INT_TIMER5);
	Encoder();
	move_motor(move_state);
	/*printf("\nSwitch Right");
	control_position(MOTOR_L,72,current_posX);
	control_position(MOTOR_R,72,current_posY);*/

	//control_position(MOTOR_L,360,thetaX);	
	return;
}

#INT_TIMER5
void Run_Motor2(void){
	disable_interrupts(INT_TIMER4);
	Encoder();
	int t = thetaY;
	printf("\n%d",t);
	drive_motor(MOTOR_L,CCW,150);
	drive_motor(MOTOR_R,CW,150);
	//control_position(MOTOR_L,360,current);
	//control_position(MOTOR_R,72,current_posY);
	return;
}


/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/**                                 EOF                                    **/
/**                                                                        **/
/****************************************************************************/