/* ========================================
 * Fully working code: 
 * PWM      : 
 * Encoder  : 
 * ADC      :
 * USB      : port displays speed and position.
 * CMD: "PW xx"
 * Copyright Univ of Auckland, 2016
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Univ of Auckland.
 *
 * ========================================
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <project.h>
#include <astar.h>
//* ========================================
#include "defines.h"
#include "vars.h"
#include "ADC.h"
#include <stdbool.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

//* ========================================
void usbPutString(char *s);
void usbPutChar(char c);
void handle_usb();
void ADC_ISR_Handler(void);
void changeDutyCycle();
void controller(uint8_t type);
void stop();
void rightTurn();
void leftTurn();
void prepTurn();
void processSamples();
void u_turn();
void path_finding();
void u_turn_pre();
void distance_cal();


//* ========================================
#define THRESHOLD_MV 2100 // Define the threshold in millivolts
#define SAMPLE_COUNT 300
#define STARTING_DIRECTION 1 // 0 right, 1 down, 2 left, 3 up
// Array to hold ADC samples
volatile uint16 adcSamples0[SAMPLE_COUNT];  
volatile uint16 adcSamples1[SAMPLE_COUNT];  
volatile uint16 adcSamples2[SAMPLE_COUNT];
volatile uint16 adcSamples3[SAMPLE_COUNT];  
volatile uint16 adcSamples5[SAMPLE_COUNT];  
volatile uint16 adcSamples6[SAMPLE_COUNT];  
volatile uint16 sampleIndex = 0;
volatile uint8 adcComplete = 0;

int32 voltage1, voltage2,voltage3, voltage4, voltage5, voltage6;
int32 min1, min2;
int32 max1, max2;
int16 count=0;
bool sensor_1, sensor_2, sensor_3, sensor_4, sensor_5, sensor_6=0;
bool forwarding_state, redirecting,correction=1;
bool u_turn_state,next_state, leftturn_state, rightturn_state,final_way_state=0;
bool flag;
int temp,temp_left;
volatile int step=1;
volatile int move;
int dis;
//int directions[] = {1,0,1,2,2,1,4,1};
int direction_count;
int directions[15*19+2];
// for path finding
Point start; 
//Point end[] = {{7,3},{11,7},{11,3},{3,11},{13,17},{9,9}};
Point end[] = {{1, 1},{9,1},{5,5},{1,7},{5,13},{9,9}};
//Point end[] = {{13,7},{11,11},{11,3},{7,11},{7,15},{13,17}};
//Point end[] = {{5,17},{5,16},{11,1},{7,11},{7,9},{13,16}};
//Point end[] = {{5,7},{13,11},{1,1},{5,13},{1,7},{13,6}};
//int direction_count = 0;
int motor_direction;
int32 end_count=0;

CY_ISR(TimerSample)
{
    temp = QuadDec_M2_GetCounter();
    temp_left = QuadDec_M1_GetCounter();
    Timer_TS_ReadStatusRegister();
    flag=1;
}


CY_ISR(ADC_ISR_Handler)
{
    if (sampleIndex < SAMPLE_COUNT)
    {
        adcSamples0[sampleIndex] =  ADC_GetResult16(0);
        adcSamples1[sampleIndex] = ADC_GetResult16(1);
        adcSamples2[sampleIndex] = ADC_GetResult16(2);
        adcSamples3[sampleIndex] = ADC_GetResult16(3);
        adcSamples5[sampleIndex] = ADC_GetResult16(5);
        adcSamples6[sampleIndex] = ADC_GetResult16(6);
        sampleIndex++;
    }
    else
    {
        processSamples();  // Process the samples to find peak-to-peak
        sampleIndex = 0;  // Reset sample index
    }
}

void processSamples(void)
{
    volatile uint16 maxValue0 = 0;
    volatile uint16 maxValue1 = 0;
    volatile uint16 maxValue2 = 0;
    volatile uint16 maxValue3 = 0;
    volatile uint16 maxValue6 = 0;
    volatile uint16 maxValue5 = 0;
    uint16 i;

    // Calculate min, max, and sum
    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        if (adcSamples0[i] > maxValue0){
            maxValue0 = adcSamples0[i];
        }
        if (adcSamples1[i] > maxValue1){
            maxValue1 = adcSamples1[i];
        }
        if (adcSamples2[i] > maxValue2){
            maxValue2 = adcSamples2[i];
        }
        if (adcSamples3[i] > maxValue3){
            maxValue3 = adcSamples3[i];
        }
        if (adcSamples5[i] > maxValue5){
            maxValue5 = adcSamples5[i];
        }
        if (adcSamples6[i] > maxValue6){
            maxValue6 = adcSamples6[i];
        }
    }
   // light LED if average peak-to-peak is less than 300mV
    if (maxValue0 > THRESHOLD_MV){
        sensor_2=0;
    }else{
        sensor_2=1;}
    
    if (maxValue1 > 2030){
        sensor_3=0;
    }else{
        sensor_3=1;}
    
    if ( maxValue2 > THRESHOLD_MV){
        sensor_1=0;
    }else{
        sensor_1=1;} 
    
    if(maxValue3> THRESHOLD_MV){
        sensor_4=0;
    }else{
        sensor_4=1;}
    
    if(maxValue5>THRESHOLD_MV){
        sensor_6=0;
    }else{
        sensor_6=1;}
    
    if(maxValue6>THRESHOLD_MV){
        sensor_5=0;
    }else{
        sensor_5=1;}
}


int main(void)
{
// --------------------------------    
// ----- INITIALIZATIONS ----------
    CYGlobalIntEnable;
    // Initialize and start components
    ADC_Start();
    ADC_StartConvert();
    Timer_TS_Start();
    PWM_1_Start();
    PWM_2_Start();
    QuadDec_M1_Start();
    QuadDec_M2_Start();

    // Set up the interrupt
    isr_ADC_StartEx(ADC_ISR_Handler);
    isr_TS_StartEx(TimerSample);
    
// ------USB SETUP ----------------    
#ifdef USE_USB    
    USBUART_Start(0,USBUART_5V_OPERATION);
#endif        
        
//    move the motor forward and delay 0.1 second for waiting sensor response
    changeDutyCycle();
    CyDelay(100);

// find the first path
    path_finding(STARTING_DIRECTION);
    
//    usbPutString(displaystring);
    for(;;)
    {        

        // when the motor meet a turning junction, it will decide:
        if(correction &&((sensor_1 && sensor_2 && forwarding_state) || (sensor_5 && sensor_4 && forwarding_state)|| redirecting) && !final_way_state){
            forwarding_state=0;
            correction=0;
            move =  directions[step];
            step++;
            if(step == (direction_count-2)){
                QuadDec_M2_SetCounter(0);
                final_way_state=1;
            }
            switch(move){
                case 0: // not turning
                    forwarding_state=1;
                    leftturn_state=0;
                    rightturn_state=0;
                    correction=1;
                    QuadDec_M2_SetCounter(0);
                    break;
                case 1: // left turn
                    forwarding_state=0;
                    leftturn_state=1;
                    rightturn_state=0;
                    break;
                case 2: //right turn
                    forwarding_state=0;
                    leftturn_state=0;
                    rightturn_state=1;
                    break;
                case 3: // u turn
                    forwarding_state=0;
                    leftturn_state=0;
                    rightturn_state=0;
                    u_turn_state=1;
                    break;
                default:
                    forwarding_state=0;
                    leftturn_state=0;
                    rightturn_state=0;
                    //stop();
            }
            if(!u_turn_state && !redirecting ){
                prepTurn();   
            }else{
                u_turn_pre();
            }
            redirecting=0;
        }        
        
        
        // moving straight and adjustment
        //controller 1 is slightly right, 2 is slightly left
        if(forwarding_state && sensor_3){
            changeDutyCycle(); // moving straight
            correction=1;
        }else if ((sensor_2 || sensor_1) && forwarding_state){ // left turn
            controller(2);  }            
        else if ((sensor_4 || sensor_5) && forwarding_state){  // right turn
            controller(1);               
        }else if(forwarding_state) {
            changeDutyCycle();
        }
        
        // count distance at the final move instruction and stop when reach distance
        if(final_way_state && forwarding_state){
            if(temp>dis){ // the distance should be at the middle of the motor
                final_way_state=0;                
                step=1; // need to reset the step.
                path_finding(directions[0]); // find the next path                  
                redirecting=1;    
                correction=1;
                u_turn_pre();
            }
        }
        // state machine(don't touch them)
        if(u_turn_state){
            u_turn();
        }
        
        if(rightturn_state){
            rightTurn();
        }
        
        if(leftturn_state){
            leftTurn();
        }
   
    }
}
//* ========================================
void usbPutString(char *s)
{
// !! Assumes that *s is a string with allocated space >=64 chars     
//  Since USB implementation retricts data packets to 64 chars, this function truncates the
//  length to 62 char (63rd char is a '!')

#ifdef USE_USB     
    while (USBUART_CDCIsReady() == 0);
    s[63]='\0';
    s[62]='!';
    USBUART_PutData((uint8*)s,strlen(s));
#endif
}
//* ========================================
void usbPutChar(char c)
{
#ifdef USE_USB     
    while (USBUART_CDCIsReady() == 0);
    USBUART_PutChar(c);
#endif    
}
//* ========================================
void handle_usb()
{
    // handles input at terminal, echos it back to the terminal
    // turn echo OFF, key emulation: only CR
    // entered string is made available in 'line' and 'flag_KB_string' is set
    
    static uint8 usbStarted = FALSE;
    static uint16 usbBufCount = 0;
    uint8 c; 
    

    if (!usbStarted)
    {
        if (USBUART_GetConfiguration())
        {
            USBUART_CDC_Init();
            usbStarted = TRUE;
        }
    }
    else
    {
        if (USBUART_DataIsReady() != 0)
        {  
            c = USBUART_GetChar();

            if ((c == 13) || (c == 10))
            {
//                if (usbBufCount > 0)
                {
                    entry[usbBufCount]= '\0';
                    strcpy(line,entry);
                    usbBufCount = 0;
                    flag_KB_string = 1;
                }
            }
            else 
            {
                if (((c == CHAR_BACKSP) || (c == CHAR_DEL) ) && (usbBufCount > 0) )
                    usbBufCount--;
                else
                {
                    if (usbBufCount > (BUF_SIZE-2) ) // one less else strtok triggers a crash
                    {
                       USBUART_PutChar('!');        
                    }
                    else
                        entry[usbBufCount++] = c;  
                }  
            }
        }
    }    
}

void distance_cal(void){
    int unit_count=directions[direction_count-2];
    if (directions[direction_count-1]){// 1 for y
        dis=unit_count*97;
    }else{
        dis=unit_count*140;
    }    
}

void path_finding(int pointing){
    start=end[end_count];
    if (end_count == 5){
        stop();
    }
    end_count++;
    a_star(start, end[end_count], pointing, directions, &direction_count);
    redirecting=1;
    step=1; // need to reset the step.
    distance_cal();
}
void changeDutyCycle()
{
    PWM_1_WriteCompare(42200);
    PWM_2_WriteCompare(43000);    
    
}


void controller(uint8_t type){
    //uint16_t val;
    switch(type){
        case 1: // right turn
            PWM_2_WriteCompare(38500);
            PWM_1_WriteCompare(42200);
            break;
        case 2: // slightly left turn
            PWM_1_WriteCompare(36500);
            PWM_2_WriteCompare(43000);  
            break;
    }
}
  
void u_turn_pre(){
    QuadDec_M1_SetCounter(0);
    PWM_1_WriteCompare(32768);
    PWM_2_WriteCompare(32768);
    CyDelay(100);
}
void u_turn(){
    PWM_2_WriteCompare(23600);
    PWM_1_WriteCompare(41000);
    if((temp_left<-185)){ // if the left wheel value is smaller than -175
        forwarding_state=1;
        u_turn_state=0; 
        u_turn_pre();
    }    
}


void stop(){
    PWM_1_WriteCompare(32768);
    PWM_2_WriteCompare(32768);
    CyDelay(10000);
}

void prepTurn(){
    PWM_2_WriteCompare(42200);
    PWM_1_WriteCompare(43000);
    CyDelay(100);
    QuadDec_M1_SetCounter(0);
    QuadDec_M2_SetCounter(0);
    PWM_1_WriteCompare(32768);
    PWM_2_WriteCompare(32768);
    CyDelay(10);  
}

void rightTurn(){
    QuadDec_M2_SetCounter(0);
    PWM_2_WriteCompare(23600);
    PWM_1_WriteCompare(41000);
    if (sensor_5){
        next_state=1;

    }
    if (next_state && sensor_4 ){
        rightturn_state=0;
        forwarding_state=1;
        next_state=0;
        QuadDec_M2_SetCounter(0);
    }
}

void leftTurn(){
    QuadDec_M2_SetCounter(0);
    PWM_1_WriteCompare(25000);
    PWM_2_WriteCompare(43000);
    if (sensor_1){
        next_state=1;        
    }
    if (next_state && sensor_2){
        leftturn_state=0;
        forwarding_state=1;
        next_state=0;
    }
}

/* [] END OF FILE */
