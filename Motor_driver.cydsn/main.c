/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdio.h>

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    
    // PWM & QuadDec 1 start
    PWM_1_Start();
    QuadDec_1_Start();
    UART_1_Start();
    
    // PWM & QuadDec 2 start
    PWM_2_Start();
    QuadDec_2_Start();
    
    QuadDec_1_SetCounter(0);
    QuadDec_2_SetCounter(0);
    
    int counter_1 = 0;
    int counter_2 = 0;
    char string_1[50];
    char string_2[50];
    
    Motor_1_IN_1_Write(1);
    Motor_1_IN_2_Write(0);
    PWM_1_WriteCompare(220);    // This change the PWM from default 127 to 220
    
    Motor_2_IN_3_Write(1);
    Motor_2_IN_4_Write(0);
    PWM_2_WriteCompare(220);
    
    // Encoder code (dk why no value showing, could be connection issue) 
    for(;;)
    {
        /* Place your application code here. */
        counter_1 = QuadDec_1_GetCounter();
        sprintf(string_1,"LEFT Motor: %d\n",counter_1);
        UART_1_PutString(string_1);
        //QuadDec_1_SetCounter(0);
        
        counter_2 = QuadDec_2_GetCounter();
        sprintf(string_2,"RIGHT Motor: %d\n",counter_2);
        UART_1_PutString(string_2);
        //QuadDec_2_SetCounter(0);
        
        // QuadDec_1_SetCounter(0);    // This function reset the value after certain period of time
        CyDelay(100);
        
    }
}
/*
This code is to make the robot go for 1 meter
int targetTicks = 2215;   // if your encoder is 390 ticks/rev

while (1)
{
    int left = QuadDec_1_GetCounter();
    int right = QuadDec_2_GetCounter();

    if ((left + right) / 2 >= targetTicks)
    {
        // Stop the robot
        PWM_1_WriteCompare(0);
        PWM_2_WriteCompare(0);
        break;
    }
}

*/

/*
PID code
basePWM = 220;

while(1)
{
    left  = QuadDec_1_GetCounter();
    right = QuadDec_2_GetCounter();

    leftSpeed  = left  - lastLeft;
    rightSpeed = right - lastRight;

    error = leftSpeed - rightSpeed;

    integral += error;
    derivative = error - previousError;

    correction = Kp*error + Ki*integral + Kd*derivative;

    PWM_1_WriteCompare(limit(basePWM - correction));
    PWM_2_WriteCompare(limit(basePWM + correction));

    previousError = error;
    lastLeft = left;
    lastRight = right;

    CyDelay(10);     // 10 ms control loop
}
*/
/* [] END OF FILE */
