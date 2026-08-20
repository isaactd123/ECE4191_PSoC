#include "project.h"
#include <stdio.h>
#include <string.h>

char commandBuffer[30];
int commandIndex = 0;


/* =========================
   MOTOR FUNCTIONS
   ========================= */

void stopRobot(void)
{
    PWM_1_WriteCompare(0);
    PWM_2_WriteCompare(0);
}


void moveForward(void)
{
    Motor_1_IN_1_Write(1);
    Motor_1_IN_2_Write(0);

    Motor_2_IN_3_Write(1);
    Motor_2_IN_4_Write(0);

    PWM_1_WriteCompare(220);
    PWM_2_WriteCompare(220);
}


void moveBackward(void)
{
    Motor_1_IN_1_Write(0);
    Motor_1_IN_2_Write(1);

    Motor_2_IN_3_Write(0);
    Motor_2_IN_4_Write(1);

    PWM_1_WriteCompare(220);
    PWM_2_WriteCompare(220);
}


void turnRight(void)
{
    
    // Right motor backward
    Motor_1_IN_1_Write(0);
    Motor_1_IN_2_Write(1);

    // Left motor forward
    Motor_2_IN_3_Write(1);
    Motor_2_IN_4_Write(0);
    
    
    PWM_1_WriteCompare(180);
    PWM_2_WriteCompare(180);
}


void turnLeft(void)
{
    // Right motor forward
    Motor_1_IN_1_Write(1);
    Motor_1_IN_2_Write(0);

    // Left motor backward
    Motor_2_IN_3_Write(0);
    Motor_2_IN_4_Write(1);

    PWM_1_WriteCompare(180);
    PWM_2_WriteCompare(180);
}


/* =========================
   COMMAND PROCESSING
   ========================= */

void processCommand(char *command)
{
    int value;

    /* FD 100 */
    if(sscanf(command, "FD %d", &value) == 1)
    {
        UART_1_PutString("Forward\r\n");

        moveForward();

        /*
         value will later be used as distance.
         Example: FD 100 = move 100 cm
        */
    }

    /* BK 50 */
    else if(sscanf(command, "BK %d", &value) == 1)
    {
        UART_1_PutString("Backward\r\n");

        moveBackward();
    }

    /* RT 90 */
    else if(sscanf(command, "RT %d", &value) == 1)
    {
        UART_1_PutString("Turn Right\r\n");

        turnRight();

        /*
         value will later be used as angle.
         Example: RT 90 = rotate 90 degrees
        */
    }

    /* LT 90 */
    else if(sscanf(command, "LT %d", &value) == 1)
    {
        UART_1_PutString("Turn Left\r\n");

        turnLeft();
    }

    /* STOP */
    else if(strcmp(command, "STOP") == 0)
    {
        UART_1_PutString("STOP\r\n");

        stopRobot();
    }

    else
    {
        UART_1_PutString("Unknown Command\r\n");
    }
}


/* =========================
   MAIN
   ========================= */

/* int main(void)
{
    char receivedChar;

    CyGlobalIntEnable;

    PWM_1_Start();
    PWM_2_Start();
    UART_1_Start();

    QuadDec_1_Start();
    QuadDec_2_Start();

    stopRobot();

    UART_1_PutString("Robot Ready\r\n");

    for(;;)
    {
        receivedChar = UART_1_GetChar();

        if(receivedChar != 0)
        {
           
            // ENTER pressed
            // Termite may send \r or \n
            
            if(receivedChar == '\r' || receivedChar == '\n')
            {
                if(commandIndex > 0)
                {
                    commandBuffer[commandIndex] = '\0';

                    processCommand(commandBuffer);

                    commandIndex = 0;
                }
            }
            else
            {
                if(commandIndex < 29)
                {
                    commandBuffer[commandIndex] = receivedChar;
                    commandIndex++;
                }
            }
        }
    }
}*/