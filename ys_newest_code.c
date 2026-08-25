/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * ========================================
 */

#include "project.h"
#include <stdio.h>
#include <stdint.h>

/* ---------- General configuration ---------- */

#define PWM_MAX             255
#define BASE_PWM            180
#define CONTROL_MS           50

/* ---------- Straight-line PID ---------- */

float Kp = 0.50f;
float Ki = 0.0002f;
float Kd = 0.00f;

/* ---------- Robot measurements ---------- */

/*
 * Replace these with your actual measurements.
 *
 * COUNTS_PER_REV:
 * Average encoder count after exactly one wheel revolution.
 *
 * WHEEL_DISTANCE_MM:
 * Centre-to-centre distance between the wheels.
 */
#define COUNTS_PER_REV       14645.17f
#define WHEEL_DIAMETER_MM      56.0f
#define WHEEL_DISTANCE_MM     215.0f

/*
 * Adjust this after testing the actual turning angle.
 */
#define TURN_CALIBRATION        .9f

/* ---------- Point-turn controller ---------- */

#define TURN_MAX_PWM          255
#define TURN_MIN_PWM           180
#define TURN_POSITION_KP     0.50f
#define TURN_SYNC_KP         0.15f
#define TURN_CONTROL_MS        10

char message[120];

/* =========================================================
 * Utility functions
 * ========================================================= */

int32_t absoluteValue(int32_t value)
{
    if (value < 0)
        return -value;

    return value;
}

int limitPWM(float value)
{
    if (value > PWM_MAX)
        return PWM_MAX;

    if (value < 0)
        return 0;

    return (int)value;
}

int limitTurnPWM(float value)
{
    if (value > TURN_MAX_PWM)
        return TURN_MAX_PWM;

    if (value < TURN_MIN_PWM)
        return TURN_MIN_PWM;

    return (int)value;
}

/* =========================================================
 * Motor direction functions
 * ========================================================= */

void rightMotorForward(void)
{
    Motor_1_IN_1_Write(1);
    Motor_1_IN_2_Write(0);
}

void rightMotorBackward(void)
{
    Motor_1_IN_1_Write(0);
    Motor_1_IN_2_Write(1);
}

void leftMotorForward(void)
{
    /*
     * Your left motor is physically reversed relative
     * to the right motor.
     */
    Motor_2_IN_3_Write(1);
    Motor_2_IN_4_Write(0);
}

void leftMotorBackward(void)
{
    Motor_2_IN_3_Write(0);
    Motor_2_IN_4_Write(1);
}

void stopMotors(void)
{
    PWM_1_WriteCompare(0);
    PWM_2_WriteCompare(0);
}

/* =========================================================
 * Straight-line PID movement
 * ========================================================= */

/*
 * Moves forward until the average encoder count reaches
 * targetTicks.
 *
 * Example:
 * driveStraightTicks(5000);
 */
void driveStraightTicks(int32_t targetTicks)
{
    int32_t previousRight = 0;
    int32_t previousLeft = 0;
    int32_t previousError = 0;

    int32_t currentRight;
    int32_t currentLeft;

    int32_t rightTicks;
    int32_t leftTicks;

    int32_t totalRight;
    int32_t totalLeft;
    int32_t averageTicks;

    int32_t error;
    int32_t derivative;

    float integral = 0.0f;
    float correction;

    int rightPWM;
    int leftPWM;
    int printCounter = 0;

    rightMotorForward();
    leftMotorForward();

    QuadDec_1_SetCounter(0);
    QuadDec_2_SetCounter(0);

    PWM_1_WriteCompare(BASE_PWM);
    PWM_2_WriteCompare(BASE_PWM);

    for (;;)
    {
        CyDelay(CONTROL_MS);

        currentRight = QuadDec_1_GetCounter();
        currentLeft = QuadDec_2_GetCounter();

        /*
         * Encoder movement during the previous control period.
         */
        rightTicks = currentRight - previousRight;
        leftTicks = currentLeft - previousLeft;

        previousRight = currentRight;
        previousLeft = currentLeft;

        rightTicks = absoluteValue(rightTicks);
        leftTicks = absoluteValue(leftTicks);

        /*
         * Total distance travelled.
         */
        totalRight = absoluteValue(currentRight);
        totalLeft = absoluteValue(currentLeft);
        averageTicks = (totalRight + totalLeft) / 2;

        if (averageTicks >= targetTicks)
            break;

        /*
         * Positive error means the left wheel is faster.
         */
        error = leftTicks - rightTicks;

        integral += error;

        /* Integral anti-windup */
        if (integral > 1000.0f)
            integral = 1000.0f;

        if (integral < -1000.0f)
            integral = -1000.0f;

        derivative = error - previousError;
        previousError = error;

        correction =
            (Kp * error) +
            (Ki * integral) +
            (Kd * derivative);

        /*
         * Left wheel faster:
         * increase right PWM and decrease left PWM.
         */
        rightPWM = limitPWM(BASE_PWM + correction);
        leftPWM = limitPWM(BASE_PWM - correction);

        PWM_1_WriteCompare(rightPWM);
        PWM_2_WriteCompare(leftPWM);

        printCounter++;

        if (printCounter >= 10)
        {
            sprintf(
                message,
                "STRAIGHT Target:%ld R:%ld L:%ld "
                "ERR:%ld RPWM:%d LPWM:%d\r\n",
                (long)targetTicks,
                (long)totalRight,
                (long)totalLeft,
                (long)error,
                rightPWM,
                leftPWM
            );

            UART_1_PutString(message);
            printCounter = 0;
        }
    }

    stopMotors();
    CyDelay(300);
}

/* =========================================================
 * Point-turn movement
 * ========================================================= */

/*
 * Positive angle = left/anticlockwise turn
 * Negative angle = right/clockwise turn
 *
 * Only multiples of 45 degrees are accepted.
 *
 * Examples:
 * pointTurn(45);
 * pointTurn(90);
 * pointTurn(-45);
 * pointTurn(-180);
 */
uint8_t pointTurn(int angleDegrees)
{
    int32_t targetTicks;
    int32_t rightTicks;
    int32_t leftTicks;
    int32_t averageTicks;
    int32_t remainingTicks;
    int32_t syncError;

    float baseTurnPWM;
    float correction;

    int rightPWM;
    int leftPWM;
    int printCounter = 0;

    if ((angleDegrees == 0) || ((angleDegrees % 45) != 0))
    {
        UART_1_PutString(
            "Error: turn angle must be a non-zero multiple of 45\r\n"
        );

        return 0;
    }

    /*
     * Distance travelled by each wheel during a point turn:
     *
     * targetTicks =
     * counts/revolution
     * x wheel-distance/wheel-diameter
     * x angle/360
     */
    targetTicks = (int32_t)(
        COUNTS_PER_REV *
        (WHEEL_DISTANCE_MM / WHEEL_DIAMETER_MM) *
        ((float)absoluteValue(angleDegrees) / 360.0f) *
        TURN_CALIBRATION
    );

    stopMotors();

    QuadDec_1_SetCounter(0);
    QuadDec_2_SetCounter(0);

    /*
     * Based on your motor direction wiring:
     *
     * Left turn:
     * right wheel forward and left wheel backward.
     *
     * Right turn:
     * right wheel backward and left wheel forward.
     */
    if (angleDegrees > 0)
    {
        rightMotorForward();
        leftMotorBackward();
    }
    else
    {
        rightMotorBackward();
        leftMotorForward();
    }

    CyDelay(100);

    for (;;)
    {
        rightTicks =
            absoluteValue(QuadDec_1_GetCounter());

        leftTicks =
            absoluteValue(QuadDec_2_GetCounter());

        averageTicks = (rightTicks + leftTicks) / 2;
        remainingTicks = targetTicks - averageTicks;

        if (remainingTicks <= 0)
            break;

        /*
         * Reduce turning speed near the target.
         */
        baseTurnPWM =
            TURN_POSITION_KP * remainingTicks;

        if (baseTurnPWM > TURN_MAX_PWM)
            baseTurnPWM = TURN_MAX_PWM;

        if (baseTurnPWM < TURN_MIN_PWM)
            baseTurnPWM = TURN_MIN_PWM;

        /*
         * Synchronise the distance travelled by both wheels.
         *
         * Positive error:
         * right wheel has travelled farther.
         */
        syncError = rightTicks - leftTicks;

        correction = TURN_SYNC_KP * syncError;

        rightPWM =
            limitTurnPWM(baseTurnPWM - correction);

        leftPWM =
            limitTurnPWM(baseTurnPWM + correction);

        PWM_1_WriteCompare(rightPWM);
        PWM_2_WriteCompare(leftPWM);

        printCounter++;

        if (printCounter >= 10)
        {
            sprintf(
                message,
                "TURN:%d Target:%ld R:%ld L:%ld "
                "RPWM:%d LPWM:%d\r\n",
                angleDegrees,
                (long)targetTicks,
                (long)rightTicks,
                (long)leftTicks,
                rightPWM,
                leftPWM
            );

            UART_1_PutString(message);
            printCounter = 0;
        }

        CyDelay(TURN_CONTROL_MS);
    }

    stopMotors();

    sprintf(
        message,
        "Completed turn: %d degrees, target:%ld ticks\r\n",
        angleDegrees,
        (long)targetTicks
    );

    UART_1_PutString(message);

    CyDelay(300);

    return 1;
}

/* =========================================================
 * Main program
 * ========================================================= */

int main(void)
{
    CyGlobalIntEnable;

    PWM_1_Start();
    PWM_2_Start();

    QuadDec_1_Start();
    QuadDec_2_Start();

    UART_1_Start();

    stopMotors();

    /*
     * Example movement sequence.
     *
     * Change or remove these commands depending on
     * what you want the robot to do.
     */

   


    for (;;)
    {
        /*
         * Robot waits here after completing the sequence.
         */
         pointTurn(45);
    }
}

/* [] END OF FILE */