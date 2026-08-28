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

#define PWM_MAX             200
#define CONTROL_MS           50

/* ---------- Straight-line master-slave PID ---------- */

/*
 * Right motor = master: PWM_1 remains fixed at MASTER_PWM.
 * Left motor  = slave:  PWM_2 is corrected to follow the right encoder.
 *
 * Begin tuning with Ki and Kd equal to zero. This matches the
 * proportional-only method shown in the lecture slide.
 */
#define MASTER_PWM           200

float Kp = 0.25f;
float Ki = 0.001f;
float Kd = 0.0f;

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
#define TURN_CALIBRATION        1.198f

/* ---------- Point-turn controller ---------- */

#define TURN_MAX_PWM          200
#define TURN_MIN_PWM          150
#define TURN_POSITION_KP     1.1f

/*
 * Master-slave sync gains for the point turn, same role as
 * Kp / Ki / Kd used in driveStraightTicks(). Right wheel is
 * master (position-controlled only), left wheel is slave
 * (position PWM + this PID correction).
 *
 * Start with Ki and Kd at zero, same as the straight-line
 * tuning approach, then add them back in once TURN_SYNC_KP
 * is dialled in.
 */
#define TURN_SYNC_KP          0.30f
#define TURN_SYNC_KI          0.003f
#define TURN_SYNC_KD          0.0001f

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
 * Moves forward using right-master/left-slave control until
 * the master (right) encoder reaches targetTicks.
 *
 * Master PWM never changes. Only the slave PWM is corrected.
 */
void driveStraightTicks(int32_t targetTicks)
{
    int32_t previousError = 0;

    int32_t currentRight;
    int32_t currentLeft;

    int32_t totalRight;
    int32_t totalLeft;

    int32_t error;
    int32_t derivative;

    float integral = 0.0f;
    float correction;

    int slavePWM;
    int printCounter = 0;

    rightMotorForward();
    leftMotorForward();

    QuadDec_1_SetCounter(0);
    QuadDec_2_SetCounter(0);

    /* Both wheels start at the same PWM. */
    PWM_1_WriteCompare(MASTER_PWM);  /* Right master: fixed */
    PWM_2_WriteCompare(MASTER_PWM);  /* Left slave: adjusted below */

    for (;;)
    {
        CyDelay(CONTROL_MS);

        currentRight = QuadDec_1_GetCounter();
        currentLeft = QuadDec_2_GetCounter();

        /*
         * Total distance travelled.
         */
        totalRight = absoluteValue(currentRight);
        totalLeft = absoluteValue(currentLeft);
        /*
         * Stop based on the master wheel. This prevents the faster
         * slave wheel from ending the movement prematurely.
         */
        if (totalRight >= targetTicks)
            break;

        /*
         * Master-slave error from the lecture method:
         *
         * error = Count_Slave - Count_Master
         *
         * Positive error: slave is ahead/faster, so reduce slave PWM.
         * Negative error: slave is behind/slower, so increase slave PWM.
         */
        error = totalLeft - totalRight;

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
         * PWM_Slave = PWM_Master - PID correction
         * PWM_Master remains unchanged.
         */
        slavePWM = limitPWM(MASTER_PWM - correction);

        PWM_1_WriteCompare(MASTER_PWM);
        PWM_2_WriteCompare(slavePWM);

        printCounter++;

        if (printCounter >= 10)
        {
            sprintf(
                message,
                "STRAIGHT Target:%ld R:%ld L:%ld "
                "ERR:%ld MASTER_PWM:%d SLAVE_PWM:%d\r\n",
                (long)targetTicks,
                (long)totalRight,
                (long)totalLeft,
                (long)error,
                MASTER_PWM,
                slavePWM
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
 *
 * Uses the same right-master/left-slave PID approach as
 * driveStraightTicks():
 *   - Right wheel PWM comes only from the position controller
 *     (TURN_POSITION_KP against remaining ticks) and is never
 *     touched by the sync correction.
 *   - Left wheel PWM = right wheel PWM - PID(leftTicks - rightTicks),
 *     using TURN_SYNC_KP / KI / KD.
 *   - The loop stops based on the master (right) encoder only,
 *     so a faster slave wheel can't end the turn early.
 */
uint8_t pointTurn(int angleDegrees)
{
    int32_t targetTicks;
    int32_t rightTicks;   /* master */
    int32_t leftTicks;    /* slave */
    int32_t remainingTicks;

    int32_t error;
    int32_t derivative;
    int32_t previousError = 0;

    float integral = 0.0f;
    float baseTurnPWM;
    float correction;

    int masterPWM;
    int slavePWM;
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

        /*
         * Stop based on the master (right) wheel only, same
         * reasoning as driveStraightTicks().
         */
        if (rightTicks >= targetTicks)
            break;

        remainingTicks = targetTicks - rightTicks;

        /*
         * Position control for the master wheel: slow down
         * as it approaches the target. This PWM is never
         * modified by the sync correction below.
         */
        baseTurnPWM =
            TURN_POSITION_KP * remainingTicks;

        if (baseTurnPWM > TURN_MAX_PWM)
            baseTurnPWM = TURN_MAX_PWM;

        if (baseTurnPWM < TURN_MIN_PWM)
            baseTurnPWM = TURN_MIN_PWM;

        masterPWM = limitTurnPWM(baseTurnPWM);

        /*
         * Master-slave error, same convention as
         * driveStraightTicks():
         *
         * error = Count_Slave - Count_Master
         *
         * Positive error: slave (left) is ahead/faster, so reduce
         * slave PWM. Negative error: slave is behind/slower, so
         * increase slave PWM.
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
            (TURN_SYNC_KP * error) +
            (TURN_SYNC_KI * integral) +
            (TURN_SYNC_KD * derivative);

        /*
         * PWM_Slave = PWM_Master - PID correction
         * PWM_Master remains unchanged.
         */
        slavePWM = limitTurnPWM(masterPWM - correction);

        PWM_1_WriteCompare(masterPWM);
        PWM_2_WriteCompare(slavePWM);

        printCounter++;

        if (printCounter >= 10)
        {
            sprintf(
                message,
                "TURN:%d Target:%ld R:%ld L:%ld "
                "ERR:%ld MASTER_PWM:%d SLAVE_PWM:%d\r\n",
                angleDegrees,
                (long)targetTicks,
                (long)rightTicks,
                (long)leftTicks,
                (long)error,
                masterPWM,
                slavePWM
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

    /*
     * Master-slave straight-line test.
     * Replace 5000 with the required master encoder target.
     */
    

    /* Optional point-turn examples:
     * pointTurn(45);
     * pointTurn(-90);
     */

    /* Perform one 45-degree turn for tuning. */
    

    /* Remain stopped after the test instead of repeating the turn. */
    for (;;)
    {
        driveStraightTicks(100000);
        //pointTurn(45);
        stopMotors();
    }
}

/* [] END OF FILE */
