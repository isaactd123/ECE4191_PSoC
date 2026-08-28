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
#include "logo_interpreter.h"
#include <string.h>

/*
    Must be >= the master/slave ESP32's MAX_PROGRAM_SIZE (240) + 1 for '\0'.
*/
#define PSOC_RX_BUFFER_SIZE 241

char Rx = '\0';
char string_1[200] = "\0";
int i = 0;
char buffer[PSOC_RX_BUFFER_SIZE];
int buffIndex = 0;

CY_ISR(ISR_Handler_1)
{
    Rx = UART_1_GetChar();
    if (Rx != '\0')
    {
        string_1[i] = Rx;
        i++;
    }
    else
    {
        string_1[i] = '\0';
        strcat(string_1,"_PSoC5LP");
        UART_2_PutString(string_1);
        strcpy(string_1,"\0");
        i = 0;
    }


}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_1_Start();
    UART_2_Start();
    isr_1_StartEx(ISR_Handler_1);

    /* -----------------------------------------------------------------
       BENCH TEST: exercise the LOGO interpreter with no ESP32 master/
       slave connected at all. Runs once on boot and prints everything
       to Termite (UART_1). Comment this whole block out (or set
       LOGO_BENCH_TEST to 0) once you're ready to test with real
       ESP-NOW traffic instead.
       ----------------------------------------------------------------- */
    #define LOGO_BENCH_TEST 1
    #if LOGO_BENCH_TEST
    UART_1_PutString("\r\n[TEST] Running hardcoded LOGO program (no ESP32 needed)...\r\n");
    runLogoProgram(
        "make \"vowel \"blank\n"
        "make \"checkpoint \"a\n"
        "while :vowel = \"blank [fd 10 lt 135 fd 3 rt 135]\n"
        "ifelse :vowel = :checkpoint [repeat 2 [fd 5]][bk 4]"
    );
    #endif

    for(;;)
    {
        /* Place your application code here. */
        if(UART_2_GetRxBufferSize() > 0)
        {
            char Rx;

            Rx = UART_2_GetChar();

            if(Rx != '\0')
            {
                if (buffIndex < (int)sizeof(buffer) - 1)
                {
                    buffer[buffIndex] = Rx;
                    buffIndex++;
                }
                else
                {
                    UART_1_PutString("[PSoC ERROR] RX buffer overflow, discarding extra bytes\r\n");
                }
            }
            else
            {
                buffer[buffIndex] = '\0';
                if(buffIndex > 0)
                {
                    UART_1_PutString("Received: ");
                    UART_1_PutString(buffer);
                    UART_1_PutString("\r\n");
                    /*decode and execute the LOGO program */
                    runLogoProgram(buffer);
                }

                buffIndex = 0;
            }
        }
    }
}
/* [] END OF FILE */