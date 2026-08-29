/* ========================================
 *
 * WriteCodeBot PSoC 5LP main
 *
 *   UART_1  ->  Termite on the PC   (debug only)
 *   UART_2  <-> ESP32-S3 turtle     (programs in, VOWEL replies in,
 *                                    CLASSIFY requests out)
 *
 * ========================================
*/
#include "project.h"
#include "logo_interpreter.h"
#include <string.h>
#include <stdio.h>

/*
    Must be >= the ESP32's MAX_PAYLOAD (240) plus one byte for '\0'.
*/
#define PSOC_RX_BUFFER_SIZE 241

/*
    Set to 1 to print every byte arriving on UART_2 as it is received.
    Use this to prove the wiring and baud rate are correct before
    worrying about anything else.
*/
#define DEBUG_ECHO_RX_BYTES 1

static char buffer[PSOC_RX_BUFFER_SIZE];
static int buffIndex = 0;

/* ================================================================
   REMOVED: CY_ISR(ISR_Handler_1)

   The old interrupt handler read UART_1, appended "_PSoC5LP" and
   echoed the result to UART_2 -- straight back at the ESP32. It was
   left over from an early loopback test.

   Your TopDesign wires isr_1 to UART_1's rx_interrupt, so it was
   reading the right UART and did NOT block reception. It is removed
   because anything typed into Termite was being transmitted to the
   ESP32, where PsocLink::service() then tried to parse it.

   In PSoC Creator, also delete the isr_1 component from TopDesign,
   or at minimum leave isr_1_StartEx() commented out as it is here.

   Note on isr_2: your schematic connects it to UART_2's
   rx_interrupt, but nothing ever calls isr_2_StartEx(). If UART_2's
   RX buffer size is set above 4, the component generates its own
   internal ISR on that same signal, and an unstarted external isr_2
   attached to it is a conflict. Either delete isr_2 and let the
   component manage its own buffer, or start it and write a handler.
   Not both.
   ================================================================ */

static void handleCompletedProgram(void)
{
    buffer[buffIndex] = '\0';

    if (buffIndex == 0)
    {
        return;
    }

    UART_1_PutString("\r\n[PSoC] Program received, ");
    {
        char lengthText[12];
        sprintf(lengthText, "%d", buffIndex);
        UART_1_PutString(lengthText);
    }
    UART_1_PutString(" bytes:\r\n");
    UART_1_PutString(buffer);
    UART_1_PutString("\r\n");

    runLogoProgram(buffer);

    buffIndex = 0;
}

int main(void)
{
    CyGlobalIntEnable;

    UART_1_Start();
    UART_2_Start();

    /* isr_1_StartEx(ISR_Handler_1);  <-- deliberately not started */

    UART_1_PutString("\r\n========================================\r\n");
    UART_1_PutString("  PSoC 5LP ready. Waiting for a program.\r\n");
    UART_1_PutString("========================================\r\n");

    for (;;)
    {
        if (UART_2_GetRxBufferSize() > 0u)
        {
            char Rx = (char)UART_2_GetChar();

#if DEBUG_ECHO_RX_BYTES
            if (Rx == '\0')
            {
                UART_1_PutString("<NUL>");
            }
            else
            {
                UART_1_PutChar(Rx);
            }
#endif

            if (Rx != '\0')
            {
                if (buffIndex < (int)sizeof(buffer) - 1)
                {
                    buffer[buffIndex] = Rx;
                    buffIndex++;
                }
                else
                {
                    UART_1_PutString(
                        "\r\n[PSoC ERROR] RX overflow, discarding\r\n");
                }
            }
            else
            {
                handleCompletedProgram();
            }
        }
    }
}

/* [] END OF FILE */