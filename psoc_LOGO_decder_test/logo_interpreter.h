#ifndef LOGO_INTERPRETER_H
#define LOGO_INTERPRETER_H

/*
    Call this with a null-terminated LOGO program string -- exactly
    what arrives in buffer[] from your ESP32 slave over UART_2.

    It will:
      1. Tokenize the string
      2. Walk through it statement by statement
      3. Execute make / while / ifelse / repeat / fd / bk / lt / rt
      4. Drive the turtle's motors as it goes (see motorForward(),
         motorBackward(), motorTurnLeft(), motorTurnRight() in
         logo_interpreter.c -- these are the functions you'll wire
         up to your real motor driver)

    Any problems (bad syntax, unknown command, unknown variable) are
    printed to UART_1 (your Termite terminal) and execution stops.
*/
void runLogoProgram(const char *program);

#endif
