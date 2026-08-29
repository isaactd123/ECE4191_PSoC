/*
    PsocLink.h

    UART bridge to the PSoC. Forwards LOGO programs down and listens
    for the CLASSIFY command coming back up.
*/

#pragma once

#include <Arduino.h>

namespace PsocLink {

void begin();

/* Must be called every loop() iteration. */
void service();

void sendProgram(const uint8_t* program, size_t length);

/* Reports a classification to the PSoC as "VOWEL:a" or "VOWEL:blank". */
void sendVowel(char normalizedVowel);

/*
    True once if the PSoC has asked for a classification since the
    last call. Reading it clears the request.
*/
bool takeClassificationRequest();

}  // namespace PsocLink
