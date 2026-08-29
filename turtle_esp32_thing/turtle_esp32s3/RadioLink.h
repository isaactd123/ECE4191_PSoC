/*
    RadioLink.h

    ESP-NOW transport for the turtle. Receives LOGO programs from the
    interface ESP32 and sends vowel classifications back.
*/

#pragma once

#include <Arduino.h>
#include "WriteCodeBotProtocol.h"

namespace RadioLink {

/* Returns false if the radio could not be started. */
bool begin();

bool isReady();

/* Must be called every loop() iteration. */
void service();

/*
    Copies a queued program into 'destination' and returns its length,
    or 0 when no program is waiting.
*/
size_t takeQueuedProgram(uint8_t* destination, size_t capacity);

/* Sends a classification, retrying until the interface acknowledges. */
void sendVowel(char normalizedVowel);

}  // namespace RadioLink
