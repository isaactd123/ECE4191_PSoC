#include "PsocLink.h"
#include "TurtleConfig.h"

#include <string.h>

namespace {

char psocCommandBuffer[32] = {};
size_t psocCommandLength = 0;
bool classificationRequested = false;

}  // namespace

namespace PsocLink {

void begin() {
  Serial1.begin(PSOC_BAUD_RATE, SERIAL_8N1, PSOC_RX_PIN, PSOC_TX_PIN);

  Serial.printf("PSoC UART: RX=GPIO%d TX=GPIO%d @ %u baud\n",
                PSOC_RX_PIN, PSOC_TX_PIN,
                static_cast<unsigned>(PSOC_BAUD_RATE));

  /*
      Announce ourselves on the wire. If DEBUG_ECHO_RX_BYTES is on in
      the PSoC's main.c, this line should appear in Termite a couple
      of seconds after the ESP32 resets. Seeing it proves the pins,
      the baud rate and the ESP-to-PSoC direction all work, before
      any program is ever sent.
  */
  Serial1.print("HELLO_FROM_ESP32\n");
  Serial1.flush();
}

void sendProgram(const uint8_t* program, size_t length) {
  if (program == nullptr || length == 0) return;

  if (DEBUG_PRINT_RECEIVED_PROGRAM) {
    Serial.println();
    Serial.println("========== PROGRAM FROM MASTER ==========");
    Serial.write(program, length);
    Serial.println();
    Serial.println("-----------------------------------------");

    // Byte-level view, so invisible characters are still obvious.
    for (size_t i = 0; i < length; ++i) {
      const uint8_t b = program[i];
      if (b == '\n')      Serial.print("\\n ");
      else if (b == '\r') Serial.print("\\r ");
      else if (b < 32 || b > 126) Serial.printf("<%02X> ", b);
      else                Serial.printf("%c ", static_cast<char>(b));
    }

    Serial.println();
    Serial.printf("Total: %u bytes, plus one NUL terminator.\n",
                  static_cast<unsigned>(length));
    Serial.println("=========================================");
  }

  // The PSoC receiver expects the program followed by one null byte.
  Serial1.write(program, length);
  Serial1.write('\0');
  Serial1.flush();

  Serial.printf("PROGRAM_FORWARDED_TO_PSOC:%u\n",
                static_cast<unsigned>(length));
}

void sendVowel(char normalizedVowel) {
  if (normalizedVowel == '_') {
    Serial1.print("VOWEL:blank\n");
  } else {
    Serial1.printf("VOWEL:%c\n", normalizedVowel);
  }
}

void service() {
  while (Serial1.available() > 0) {
    const char incoming = static_cast<char>(Serial1.read());

    if (incoming == '\n' || incoming == '\0') {
      psocCommandBuffer[psocCommandLength] = '\0';

      if (strcmp(psocCommandBuffer, "CLASSIFY") == 0) {
        classificationRequested = true;
      } else if (psocCommandLength > 0) {
        // Anything else the PSoC says is surfaced for debugging.
        Serial.printf("[PSOC] %s\n", psocCommandBuffer);
      }

      psocCommandLength = 0;
    } else if (incoming != '\r') {
      if (psocCommandLength < sizeof(psocCommandBuffer) - 1)
        psocCommandBuffer[psocCommandLength++] = incoming;
      else
        psocCommandLength = 0;
    }
  }
}

bool takeClassificationRequest() {
  const bool requested = classificationRequested;
  classificationRequested = false;
  return requested;
}

}  // namespace PsocLink