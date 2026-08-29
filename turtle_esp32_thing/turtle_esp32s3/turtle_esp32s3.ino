/*
  XIAO ESP32S3 Sense — WriteCodeBot turtle

    interface ESP32 <-- ESP-NOW --> this board <-- UART --> PSoC
                                         |
                                         +-- OV2640 camera

  Programs travel down to the PSoC, vowel classifications travel back
  up to the interface. The classifier itself lives in
  VowelClassifier.cpp and is unchanged from the original sketch.

  Board settings:
    - Board: XIAO_ESP32S3
    - Tools > PSRAM > OPI PSRAM   (required)
*/

#include "TurtleConfig.h"
#include "RadioLink.h"
#include "PsocLink.h"
#include "VowelClassifier.h"

static unsigned long lastCapture = 0;

static void publishVowel(char vowel) {
  const char lowered =
      static_cast<char>(tolower(static_cast<unsigned char>(vowel)));
  const bool isBlank = (lowered == '_' || lowered == ' ');
  const char normalized = isBlank ? '_' : lowered;

  PsocLink::sendVowel(normalized);

  if (isBlank) {
    Serial.println("EVENT:VOWEL:blank");
  } else {
    Serial.printf("EVENT:VOWEL:%c\n", normalized);
  }

  // The radio protocol uses '_' as the one-byte representation of blank.
  RadioLink::sendVowel(normalized);
}

static void forwardAnyQueuedProgram() {
  uint8_t program[WriteCodeBot::MAX_PAYLOAD];

  const size_t length =
      RadioLink::takeQueuedProgram(program, sizeof(program));

  if (length > 0) {
    PsocLink::sendProgram(program, length);
  }
}

void setup() {
  Serial.begin(DEBUG_BAUD_RATE);
  PsocLink::begin();
  delay(2000);

  Serial.println();
  Serial.println("XIAO ESP32S3 WriteCodeBot turtle");

  /*
      FIX 2: the radio is started FIRST.

      In the original sketch initialiseEspNow() was the last statement
      in setup(), sitting behind three early returns -- missing PSRAM,
      failed buffer allocation, and failed camera init. Any one of
      those (a loose camera ribbon is the usual culprit) meant setup()
      returned before the radio existed, so the turtle silently
      received nothing from the master for the rest of the session.

      Program delivery must not depend on the camera working.
  */
  if (!RadioLink::begin()) {
    Serial.println("Radio failed to start. Restarting in five seconds...");
    delay(5000);
    ESP.restart();
  }

  if (!VowelClassifier::begin()) {
    Serial.println("Classifier unavailable. Continuing without the camera.");
    Serial.println("Programs will still be forwarded to the PSoC.");
  }

  Serial.printf("Automatic classification: %s\n",
                ENABLE_AUTOMATIC_CLASSIFICATION ? "ENABLED" : "DISABLED");
  Serial.printf("Frame streaming to PC: %s\n",
                ENABLE_FRAME_STREAMING ? "ENABLED" : "DISABLED");
  Serial.println("Send CLASSIFY and a newline from the PSoC to classify.");
  Serial.println("System ready");
}

void loop() {
  forwardAnyQueuedProgram();
  PsocLink::service();
  RadioLink::service();

  if (!VowelClassifier::isReady()) {
    delay(5);
    return;
  }

  const unsigned long now = millis();
  const bool automaticCaptureDue =
      ENABLE_AUTOMATIC_CLASSIFICATION && now - lastCapture >= CAPTURE_INTERVAL_MS;
  const bool requested = PsocLink::takeClassificationRequest();

  if (!requested && !automaticCaptureDue) {
    delay(1);
    return;
  }

  static CandidateResult result;
  const bool success = VowelClassifier::captureAndClassify(result);

  publishVowel(success ? (result.blank ? '_' : result.vowel) : '_');

  // Start the next interval only after classification has finished.
  lastCapture = millis();
}
