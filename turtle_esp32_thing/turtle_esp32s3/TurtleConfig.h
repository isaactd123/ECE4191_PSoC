/*
    TurtleConfig.h

    Every tunable constant for the turtle ESP32-S3, shared by the
    vision, radio and PSoC modules.
*/

#pragma once

#include <Arduino.h>

/* ---------------- Camera geometry ---------------- */

constexpr int RAW_WIDTH = 320;
constexpr int RAW_HEIGHT = 240;
constexpr size_t RAW_PIXELS = RAW_WIDTH * RAW_HEIGHT;

constexpr int CROP_Y_START = 20;
constexpr int CROP_Y_END = 235;
constexpr int CROP_X_START = 55;
constexpr int CROP_X_END = 275;

constexpr int CROP_WIDTH = CROP_X_END - CROP_X_START;   // 220
constexpr int CROP_HEIGHT = CROP_Y_END - CROP_Y_START;  // 215
constexpr size_t CROP_PIXELS = CROP_WIDTH * CROP_HEIGHT;

constexpr int CANVAS_SIZE = 220;
constexpr int CANVAS_MARGIN = 15;
constexpr size_t CANVAS_PIXELS = CANVAS_SIZE * CANVAS_SIZE;

constexpr int MIN_COMPONENT_AREA = 8;

/* ---------------- Behaviour ---------------- */

constexpr unsigned long CAPTURE_INTERVAL_MS = 5000;

/*
    Keep true while bench-testing the classifier on its own.

    Set FALSE for integration with the PSoC. When true the turtle
    classifies every five seconds and radios a vowel to the interface
    each time, which floods the master and triggers unwanted audio.
*/
constexpr bool ENABLE_AUTOMATIC_CLASSIFICATION = false;

/*
    Streams the raw 76800-byte frame to the PC over USB serial.

    Set FALSE unless the companion Python receiver is actually
    running. With no host draining the USB CDC buffer this transfer
    stalls the main loop, which delays forwarding programs to the
    PSoC.
*/
constexpr bool ENABLE_FRAME_STREAMING = false;

/* Give up on a stalled USB transfer rather than blocking forever. */
constexpr uint32_t FRAME_STREAM_TIMEOUT_MS = 2000;

/*
    Prints the full text of every program received from the master
    before forwarding it to the PSoC. Useful for confirming that what
    left the PC is exactly what arrived here.
*/
constexpr bool DEBUG_PRINT_RECEIVED_PROGRAM = true;

/* ---------------- Radio and PSoC UART ---------------- */

/* Must match ESPNOW_CHANNEL in interface_esp32.ino. */
constexpr uint8_t ESPNOW_CHANNEL = 6;

constexpr uint32_t DEBUG_BAUD_RATE = 921600;
constexpr uint32_t PSOC_BAUD_RATE = 9600;

constexpr uint32_t RADIO_ACK_TIMEOUT_MS = 350;
constexpr uint8_t MAX_VOWEL_SEND_ATTEMPTS = 4;

/*
    PSOC_RX_PIN receives, so it wires to the PSoC Rx_2's partner --
    the PSoC UART_2 TX pin.
    PSOC_TX_PIN transmits, so it wires to the PSoC UART_2 RX pin.

    DO NOT USE D6 AND D7 HERE.

    On the XIAO ESP32S3, D6 is GPIO43 (U0TXD) and D7 is GPIO44
    (U0RXD) -- the ESP32-S3's default UART0 pins. Pointing Serial1
    at them puts UART1 and UART0 on the same pads, so Serial1's
    output can be lost entirely. Worse, the ROM bootloader prints
    its boot log on GPIO43 at 115200 baud on every reset, and that
    garbage ends up prepended to the next program in the PSoC's
    receive buffer.

    D0 (GPIO1) and D1 (GPIO2) are plain GPIOs on this board: no
    strapping function, and not used by the camera, microphone or
    SD card.
*/
constexpr int PSOC_RX_PIN = D0;   // GPIO1  <- PSoC UART_2 Tx_2
constexpr int PSOC_TX_PIN = D1;   // GPIO2  -> PSoC UART_2 Rx_2