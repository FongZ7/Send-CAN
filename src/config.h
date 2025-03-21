// #pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <eXoCAN.h>

#define LED_PIN PC13

void convertToHex(uint8_t hexValue[], int values[], int size);