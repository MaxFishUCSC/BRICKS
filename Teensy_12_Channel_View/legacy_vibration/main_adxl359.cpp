// ============================================================================
// LEGACY: ADXL359 vibration measurement sketch (previous project in this
// folder, archived 2026-08).  The Teensy_12_Channel_View project now contains
// the 12-channel logic analyzer firmware instead.  This file is kept for
// reference only - it is NOT part of the current build.
// ============================================================================
// (Original content of src/main.cpp before it was replaced by the
// 12-channel logic analyzer firmware.  See live_plot.py in this folder for
// the matching host-side plotting script.)

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#define CS_PIN      35
#define DRDY_PIN     8

#define DEVID_REG   0x00
#define XDATA_H     0x08
#define RANGE_REG   0x2C
#define POWER_CTL   0x2D
#define SENSITIVITY 51200.0f
#define N_SAMPLES   512

SPISettings adxlSettings(5000000, MSBFIRST, SPI_MODE0);

volatile bool dataReady = false;

void drdyISR() { dataReady = true; }

uint8_t readReg(uint8_t reg) {
    SPI.beginTransaction(adxlSettings);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer((reg << 1) | 0x01);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    return value;
}

void writeReg(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(adxlSettings);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer((reg << 1) & 0xFE);
    SPI.transfer(value);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

int32_t convert20bit(uint8_t b0, uint8_t b1, uint8_t b2) {
    int32_t value = ((int32_t)b0 << 12) |
                    ((int32_t)b1 << 4)  |
                    ((int32_t)b2 >> 4);
    if (value & 0x80000) value |= 0xFFF00000;
    return value;
}

void readAcceleration(int32_t &x, int32_t &y, int32_t &z) {
    uint8_t buffer[9];
    SPI.beginTransaction(adxlSettings);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer((XDATA_H << 1) | 0x01);
    for (int i = 0; i < 9; i++) buffer[i] = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    x = convert20bit(buffer[0], buffer[1], buffer[2]);
    y = convert20bit(buffer[3], buffer[4], buffer[5]);
    z = convert20bit(buffer[6], buffer[7], buffer[8]);
}

float xBuffer[N_SAMPLES];
float yBuffer[N_SAMPLES];
float zBuffer[N_SAMPLES];
int   sampleIndex = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("ADXL359 Vibration Measurement");

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    pinMode(DRDY_PIN, INPUT);
    SPI.begin();
    delay(50);

    uint8_t id = readReg(DEVID_REG);
    Serial.print("Device ID = 0x");
    Serial.println(id, HEX);
    if (id != 0xAD) {
        Serial.println("ADXL359 NOT FOUND - check wiring");
        while(1);
    }

    writeReg(RANGE_REG, 0x01);
    writeReg(POWER_CTL, 0x00);
    delay(100);

    attachInterrupt(digitalPinToInterrupt(DRDY_PIN), drdyISR, RISING);
    Serial.println("Ready");
}

void loop() {
    if (!dataReady) return;
    dataReady = false;

    int32_t rawX, rawY, rawZ;
    readAcceleration(rawX, rawY, rawZ);

    xBuffer[sampleIndex] = rawX / SENSITIVITY;
    yBuffer[sampleIndex] = rawY / SENSITIVITY;
    zBuffer[sampleIndex] = rawZ / SENSITIVITY;
    sampleIndex++;

    if (sampleIndex >= N_SAMPLES) {
        sampleIndex = 0;

        float sumSqX = 0, sumSqY = 0, sumSqZ = 0;

        for (int i = 0; i < N_SAMPLES; i++) {
            sumSqX += xBuffer[i] * xBuffer[i];
            sumSqY += yBuffer[i] * yBuffer[i];
            sumSqZ += zBuffer[i] * zBuffer[i];
        }

        float rmsX = sqrt(sumSqX / N_SAMPLES);
        float rmsY = sqrt(sumSqY / N_SAMPLES);
        float rmsZ = sqrt(sumSqZ / N_SAMPLES);

        Serial.print(rmsX, 6); Serial.print(",");
        Serial.print(rmsY, 6); Serial.print(",");
        Serial.println(rmsZ, 6);
    }
}
