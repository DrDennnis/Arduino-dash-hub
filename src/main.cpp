#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>

const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial mySerial(rxPin, txPin);

const unsigned int MAX_BUFFER_LENGTH = 10;
byte buf[MAX_BUFFER_LENGTH];

// oil Temperature
// Fill in some datapoints in a Steinhart-Hart model, and fill this in.
const double knownTemperatureResistor = 150.0; // Get an resistor that should be most accurate at the required temp
const double c1 = 0.0015104550472979056, c2 = 0.0002489673846169903, c3 = 1.0325123237917102e-8;

// oil Pressure
const float pressureZero = 102.4; // raw value at 0 psi 0.5v
const float pressureMax = 921.6; // raw value at max psi 4.5v
const int pressureTransducermaxPSI = 150;

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting loop!");

    // Define pin modes for TX and RX
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    mySerial.begin(9600);
}

void printBufAsHex(byte* buf) {
  for (unsigned int i = 0; i < MAX_BUFFER_LENGTH; i++) {
    if (buf[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

//  GND - 1k resistor - A0 - sensor - 5V
float getTemperatureByPin(int pin) {
  double adc_raw = analogRead(pin);
  double voltage = adc_raw / 1024.0 * 5;
  double resistance = ((1024 * knownTemperatureResistor / adc_raw) - knownTemperatureResistor);

  double logR  = log(resistance);
  double logR3 = logR * logR * logR;

  // 9.5 is // Dissipation factor (mW/°C)
  double steinhart = 1.0 / (c1 + c2 * logR + c3 * logR3);
  return steinhart - voltage * voltage / (9.5 * knownTemperatureResistor) - 273.15;
}

float getPressureByPin(int pin) {
  float pressureRaw = analogRead(pin);
  float pressureValue = ((pressureRaw - pressureZero) * pressureTransducermaxPSI) / (pressureMax - pressureZero);

  // cool, now lets not get below 0, how do we handle vacuum anyway - vacuum can be in kpa
  if (pressureValue < 0) {
    pressureValue = 0;
  }

  if (pressureValue > pressureTransducermaxPSI) {
    pressureValue = 150;
  }

  // From PSI to BAR
  return pressureValue * 0.0689475729;
}

float getLambdaByPin(int pin) {
  float lambdaVoltage = analogRead(pin) * (5.0 / 1023.0);
  return (0.1621 * lambdaVoltage) + 0.4990;
}

void loop() {
  buf[0] = (int)getTemperatureByPin(A0);
  buf[1] = (int)(getPressureByPin(A1) * 10);
  buf[2] = (int)(getLambdaByPin(A2) * 100);
  buf[3] = 0;
  buf[4] = 0;
  buf[5] = 0;
  buf[6] = 0;
  buf[7] = 0;

  int checksum = 0;
  for (unsigned int i = 0; i < (MAX_BUFFER_LENGTH - 2); i++) {
    checksum ^= buf[i];
  }

  buf[8] = checksum,
  buf[9] = '\n';

  // printBufAsHex(buf);

  mySerial.write(buf, MAX_BUFFER_LENGTH);

  delay(150);
}
