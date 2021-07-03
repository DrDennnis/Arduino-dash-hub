#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP280 bme;

#define OLED_RESET -1
Adafruit_SH1106 display(OLED_RESET);

// Main
const int baudRate = 9600;
const int Vin = 5;

// Temperature
const int temperatureInput = A0;
const float knownTemperatureResistor = 2000;

int temperatureRaw = 0;
float unknownTemperatureResistor = 0;
float temperatureValue = 0;

// Pressure
const int pressureInput = A1;
const int pressureZero = 102.4; // raw value at 0 psi 0.5v
const int pressureMax = 921.6; // raw value at max psi 4.5v
const int pressureTransducermaxPSI = 150;

float pressureRaw = 0;
float pressureValue = 0;
float pressureValueBar = 0;

void setup()
{
    Serial.begin(baudRate);

    display.begin(SH1106_SWITCHCAPVCC, 0x3C);

    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }

    display.setTextSize(2);
    display.setTextColor(WHITE);

    Serial.println("Starting loop!");
}

void loop()
{
    // Temperature
    // GND- 2K - A0 - 5V
    //  |-\/\/-|-\/\/-|
    temperatureRaw = analogRead(temperatureInput);
    unknownTemperatureResistor = knownTemperatureResistor * ((Vin / ((temperatureRaw * Vin) / 1024.0)) -1);

    float Vout = (Vin * temperatureRaw) / 1023;    // Convert Vout to volts
    float R = knownTemperatureResistor * (1 / ((Vin / Vout) - 1));  // Formula to calculate tested resistor's value

    temperatureValue = -30.5 * log(unknownTemperatureResistor / 1741.);
    //temperatureValue = 1350 * pow(log(unknownTemperatureResistor / 200), 2); // ^2
    //temperatureValue = 560 * pow(log(unknownTemperatureResistor / 260), 2); // ^2

    // Pressure
    pressureRaw = analogRead(pressureInput);
    pressureValue = ((pressureRaw - pressureZero) * pressureTransducermaxPSI) / (pressureMax - pressureZero);
    pressureValueBar = pressureValue * 0.0689475729;

    display.clearDisplay();
    display.setCursor(0, 0);

    display.print("R:");
    display.println(unknownTemperatureResistor);

    display.print("Rx:");
    display.println(R);

    display.print(temperatureValue);
    display.println(" *C");

    display.print(pressureValueBar);
    display.println(" bar");

//    display.print(bme.readTemperature());
//    display.println(" *C");
//
//    display.print(bme.readPressure() / 100.0F);
//    display.println("hPa press");
//
//    display.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
//    display.println("m atti");

    //display.print(bme.readHumidity());
    //display.println("h%");

    display.display();

    delay(100);
}
