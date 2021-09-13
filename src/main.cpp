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

    display.setTextSize(3);
    display.setTextColor(WHITE);

    Serial.println("Starting loop!");
}

void loop()
{
    // Temperature        
    unknownTemperatureResistor = knownTemperatureResistor * ((Vin / ((analogRead(temperatureInput) * Vin) / 1024.0)) -1);

    // GND- 2K - A0 - 5V
    //  |-\/\/-|-\/\/-|
    // out[] holds the values wanted in degrees C
    int out[] = { 170, 160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10 };

    // in[] holds the measured analogRead() values for defined distances
    // note: the in array should have increasing values
    int in[]  = {20, 24, 30, 40, 49, 64, 83, 108, 147, 200, 278, 388, 570, 844, 1286, 2031, 3100 };
    temperatureValue = multiMap(unknownTemperatureResistor, in, out, 17);

    // Pressure
    pressureRaw = analogRead(pressureInput);
    pressureValue = ((pressureRaw - pressureZero) * pressureTransducermaxPSI) / (pressureMax - pressureZero);
    pressureValueBar = pressureValue * 0.0689475729;

    display.clearDisplay();
    display.setCursor(5, 0);

//    display.print("R:");
//    display.println(unknownTemperatureResistor);

    display.print(temperatureValue, 1);
    display.setCursor(90, -14);
    display.print(".");
    display.setCursor(105, 0);
    display.print("C");

    display.setCursor(5, 30);
    display.print(pressureValueBar, 1);
    display.setCursor(97, 30);
    display.print("b");
    display.setCursor(110, 30);
    display.print(".");

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

    delay(50);
}

// note: the _in array should have increasing values
int multiMap(int val, int* _in, int* _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}
