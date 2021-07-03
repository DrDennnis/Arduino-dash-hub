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

//////////////
#define coolantsensorDivider 2970   //defines the resistor value that is in series in the voltage divider
#define coolantsensorPin A0         //defines the analog pin of the input voltage from the voltage divider
#define NUMSAMPLES 5                //defines the number of samples to be taken for a smooth average

const float steinconstA = 0.00132774106461327;        //steinhart equation constant A, determined from wikipedia equations
const float steinconstB = 0.000254470874104285;       //steinhart equation constant B, determined from wikipedia equations
const float steinconstC = 0.000000101216538378909;    //steinhart equation constant C, determined from wikipedia equations

int samples[NUMSAMPLES];                              //variable to store number of samples to be taken
//////////////


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
    //temperatureRaw = analogRead(temperatureInput);
    //float voltage = (temperatureRaw / 1023.0) * 5;
    //unknownTemperatureResistor = knownTemperatureResistor * ((Vin / ((temperatureRaw * Vin) / 1024.0)) -1);

    //temperatureValue = -30.5 * log(unknownTemperatureResistor / 1741.);
    //temperatureValue = 1350 * pow(log(unknownTemperatureResistor / 200), 2); // ^2
    //temperatureValue = 560 * pow(log(unknownTemperatureResistor / 260), 2); // ^2

///////////
    uint8_t i;                                          //integer for loop
    float average;                                      //decimal for average
    
    for (i=0; i<NUMSAMPLES; i++) {                      
        samples[i] = analogRead(coolantsensorPin);        //takes samples at number defined with a short delay between samples
        delay(10);
    }

    average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
    average += samples[i];                            //adds all number of samples together
  }
  average /= NUMSAMPLES;                              //divides by number of samples to output the average

  Serial.print("Average Analog Coolant Reading = ");
  Serial.println(average);                                        //analog value at analog pin into arduino
  average = (coolantsensorDivider*average)/(1023-average);        //conversion equation to read resistance from voltage divider
  Serial.print("Coolant Sensor Resistance = ");
  Serial.println(average);

  float steinhart;                              //steinhart equation to estimate temperature value at any resistance from curve of thermistor sensor
  steinhart = log(average);                     //lnR
  steinhart = pow(steinhart,3);                 //(lnR)^3
  steinhart *= steinconstC;                     //C*((lnR)^3)
  steinhart += (steinconstB*(log(average)));    //B*(lnR) + C*((lnR)^3)
  steinhart += steinconstA;                     //Complete equation, 1/T=A+BlnR+C(lnR)^3
  steinhart = 1.0/steinhart;                    //Inverse to isolate for T
  steinhart -= 273.15;                          //Conversion from kelvin to celcius


///////////

    // Pressure
    pressureRaw = analogRead(pressureInput);
    pressureValue = ((pressureRaw - pressureZero) * pressureTransducermaxPSI) / (pressureMax - pressureZero);
    pressureValueBar = pressureValue * 0.0689475729;

    display.clearDisplay();
    display.setCursor(0, 0);

    display.print(steinhart);
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
