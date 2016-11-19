#include <AirQuality.h>

#include <Adafruit_Sensor.h>

#include <DHT.h>
#include <DHT_U.h>
#include "Arduino.h"

#define DHTPIN 2  //PIN Numérique du capteur TEMP/HUMI
#define MQ135PIN A7//Air Quality Sensor 

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

AirQuality airqualitysensor;
int current_quality =-1;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting");

  airqualitysensor.init(14);
  dht.begin();
  //pinMode(sensorPin, INPUT);
}

void loop() {
    // Wait a few seconds between measurements.
  delay(2000);

  getDHTValues(); 

  current_quality=airqualitysensor.slope();
  
    Serial.print("Air Quality: ");
    Serial.print(current_quality);
    Serial.print(" ");
    if (current_quality==0)
        Serial.println("Emergency   ");
    else if (current_quality==1)
        Serial.println("Hi Pollution");
    else if (current_quality==2)
        Serial.println("Lo Pollution");
    else if (current_quality ==3)
        Serial.println("Fresh air   ");
    
}

//Valeur du capteur de température et humidité.
void getDHTValues(){
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();


  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat(chaleur ressentie) index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);


  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.println(" *C ");
}  

ISR(TIMER2_OVF_vect)
{
  if(airqualitysensor.counter==122)//set 2 seconds as a detected duty
  {
    airqualitysensor.last_vol=airqualitysensor.first_vol;
    airqualitysensor.first_vol=analogRead(MQ135PIN);
    airqualitysensor.counter=0;
    airqualitysensor.timer_index=1;
    //PORTB=PORTB^0x20;
    }
  else
  {
    airqualitysensor.counter++;
  }
}
