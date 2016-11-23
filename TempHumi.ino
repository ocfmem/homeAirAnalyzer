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

//Air Quality
AirQuality airqualitysensor;
int current_quality =-1;

//Dust Sensor
int dust_pin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

//LEDS
#define red_led_pin 3
#define yellow_led_pin 5 
#define blue_led_pin 6  

void setup() {
  Serial.begin(9600);
  Serial.println("Starting");

  airqualitysensor.init(14);
  dht.begin();
  //pinMode(sensorPin, INPUT);

  pinMode(dust_pin,INPUT);

  pinMode(red_led_pin,OUTPUT);
  pinMode(yellow_led_pin,OUTPUT);
  pinMode(blue_led_pin,OUTPUT);
  starttime = millis();//get the current time;
  
}

void loop() {
    // Wait a few seconds between measurements.
  resetLeds();  
  delay(2000);

  getDHTValues(); 
  getAirQuality();
  getDustDectector();
  
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

void getAirQuality(){
    current_quality=airqualitysensor.slope();
  
    Serial.print("Air Quality: ");
    Serial.print(airqualitysensor.first_vol);
    Serial.print(" ");
    if (current_quality==0){
      Serial.println("Emergency   ");
      lightRedLed();
    }
    else if (current_quality==1){
      Serial.println("Hi Pollution");
      lightYellowLed();
    }
    else if (current_quality==2){
      Serial.println("Lo Pollution");
      lightBlueLed(); 
    }
    else if (current_quality ==3){
      Serial.println("Fresh air   ");
      lightBlueLed(); 
    }
}

void getDustDectector(){
  // Checking Dust Sensor
  duration = pulseIn(dust_pin, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;

  if ((millis()-starttime) > sampletime_ms) //if the sample time == 30s
  {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    /*Serial.print(lowpulseoccupancy);
    Serial.print(",");
    Serial.print(ratio);
    Serial.print(",");
    Serial.println(concentration);*/
    Serial.print("Particles:");
    Serial.print("    ");
    Serial.println(concentration);
    lowpulseoccupancy = 0;
    starttime = millis();
  }
}

void resetLeds(){
      digitalWrite(red_led_pin, LOW);  
      digitalWrite(yellow_led_pin, LOW);  
      digitalWrite(blue_led_pin, LOW);  
}

void lightRedLed(){
      digitalWrite(red_led_pin, HIGH);  
      digitalWrite(yellow_led_pin, LOW);  
      digitalWrite(blue_led_pin, LOW);  
}

void lightYellowLed(){
      digitalWrite(red_led_pin, LOW);  
      digitalWrite(yellow_led_pin, HIGH);  
      digitalWrite(blue_led_pin, LOW);  
}

void lightBlueLed(){
      digitalWrite(red_led_pin, LOW);  
      digitalWrite(yellow_led_pin, LOW);  
      digitalWrite(blue_led_pin, HIGH);  
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

