#include <AirQuality.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "Arduino.h"
#include "U8glib.h"

#define DHTPIN 2  //PIN Numérique du capteur TEMP/HUMI
#define MQ135PIN A7//Air fQuality Sensor  
#define PM10_PIN 8
#define PM25_PIN 9

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE);
float tempRes = 0;
float temp = 0;
float hum = 0;

//Air Quality
AirQuality airqualitysensor;
int current_quality =-1;
int air_quality_value = 0;

//Dust Sensor
unsigned long duration;
unsigned long starttime;
unsigned long endtime;
unsigned const long sampletime_ms = 30000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
long concentrationPM_1_0 = 0;
long concentrationPM_2_5 = 0;

//LEDS
#define red_led_pin 3
#define yellow_led_pin 5 
#define blue_led_pin 6  

//Display
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);  
const int BuffSize= 9; 

void setup() {
  
  u8g.firstPage(); 
  do {  
    u8g.setFont(u8g_font_profont15);
    u8g.drawStr(0,14, "Init...");  
  } while( u8g.nextPage() ); 
     
  Serial.begin(9600);
  Serial.println("Starting");

  airqualitysensor.init(14);
  dht.begin();
  //pinMode(sensorPin, INPUT);

  pinMode(PM10_PIN,INPUT);
  pinMode(PM25_PIN,INPUT);

  pinMode(red_led_pin,OUTPUT);
  pinMode(yellow_led_pin,OUTPUT);
  pinMode(blue_led_pin,OUTPUT);
 
  delay(2000);
}

void loop() {
  getDHTValues(); 
  displayTempHum();
  
  getAirQuality();
  getDustDectector(); //prend 1 minute :/
  blinkLeds();
  displayAirQuality();
}

//Valeur du capteur de température et humidité.
void getDHTValues(){
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    return;
  }

  hum = h;
  temp = t;
  // Compute heat(chaleur ressentie) index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  tempRes = hic;

}  

void getAirQuality(){
    current_quality=airqualitysensor.slope();
    air_quality_value = airqualitysensor.first_vol;
}


void getDustDectector(){
  // Checking Dust Sensor
  concentrationPM_1_0 = (long)getPM(PM10_PIN);
  concentrationPM_2_5 = (long)getPM(PM25_PIN);
}

void blinkLeds(){
    if (current_quality==0){
      lightRedLed();
    }
    else if (current_quality==1){
      lightYellowLed();
    }
    else if (current_quality==2){
      lightBlueLed(); 
    }
    else if (current_quality ==3){
      lightBlueLed(); 
    }
    
    delay(500);

    resetLeds();
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

void displayTempHum(){
   char buf[BuffSize];
  
   u8g.firstPage();  
  do {
      int index = 0;
      u8g.setFont(u8g_font_profont12);
      index = u8g.drawStr( index, 10, "T: ");
      if(temp>=0){
          index += u8g.drawStr( index, 10, dtostrf(temp, 4, 1, buf));
      }else{
          index += u8g.drawStr( index, 10, dtostrf(temp, 5, 1, buf));
      }
      index += u8g.drawStr( index, 10, "\260C");
      index+=5;//espace en Temp et Hum
      
      index += u8g.drawStr( index, 10, "Hum: ");
      index += u8g.drawStr( index, 10, dtostrf(hum, 2, 0, buf));
      index += u8g.drawStr( index, 10, "%");
      index = 0;

      u8g.setFont(u8g_font_profont15);
      index += u8g.drawStr( index, 24, "Ressentie: ");
      index = 0;
      u8g.setFont(u8g_font_profont22);
      index += u8g.drawStr( index, 52, dtostrf(tempRes, 5, 1, buf));
      index += u8g.drawStr( index, 52, "\260C");
  } while( u8g.nextPage() );
}

void displayAirQuality(){
   char buf[BuffSize];
  String str;
  
   u8g.firstPage();  
  do {
      int index = 0;
      u8g.setFont(u8g_font_profont12);
      index += u8g.drawStr( index, 10, "Q.Air: ");
      if (current_quality==0){
        index += u8g.drawStr( index, 10, "Danger!!!");
      }
      else if (current_quality==1){
        index += u8g.drawStr( index, 10, "Mauvaise");
      }
      else if (current_quality==2){
        index += u8g.drawStr( index, 10, "Correcte");
      }
      else if (current_quality ==3){
        index += u8g.drawStr( index, 10, "Bonne");
      }
      index += u8g.drawStr( index, 10, "(");
      snprintf (buf, BuffSize, "%d", air_quality_value);
      index += u8g.drawStr( index, 10, buf);
      index += u8g.drawStr( index, 10, ")");

      index = 0;
      u8g.setFont(u8g_font_profont12);
      index += u8g.drawStr( index, 21, "Conc.Particules: ");

      float ppmv=getPPMV(concentrationPM_1_0);
      drawPMValue(32, buf, str, "PM>1.0: ",ppmv);
      
      ppmv=getPPMV(concentrationPM_2_5);
      drawPMValue(43, buf, str, "PM>2.5: ",ppmv);

      long under2_5 = concentrationPM_1_0 - concentrationPM_2_5;
      ppmv=getPPMV(under2_5);

      Serial.print(concentrationPM_1_0);
      Serial.print("/");
      Serial.print(concentrationPM_2_5);
      Serial.print("/");
      Serial.println(under2_5);
      
      drawPMValue(54, buf, str, "PM2.5: ",ppmv);
      
  } while( u8g.nextPage() );

  delay(20000);
}

void drawPMValue(int pos_y, char buf[], String str, char *labelPM, float ppmv){
      int index = 0;
      index += u8g.drawStr( index, pos_y,labelPM);
      str = String(ppmv);
      str.toCharArray(buf, BuffSize);
      index += u8g.drawStr( index, pos_y,buf);
      index += u8g.drawStr( index, pos_y, "mg/m3 ");
}

long getPM(int DUST_SENSOR_DIGITAL_PIN) {

  starttime = millis();

  while (1) {
  
    duration = pulseIn(DUST_SENSOR_DIGITAL_PIN, LOW);
    lowpulseoccupancy += duration;
    endtime = millis();
    
    if ((endtime-starttime) > sampletime_ms)
    {
    ratio = (lowpulseoccupancy-endtime+starttime)/(sampletime_ms*10.0);  // Integer percentage 0=>100
    long concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    lowpulseoccupancy = 0;
    return(concentration);    
    }
  }  
}

float getPPMV(long concentration){
  return (((concentration*0.0283168/100) *  (0.08205*temp)/0.01))/1000;
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

