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
unsigned long durationPM10;
unsigned long durationPM25;
unsigned const long sampletime_ms = 60000;//sample 60s ;
unsigned long lowpulseoccupancyPM10 = 0;
unsigned long lowpulseoccupancyPM25 = 0;
long concentrationPM_1_0 = 0;
long concentrationPM_2_5 = 0;
unsigned long cumulatedDurationPM10;
unsigned long cumulatedDurationPM25;
unsigned const short BuffSize= 9; 
char buf[BuffSize];
   
//LEDS
#define red_led_pin 3
#define yellow_led_pin 5 
#define blue_led_pin 6  

//Display
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);  

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
  getPMs(2500);
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
      u8g.setFont(u8g_font_profont15);
      index += u8g.drawStr( index, 52, dtostrf(tempRes, 5, 1, buf));
      index += u8g.drawStr( index, 52, "\260C");
  } while( u8g.nextPage() );
}

void displayAirQuality(){
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

      float ppmvPM10=pcs2ugm3(concentrationPM_1_0);
      drawPMValue(32, buf, str, "PM>1.0: ",ppmvPM10);
      
      float ppmvPM25=pcs2ugm3(concentrationPM_2_5);
      drawPMValue(43, buf, str, "PM>2.5: ",ppmvPM25);
      
      drawPMValue(54, buf, str, "PM2.5: ",ppmvPM10 - ppmvPM25);
      
  } while( u8g.nextPage() );

  delay(5000);
}

void drawPMValue(int pos_y, char buf[], String str, char *labelPM, float ppmv){
      int index = 0;
      index += u8g.drawStr( index, pos_y,labelPM);
      str = String(ppmv);
      str.toCharArray(buf, BuffSize);
      index += u8g.drawStr( index, pos_y,buf);
      index += u8g.drawStr( index, pos_y, "mg/m3 ");
}

void getPMs(long maxDurationMS){
  cumulateLowOccupancyPM10(maxDurationMS);
  cumulateLowOccupancyPM25(maxDurationMS);
  if(cumulatedDurationPM10>sampletime_ms
      && cumulatedDurationPM25>sampletime_ms){
      float ratio = (lowpulseoccupancyPM10)/(cumulatedDurationPM10*10.0);  // Calcul du ratio:Temps en mode LOW sur toute la durée. pulseIn retourne la valeur en microseconds.
      concentrationPM_1_0 = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve in pcs/0.01cf
      lowpulseoccupancyPM10 = 0;
      cumulatedDurationPM10 = 0;
//      Serial.print("PM10: ");
//      Serial.print(ratio);
//      Serial.print(" / ");
//      Serial.print(concentrationPM_1_0);
//      Serial.println(" pcs/0.01cf");

      ratio = (lowpulseoccupancyPM25)/(cumulatedDurationPM25*10.0);  // Calcul du ratio:Temps en mode LOW sur toute la durée. pulseIn retourne la valeur en microseconds.
      concentrationPM_2_5 = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve in pcs/0.01cf
      lowpulseoccupancyPM25 = 0;
      cumulatedDurationPM25 = 0;
//      Serial.print("PM25: ");
//      Serial.print(ratio);
//      Serial.print(" / ");
//      Serial.print(concentrationPM_2_5);
//      Serial.println(" pcs/0.01cf");
  }         
}


void cumulateLowOccupancyPM10(long maxDurationMS){
    long starttime = millis();
    while(1){
      durationPM10 = pulseIn(PM10_PIN, LOW);
      lowpulseoccupancyPM10 += durationPM10;
       long endtime = millis();
       cumulatedDurationPM10+= endtime-starttime;
       if(endtime-starttime>maxDurationMS){
          //Si on a déjà passé 2,5 secondes à accumuler on suspend le calcul
          return (0);
       }
    }
}

void cumulateLowOccupancyPM25(long maxDurationMS){
    long starttime = millis();
    while(1){
      durationPM25 = pulseIn(PM25_PIN, LOW);
      lowpulseoccupancyPM25 += durationPM25;
       long endtime = millis();
       cumulatedDurationPM25+= endtime-starttime;
       if(endtime-starttime>maxDurationMS){
          //Si on a déjà passé 2,5 secondes à accumuler on suspend le calcul
          return (0);
       }
    }
}

//Quelle formule est bonne entre getPPVM et pcs2ugm3?
//float getPPMV(long concentration){
//  return (((concentration*0.0283168/100) *  (0.08205*temp)/0.01))/1000;
//}

//https://github.com/intel-iot-devkit/upm/pull/409/files
double pcs2ugm3 (double concentration_pcs)
{
    double pi = 3.14159;
    // All particles are spherical, with a density of 1.65E12 µg/m3
    double density = 1.65 * pow (10, 12);
    // The radius of a particle in the PM2.5 channel is .44 µm
    double r25 = 0.44 * pow (10, -6);
    double vol25 = (4/3) * pi * pow (r25, 3);
    double mass25 = density * vol25; // ug
    double K = 3531.5; // per m^3 

    return concentration_pcs * K * mass25;
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

