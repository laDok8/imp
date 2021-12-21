#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"           
#include "heartRate.h"         
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>



//inicalizace MAX30102
MAX30105 particleSensor;
long lastBeat = 0;
float bpm;
int beatAvg;


//inicializace SSD1306
#define OLED_MOSI  19
#define OLED_CLK   18
#define OLED_DC    13
#define OLED_CS    12
#define OLED_RESET 23
Adafruit_SSD1306 display(128, 64,  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);


//pomocna funkce zobrazeni textu
void displayText(int bpm){
  display.clearDisplay();

  // vlevo nahore start
  display.setCursor(0,0);             
  display.setTextSize(1);             
  display.setTextColor(SSD1306_WHITE);
  if( bpm == 0){
    display.print("PUT FINGER ON SENSOR");
  } else {
    display.print("Heart Rate:" + String(bpm) + " BPM");
  }
  display.display();
}

const uint8_t bpmFilter = 4;
byte vzorky[4]; //Array of heart rates
byte p = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");
  
  // init max30102
  particleSensor.begin(Wire, I2C_SPEED_FAST); //defaultni port (SDA/SCL)
  particleSensor.setup(); 
  particleSensor.setPulseAmplitudeRed(0x0a); //RED led


  // init ssd1306
  display.begin(SSD1306_SWITCHCAPVCC);
  display.display();

}

//hlavni smycka
void loop() {
 long irVal= particleSensor.getIR();

 if (checkForBeat(irVal)){
    long delta = millis() - lastBeat;
    lastBeat = millis();

    bpm = 60 / (delta / 1000.0);

    // vyfiltrujeme vhodne hodnoty
    if (bpm > 20 && bpm < 255)
    {
      vzorky[p++] = (byte)bpm; 
      p %= bpmFilter;

      //low pass filter ze 4 vzorku
      beatAvg = 0;
      for (byte x = 0 ; x < bpmFilter ; x++)
        beatAvg += vzorky[x];
      beatAvg /= bpmFilter;
    }
  }

  Serial.print("BPM="+ String(bpm)+ ", Avg BPM=" + String(beatAvg) + "\n");

  if (irVal < 50000)
    displayText(0);
  else{
    displayText((int)beatAvg);
  }
}

