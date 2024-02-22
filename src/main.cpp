/**
 * @name IMP - Meranie srdecneho tepu [digitalny senzor]
 * @file main.cpp
 * @author Marcel Feiler - xfeile00
 */

// pouzite kniznice
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
// nastavenie konstant pre parametre displeja
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_MOSI   23
#define OLED_CLK    18
#define OLED_DC     16
#define OLED_CS     5
#define OLED_RESET  17

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

MAX30105 particleSensor;
// premenne pre vypocet SpO2
double avered    = 0; 
double aveir     = 0;
double sumirrms  = 0;
double sumredrms = 0;
int    i         = 0;
int    Num       = 100;  // vypocet SpO2 s tymto vzorkovacim intervalom
float  ESpO2;            // inicializacia pre SpO2
double FSpO2     = 0.7;  // filter factor pre urcenie SpO2
double frate     = 0.95; // low pass filter pre IR/red LED hodnoty
#define TIMETOBOOT 3000  // hodnota v msec pre output SpO2
#define SCALE      88.0  // nastavenie zobrazenia merania tepu a SpO2 v rovnakej skale
#define SAMPLING   100 //25 //5 //vzorkovanie
#define FINGER_ON  30000 // ak cerveny signal nizsi ako tato hodnota, indikuje sa absencia prsta na senzore
#define USEFIFO
//--------------------------------------------------------------------------

// premenne pre meranie tepu za minutu a priemerneho tepu za minutu
byte rateSpot = 0;
long lastBeat = 0; // cas zaznamenania posledneho uderu srdca
int beatAvg;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE]; //pole pre tepy za minutu, pomocne pole pre zistenie beatAvg hodnoty
float beatsPerMinute;
//--------------------------------------------------------------------------

// inicializacia displeja a senzoru MAX30102
void initializingBoth() {
  // inicializacia senzora
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }

  // inicializacia displeja
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
}

// funkcia pre konfiguraciu senzora MAX30102
void particleSettingUp() {
  byte ledBrightness = 0x7F; // moznosti: 0=Off to 255=50mA
  byte sampleAverage = 4; // moznosti: 1, 2, 4, 8, 16, 32
  byte ledMode       = 2; // moznosti: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  // moznosti: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate     = 3200; // moznosti: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth     = 411; // moznosti: 69, 118, 215, 411
  int adcRange       = 16384; // moznosti: 2048, 4096, 8192, 16384
  
  // nastavenie chcenych parametrov senzoru MAX30102
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //konfiguracia senzora
  particleSensor.enableDIETEMPRDY();
}

// funkcia pre vypocet tepov za minutu (beatsPerMinute) a priemerneho tepu za minutu (beatAvg)
void heartRateComp() {
  long delta = millis() - lastBeat;
  lastBeat = millis();

  beatsPerMinute = 60 / (delta / 1000.0);

  if (beatsPerMinute < 255 && beatsPerMinute > 20){ //ak je namerana regularna hodnota
    rates[rateSpot++] = (byte)beatsPerMinute; // ukladanie tohto citania v poli
    rateSpot %= RATE_SIZE; // wrapne premennu

    // vezme priemer nacitanych hodnot
    beatAvg = 0;
    for (byte x = 0; x < RATE_SIZE; x++)
      beatAvg += rates[x];
    beatAvg /= RATE_SIZE;
  }
}

// funkcia pre vypis varovania na displej z dovodu absencie prsta na senzore
void showWarning() {
  display.clearDisplay();
  display.setCursor(0, 30);
  display.println("NO FINGER ON SENSOR");
  display.display();
}

// funkcia pre vypis BPM, AVGBPM, EsPO2 levelu na displej
void showOnDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("BPM=");
  display.println(beatsPerMinute);
  display.setCursor(0, 10);
  display.print("Avg BPM=");
  display.println(beatAvg);
  display.setCursor(0, 20);
  display.print("Oxygen % = ");
  display.print(ESpO2);
  display.println("%");
  display.display();
}

// funkcia pre vypocet hodnoty SpO2
int computingSpO2(double fred, double fir, uint32_t red, uint32_t ir) {
  avered = avered * frate + (double)red * (1.0 - frate); // priemer red levelu low pass filtrom
  aveir = aveir * frate + (double)ir * (1.0 - frate); // priemer IR levelu low pass filtrom
  sumredrms += (fred - avered) * (fred - avered); // stvorcovy sucet alternate komponenty red levelu
  sumirrms += (fir - aveir) * (fir - aveir); // stvorcovy sucet alternate komponenty IR levelu
  if ((i % SAMPLING) == 0) {
    if (millis() > TIMETOBOOT) {
      float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
      float red_forGraph = (2.0 * fred - avered) / avered * SCALE;
      // skracovanie pre automatické škálovanie sériového plotra
      if (ir_forGraph > 100.0) ir_forGraph = 100.0;
      if (ir_forGraph < 80.0) ir_forGraph = 80.0;
      if (red_forGraph > 100.0) red_forGraph = 100.0;
      if (red_forGraph < 80.0) red_forGraph = 80.0;

      if (ir < FINGER_ON) { // prst nie je na senzore
        showWarning();
        return 1;
      }

      if (ir > FINGER_ON) {
        showOnDisplay();
      }
    }
  }

  return 0;
}



void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  // inicializacia displeja a senzora MAX30102
  initializingBoth();

  display.display();
  delay(2000); 
  display.clearDisplay();  // vycistenie bufferu

  // nastavenie parametrov senzoru MAX30102
  particleSettingUp();
}

void loop()
{
  uint32_t ir, red;
  double fred, fir;
  double SpO2 = 0; // raw SpO2 pred low pass filtraciou
  
#ifdef USEFIFO
  particleSensor.check();

  while (particleSensor.available()) {
#ifdef MAX30105
    red = particleSensor.getFIFORed(); // Sparkfun's MAX30105
    ir  = particleSensor.getFIFOIR();  // Sparkfun's MAX30105
#else
    red = particleSensor.getFIFOIR();  // Sparkfun's MAX30105
    ir  = particleSensor.getFIFORed(); // Sparkfun's MAX30105
#endif

    long irValue = particleSensor.getIR();
    // zachytenie tepu
    if (checkForBeat(irValue) == true) {
      heartRateComp();  // volanie procedury pre vypocet tepov za minutu a priemerneho tepu za minutu
    }


    i++; //counter
    fred = (double)red;
    fir  = (double)ir;
    int tmp_break = computingSpO2(fred, fir, red, ir);  //vypocet SpO2
    if (tmp_break != 0) // v pripade zistenie absencie prsta na senzore
      break; 

    if ((i % Num) == 0) { // v pripade ked sme dosiahli 100-ho vzorku
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      SpO2 = -23.3 * (R - 0.4) + 100;
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2; // low pass filter
      sumredrms = 0.0;
      sumirrms = 0.0;
      i = 0;
      break;  // dalsia sada vzorkov od 0
    }

    particleSensor.nextSample(); // pokracovanie na dalsi vzorek
  #endif
  }

}