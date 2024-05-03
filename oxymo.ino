/*
   
   Oxymo
   
   Created with help from
   https://github.com/sparkfun/MAX30105_Breakout
   
*/

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "LowPower.h"
#include "U8glib.h"
#include "math.h"

//#define DEBUG
// tiny:
// u8g_font_4x6r

#define IR_INT 2
#define BATT 12

#define OK_BUTTON 5
#define LEFT_BUTTON 2
#define RIGHT_BUTTON 3
#define TIMEOUT 512
#define HEART_LED 4
#define THRESHOLD 50000
#define SKIP 2

byte symbols[0x9F - 0x20 + 1];
MAX30105 particleSensor;

//U8GLIB_SSD1306_128X64 u8g(13, 11, 9, 10, 8);  // SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
U8GLIB_SH1106_128X64 u8g(13, 11, 9, 10);  // SW SPI Com: SCK = 13, MOSI = 11, CS = 10, dc = 9

const int SAMPLE_SIZE = 6; //Increase this for more averaging. 4 is good.
float rates[SAMPLE_SIZE]; //Array of heart rates
int rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

const uint32_t SAMPLE_LENGTH = 128 ; //Number of samples to record
float ac_signal_data[SAMPLE_LENGTH];

// very simple chart
uint8_t top = 32;
uint8_t bottom = 63;
uint8_t scan = 0;
char symb[2] = " ";
char symb2[2] = " ";

uint8_t symbX, symbY = -1;

int prev_left = HIGH;
int prev_right = HIGH;
String s;

const uint32_t N_AVG_SAMPLES = 16; //Number of samples to record
float bpms[N_AVG_SAMPLES] = { 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f, 80.0f };
uint8_t avg_scan = 0;
uint8_t n_avgs = 0;

float beatEst;
float avgBpm;

float sumAvg;
float avg_samples;
int maximum;


uint8_t font = 0;
uint32_t timer = 0;
uint8_t beat = 0;
uint8_t frame = 0;
uint8_t batt;
uint8_t butt;
uint8_t butt2;
uint8_t butt3;
uint8_t heart = 1;
uint8_t min_samples_reached = 0;

bool finger_down = false;

long irValue;
long redValue;

long prev_irValue;
long samples;

template <typename T>
void printAt(int x, int y, const T & data) {
  u8g.setPrintPos(x, y);
  u8g.print(data);
}

// splash screen with string message
void splash(String msg, int stars, int pause = 1000) {

  u8g.firstPage();

  do {
    printAt(0, 12, msg);
    u8g.setFont(u8g_font_unifont_77);

    for (int i = 0; i < 8; i++) {
      printAt(16 * i, 64, 8);
    }

    for (int i = 0; i < stars; i++) {
      int x = random(0, 128);
      int y = random(0, 48);
      u8g.drawPixel(x, y);
    }

    u8g.setFont(u8g_font_7x13);

  } while ( u8g.nextPage() );

  delay(pause);

}

// reset by jump pc to 0
void resetFunc() {
  asm volatile ("jmp 0x0");
}

// send atmega to sleep
void sleep() {

  attachInterrupt(digitalPinToInterrupt(LEFT_BUTTON), wake, HIGH);

  delay(100);

  particleSensor.shutDown();

  delay(100);

  u8g.sleepOn();

  delay(100);

  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

}

// wake up routine
void wake() {

  delay(100);
  // hack to fix MAX30102 not restarting after sleep
  resetFunc();

  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
  //digitalWrite(SENS_PWR,HIGH);
  //detachInterrupt(1);
  u8g.sleepOff();
  //particleSensor.setLEDMode(1);
  delay(100);

  particleSensor.wakeUp();

}

void setup()
{

  // vertically flip display
  u8g.setRot180();
  symb[0] = 0x81;


#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Init");
#endif

  // Initialize MAX ir sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
#ifdef DEBUG
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
#endif
  }
  particleSensor.setLEDMode(1);
  particleSensor.setPulseAmplitudeRed(0x0);
  particleSensor.setPulseAmplitudeIR(0x0);
  particleSensor.setPulseAmplitudeProximity(0x0);
  particleSensor.setProximityThreshold(0x00);
  particleSensor.enablePROXINT();
  particleSensor.setPulseWidth(0x00);
  particleSensor.setup(); //Configure sensor with default settings

  // initialize OLED display
  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensityf
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255, 255, 255);
  }

  pinMode(OK_BUTTON, INPUT_PULLUP);
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
  pinMode(IR_INT, INPUT);
  pinMode(HEART_LED, OUTPUT);
  pinMode(BATT, INPUT);

  for (int i = 0; i < 128; i++) {
    ac_signal_data[i] = i;
  }

}

void Reset() {

  for (uint32_t i = 0; i < SAMPLE_LENGTH; i++) {
    ac_signal_data[i] = 0;
  }

  beatEst = 0;
  scan = 0;
  samples = 0;

}

// print heart character
void print_heart_char() {
  u8g.setFont(u8g_font_unifont_76);
  u8g.drawStr(0, 12, symb);
  u8g.setFont(u8g_font_u8glib_4r);
}

void draw_box(int x, int y, int w, int h) {
  u8g.drawLine(x, y, x, y + h - 1);
  u8g.drawLine(x, y + h - 1, x + w - 1, y + h - 1);
  u8g.drawLine(x + w - 1, y + h - 1, x + w - 1, y);
  u8g.drawLine(x + w - 1, y, x, y);
}

uint8_t midpoint;
float min_value = 100;
float max_value = -100;

// find scale of ir data chart
void find_scale() {

  min_value = 100;
  max_value = -100;

  size_t start = 0;

  for (size_t i = 0; i < scan; i++) {
    if (ac_signal_data[i] < min_value) {
      min_value = ac_signal_data[i];
    }
    if (ac_signal_data[i] > max_value) {
      max_value = ac_signal_data[i];
    }
  }

}

int SCREEN_WIDTH = 128;
int SCREEN_HEIGHT = 64;

// method to plot sensor data on the display
void plot(float *data, int x, int y, int w, int h, bool show_data) {

  // Clear the renderer
  // Calculate min and max values from the signal
  // Calculate the y-axis range for the signal
  //draw_box(x, y, w, h);

  if (!show_data) {
    //  return;
  }

  float yRange = max_value - min_value;

  int x2, y2;
  int x1 = x;
  int y1 = min(h + 1, max(0, y + static_cast<int>(((static_cast<float>(ac_signal_data[0]) - min_value) / yRange) * h)));

  // Plot the signal within the specified window
  for (size_t i = 0; i < SAMPLE_LENGTH; i++) {

    if (i == scan) {
      continue;
    }

    x2 = max(0, x + static_cast<int>((i + 1) * w / (SCREEN_WIDTH - 1)));
    y2 = min(h + 1, max(0, y + static_cast<int>(((static_cast<float>(ac_signal_data[i + 1]) - min_value) / yRange) * h)));

    u8g.drawLine(x1, y1, x2, y2);

    x1 = x2;
    y1 = y2;

  }

  //u8g.drawPixel(scan,16);

}
uint8_t framecount = 0;

// reset average bpm estimation
void clear_avgs(){
  for(uint8_t i = 0; i < N_AVG_SAMPLES; i++){
    bpms[i] = 80.0f;
  }
}

void update_display()
{
  framecount=(framecount+1)%256;
  
  // draw boundary
  uint8_t x = 0;
  uint8_t y = 0;
  uint8_t w = 128;
  uint8_t h = 26;

  // draw ir data
  plot(ac_signal_data, x, y, w, h, finger_down);
  
  //print_heart_char();
  //printAt<String>(8,12,s);

  // calculate avg bpm
  avgBpm = 0;
  for (uint8_t i = 0; i < N_AVG_SAMPLES; i++) {

#ifdef DEBUG
    Serial.print(bpms[i]);
    Serial.print(" ");
#endif

    avgBpm += bpms[i];
  }
  
#ifdef DEBUG
  Serial.println();
#endif

  avgBpm /= N_AVG_SAMPLES;

  u8g.setFont(u8g_font_7x13);
  //printAt<float>(128-20,12,beatEst);
  printAt(0, 64 - 10, "BPM");

  if(n_avgs > 4){
    printAt<uint8_t>(32, 64 - 10, avgBpm);    
  }else{
    printAt(32, 64 - 10, "--");    
  }

  float px = 0.0f;
  float py = 128.0f;

  int min = -100;
  int max = 100;

  int minVal = -100;
  int maxVal = -100;

  // drawing chart data
  for (int i = 0; i < 16; i++) {

    float x = i;
    float y = 18;

    float yRange = 100;
    x = static_cast<int>(i * 8 * 128 / (SCREEN_WIDTH - 1));
    y = 64 - static_cast<int>(((static_cast<float>(bpms[i]) - 25) / yRange) * 32);

    if (y > min) {
      min = y;
      minVal = bpms[i];
    }

    if (y < max) {
      max = y;
      maxVal = bpms[i];
    }

    px = x;
    py = y;

  }

}

uint8_t ir_value_diff = 0;
uint8_t prev_ir_value_diff = 0;
int32_t irAvgReg = 0;
int16_t IRAE = 0;

bool first = true;

void loop()
{

  u8g.firstPage();
  do {

    prev_irValue = irValue;

    // get sensor data
    irValue = particleSensor.getIR();
    redValue = particleSensor.getRed();
    
    int ok = digitalRead(OK_BUTTON);
    int left = digitalRead(LEFT_BUTTON);
    int right = digitalRead(RIGHT_BUTTON);

    // rotate right
    if (left == 0 && right == 1 && prev_left == 1 && prev_right == 1) {
      s = ">";
      max_value += 25;
      min_value -= 25;
    }

    // rotate left
    if (left == 1 && right == 0 && prev_left == 1 && prev_right == 1) {
      s = "<";
      max_value -= 25;
      min_value += 25;
    }
    if (ok == 0) {
      s = "O";
      max_value = -100;
      min_value = 100;
      n_avgs = 0;
      
      // clear out average
      clear_avgs();
    }

    prev_left = left;
    prev_right = right;

    // get fingerdown
    finger_down = irValue > THRESHOLD;

    // update ir scan data
    ac_signal_data[scan] = get_AC_Signal_Current(irValue);
    scan = (scan + 1) % SAMPLE_LENGTH;
    
    switch (finger_down) {

      // no finger detected
      case 0: {
 #ifdef DEBUG
          Serial.println("No finger");
 #endif
          first = false;

          PORTD &= ~0x16;

          first = true;

          timer++;
          
          // activate sleep when time
          if (timer == TIMEOUT) {

            splash("zzz", 64, 5000);
            sleep();

            timer = 0;
          }

          break;

        }

      // when finger is placed
      case 1: {
#ifdef DEBUG
          Serial.println("finger detected");
#endif
          // restart finger time
          if (first) {
            first = false;
            timer = 0;
            n_avgs = 0;
          }
          
          // on heart beat - from SparkFun MAX library
          if (checkForBeat(irValue)) {
            //Heart LED on
            PORTD |= 0x16; 
            
            // get beat timing
            long delta = millis() - lastBeat;
            lastBeat = millis();

            // estimate bpm
            beatEst = 60 / (delta / 1000.0);

            // add estimated bpm to average list
            addBpmToAverage(beatEst);
            n_avgs++;
            if(n_avgs >= 255){
              n_avgs = 4;
            }

          } else {
            // no beat
            //Heart LED off
            PORTD &= ~0x16;
          }

          break;

        }
      default: {
         
          //Heart LED off
          PORTD &= ~0x16;
          break;

        }
    }

    // this function is pretty slow
    update_display();

  } while ( u8g.nextPage() );

  // rebuild the picture after some delay
  // delay(0);
}

void addBpmToAverage(int avg) {
  bpms[avg_scan] = avg;
  avg_scan = (avg_scan + 1) % N_AVG_SAMPLES;
}
