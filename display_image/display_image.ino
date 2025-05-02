#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
//#include <Arduino_FreeRTOS.h>
#include <string>

using namespace std;

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define SCREEN_SIZE 8192

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

uint16_t x_cord;
uint16_t y_cord;
uint8_t bitmap_arr[SCREEN_SIZE];
uint8_t mask = 128; //10000000
uint8_t msg;
uint8_t pixel;
int msg_len;

static unsigned char lookup[16] = {
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };

uint8_t reverse(uint8_t n) {
   // Reverse the top and bottom nibble then swap them.
   return (lookup[n&0b1111] << 4) | lookup[n>>4];
}

void disp_write(const char *str) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(str);
  display.display();
}

void read_serial() {
  static int i = 0;
  while (Serial.available()) {
    msg = (uint8_t)Serial.read();

    msg_len = 8;
    
    for (int j = 0; j < msg_len; j++) {
        pixel = (msg&128);
        msg = msg << 1;
        bitmap_arr[i+j] = pixel;
        
        /*
        x_cord = (uint16_t)(i+j)%SCREEN_WIDTH;
        y_cord = (uint16_t)(i+j)/SCREEN_WIDTH;
        display.drawPixel(x_cord, y_cord, pixel);
        */

        /*
        display.clearDisplay();
        display.setCursor(0, 0);
        display.printf("%i\n%i\n%i", pixel, x_cord, y_cord);
        display.display();
        */
        
      //if ((i+j)%128 == 0) display.display();
      if ((i+j) >= SCREEN_SIZE) {
        write_image();
        display.display();
        i = 0;
        break;
      }
    }
    i += msg_len;
  }
}

void write_image() {
  display.clearDisplay();
  display.drawRamBitmap(0, 0, SCREEN_HEIGHT, SCREEN_WIDTH, 1, bitmap_arr, SCREEN_SIZE);
  display.display();
}

void Task(void *Parameters) {
  
}
*/
void setup() {
  Serial.begin(115200);
  //delay(2000);
  //Serial.println("Starting setup");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  /*xTaskCreate(
    functionhandle //function handle
    , "stuff"
    , 128    // stack size
    , &void  // parameters
    , 3      // high priority
    , NULL); // task handle
*/


  delay(1000);

  //Serial.println("setup done");
  display.setTextColor(WHITE);
  display.setTextSize(1);
  disp_write("ready");
}

void loop() {
  if (Serial.available()) {
    read_serial();
  }
}