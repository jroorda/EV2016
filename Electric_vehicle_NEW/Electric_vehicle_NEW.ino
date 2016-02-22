#include <Encoder.h>


// 4133 wheel counts per meter
// wheelCounts is the real number of  counts traveled
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// If using software SPI (the default case):
#define OLED_MOSI   9
#define OLED_CLK    8
#define OLED_DC    16
#define OLED_CS    14
#define OLED_RESET 15
#define Button_Pin 7
#define Pot_Pin A0
Encoder myEnc(2, 3);
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);


#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000
};


#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

long counts = 24932;
char stringCounts[8];
// end of OLED Setup


//const int encYellow = 2;
//const int encWhite = 3;
const int startButton = 10;
const int motorInA = 4;
const int motorInB = 5;
const int motorpwm = 6;

byte encoderState, lastEncoderState;
long wheelCounts;
bool backward;
int motSpeed;
int error;
long breakTime;

long startTime;
long stopTime;
boolean startTimeSaved;
boolean stopTimeSaved;

void setup() {
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  // Clear the buffer.
  display.clearDisplay();
  testdrawchar(counts);
  delay(200);

  pinMode(Pot_Pin, INPUT);
  pinMode(Button_Pin, INPUT);
  digitalWrite(Button_Pin, HIGH);
  // end OLED setup

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(startButton, INPUT);
  pinMode(motorInA, OUTPUT);
  pinMode(motorInB, OUTPUT);
  pinMode(motorpwm, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(startButton, HIGH);
  digitalWrite(motorInA, LOW);
  digitalWrite(motorInB, LOW);
  digitalWrite(motorpwm, LOW);

  digitalWrite(motorInB, LOW);
  digitalWrite(motorInA, HIGH);

  Serial.begin(115200);
  Serial.print(1);

  startTimeSaved = false;
  stopTimeSaved = false;
}

void loop() {
  encoderState = 0;
  lastEncoderState = 0;
  wheelCounts = 0;
  backward = false;
  error = 0;
  myEnc.write(0);
  startTimeSaved = false;
  stopTimeSaved = false;

  //Ready Stage
  while (true) {
    if (!digitalRead(Button_Pin)) {
      long inVal = analogRead(Pot_Pin);
      long xPos = inVal * display.width() / 1024;
      inVal = (inVal / 10) - 20;
      if (inVal < 0) {
        inVal = inVal / 4;
      }
      counts = counts + inVal;
      display.clearDisplay();
      testdrawchar(counts);
      display.drawLine(xPos, display.height() - 1, xPos, display.height() - 10, WHITE);
      display.drawLine(22, display.height() - 1, 22, display.height() - 10, WHITE);
      display.display();
      //delay(100);
    } else {
      long inVal = analogRead(Pot_Pin);
      long xPos = inVal * display.width() / 1024;
      display.clearDisplay();
      testdrawchar(counts);
      display.drawLine(xPos, display.height() - 1, xPos, display.height() - 10, WHITE);
      display.drawLine(22, display.height() - 1, 22, display.height() - 10, WHITE);
      display.display();
    }
    if (digitalRead(startButton)) {
      delay(500);
      break;
    }
  }

  //Run Stage
  wheelCounts = myEnc.read();
  updateMotorSpeed(255, backward);
  while ((wheelCounts + 1385) < counts) {
    wheelCounts = myEnc.read();
    if (digitalRead(startButton)) {
      wheelCounts = 0;
      break;
    }
    if (!startTimeSaved) {
      if (wheelCounts > 1385) {
        startTime = millis();
        startTimeSaved = true;
      }
    } else if (!stopTimeSaved) {
      if (wheelCounts > 23546) {
        stopTime = millis();
        stopTimeSaved = true;
      }
    }
  }

  if (!stopTimeSaved) {
    stopTime = millis();
  }
  //Approach Stage
  breakTime = millis();
  updateMotorSpeed(0, backward);
  delay(1000);




  //Wait Stage
  updateMotorSpeed(0, backward);
  delay(1000);
  wheelCounts = myEnc.read();

  //Final Stage
  bool goDirection = true;
  long targetStop = 0;
  targetStop = setTargetStop(counts, wheelCounts);
  if (wheelCounts > targetStop) {
    goDirection = false;
  }
  while (true) {
    wheelCounts = myEnc.read();

    //Forwards
    if (goDirection) {
      if (wheelCounts > targetStop) {
        updateMotorSpeed(0, false);
        delay(500);
        wheelCounts = myEnc.read();
        targetStop = setTargetStop(wheelCounts, counts);
        if (wheelCounts > targetStop) {
          goDirection = false;
        }
        if (abs(wheelCounts - targetStop) < 10) {
          break;
        }
      } else {
        updateMotorSpeed(75, false);
      }
    } else {
      //Backwards
      if (wheelCounts < targetStop) {
        updateMotorSpeed(0, true);
        delay(500);
        wheelCounts = myEnc.read();
        targetStop = setTargetStop(wheelCounts, counts);
        if (abs(wheelCounts - targetStop) < 10) {
          break;
        }
        if (wheelCounts < targetStop) {
          goDirection = true;
        }
      } else {
        updateMotorSpeed(75, true);
      }
    }
  }

  //Display Stage
  display.clearDisplay();
  testdrawchar(stopTime-startTime);
  display.display();
  while(true){
    if (digitalRead(startButton)) {
      delay(500);
      break;
    }
  }
}

long setTargetStop(long counts, long target) {
  long error = counts - target;
  return (counts - (error / 2));
}

void testdrawchar(long counts) {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  ltoa(counts, stringCounts, 10);

  for (uint8_t i = 0; i < 8; i++) {
    display.write(stringCounts[i]);
  }
  display.display();
  //Serial.print(1);
  //end of OLED Loop
}


void updateMotorSpeed(int motorSpeed, bool motorDirection) {
  if (motorDirection) {
    digitalWrite(motorInA, LOW);
    digitalWrite(motorInB, HIGH);
  } else {
    digitalWrite(motorInB, LOW);
    digitalWrite(motorInA, HIGH);
  }
  analogWrite(motorpwm, motorSpeed);
}






