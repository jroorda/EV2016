#include <Encoder.h>

#define APPROACH_DIST 3000  //When it will start Braking
#define APPROACH_POWER 65   //Ammount of power it uses to get going
#define FINAL_COAST_DIST 100  //When it will turn off the motor
#define DEFAULT_DIST 24932   //What the counter starts at

#define BREAK_TIME 16000    //Speed that trigger braking
#define APPROACH_TIME 20000  //Target speed of the approach
#define BREAK_POWER 90      //Power used to brake
//24932 old value

// 4133 wheel counts per meter
// wheelCounts is the real number of  counts traveled
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// If using software SPI (the default case):
#define OLED_MOSI   9
#define OLED_CLK    8
#define OLED_DC    A3
#define OLED_CS    A2
#define OLED_RESET A1
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

long counts = DEFAULT_DIST;
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

  Serial1.begin(115200);
  Serial1.print(1);

}

void loop() {
  encoderState = 0;
  lastEncoderState = 0;
  wheelCounts = 0;
  backward = false;
  error = 0;
  myEnc.write(0);

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
      Serial1.println("Start");
      delay(500);
      break;
    }
  }

  //Run Stage
  wheelCounts = myEnc.read();
  updateMotorSpeed(255, false);
  while ((wheelCounts + APPROACH_DIST) < counts) {
    wheelCounts = myEnc.read();
    if (digitalRead(startButton)) {
      wheelCounts = 0;
      break;
    }
  }

  //Approach Stage
  wheelCounts = myEnc.read();

  boolean reverseAllowedMaster = true;
  boolean reverseAllowed = true;
  boolean lastStateBrake = false;
  unsigned long timediff = 0;
  unsigned long timediffTest = 0;
  unsigned long savedtime = 0;
  savedtime = micros();
  boolean onCount = false;
  boolean firstRun = false;
  int fastCoastCount = 0;

  while ((wheelCounts + FINAL_COAST_DIST) < counts) {
    wheelCounts = myEnc.read();

    //Determine speed
    //See if wheelCount has the correct ending bytes
    if (!(wheelCounts & 0x0F)) {
      //Find the time difference
      if (!onCount) {
        timediffTest = micros() - savedtime;
        savedtime = micros();
        if (timediffTest > 500) {
          timediff = timediffTest;
        }
        onCount = true;
        Serial1.println(timediff);
        firstRun = true;
      }
    } else {
      onCount = false;
    }
    if (micros() - savedtime > 800000) {
      timediff = micros() - savedtime;
      savedtime = micros();
      Serial1.println(timediff);
      firstRun = true;
    }

    if (firstRun) {
      //Brake
      if (timediff <= BREAK_TIME && reverseAllowed && reverseAllowedMaster) {
        updateMotorSpeed(BREAK_POWER, true);
        fastCoastCount = 0;
        lastStateBrake = true;
        Serial1.print("B");
      }

      //Coast
      else if (timediff <= APPROACH_TIME ) {
        updateMotorSpeed(0, false);
        reverseAllowed = false;
        lastStateBrake = false;
        if (timediff <= BREAK_TIME) {
          if (fastCoastCount++ > 3) {
            reverseAllowed = true;
          }
        }
        Serial1.print("C");
      }

      //Power
      else {
        if (lastStateBrake) {
          updateMotorSpeed(0, false);
          reverseAllowed = false;
          lastStateBrake = false;
          Serial1.print("c");
        } else {
          updateMotorSpeed(APPROACH_POWER, false);
          reverseAllowed = false;
          reverseAllowedMaster = false;
          Serial1.print("P");
        }
      }
    }

    firstRun = false;
    if (digitalRead(startButton)) {
      wheelCounts = 0;
      break;
    }
  }

  //Final Stage
  updateMotorSpeed(0, true);

  //Display Stage
  display.clearDisplay();
  while (true) {
    wheelCounts = myEnc.read();
    display.clearDisplay();
    testdrawchar(wheelCounts);
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






