#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <menu.h>
#include <menuIO/lcdOut.h>
#include <menuIO/encoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>
#include <EEPROM.h>
using namespace Menu;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define encA 3
#define encB 2
#define encBtn 9

#define CONFIG_VERSION "VER01"
#define CONFIG_START 32

typedef struct {
  char version[6];
  unsigned int numInjectors;
  unsigned int workTime;
  int injMode;
  unsigned int RPM;
  int fireMode;
  unsigned int duty;
} configuration_type;

enum fireModes {
  BATCH = 1,
  SEQUENTIAL = 2
};

enum injectionModes {
  CRANK_360 = 1,
  CAM_720 = 2
};

enum injectionStates {
  LOW_RPM,
  HIGH_RPM,
  MANUAL,
  LEAK,
  STOP
};

configuration_type FACTORY_CONFIGURATION = {
  CONFIG_VERSION,
  4,
  1,
  CAM_720,
  1000,
  BATCH,
  50
};

// injector lab variables
unsigned int numInjectors = FACTORY_CONFIGURATION.numInjectors;
unsigned int workTime = 1;
enum injectionModes injMode = CAM_720;
enum fireModes fireMode = BATCH;
unsigned int RPM = 1000;
unsigned int duty = 50;

configuration_type CONFIGURATION = {
  CONFIG_VERSION,
  numInjectors,
  workTime,
  injMode,
  RPM,
  fireMode,
  duty
};

enum injectionStates actualState = STOP;

encoderIn<encA,encB> encoder;
#define ENC_SENSIVITY 4
encoderInStream<encA,encB> encStream(encoder, ENC_SENSIVITY);

keyMap encBtn_map[] = {{-encBtn, defaultNavCodes[enterCmd].ch}};
keyIn<1> encButton(encBtn_map);

menuIn* inputsList[] = {&encStream, &encButton};
chainStream<2> in(inputsList);

byte injectorChar[] = {
  B01010,
  B01110,
  B11111,
  B11111,
  B01110,
  B01110,
  B01110,
  B00100
};

unsigned long onTime = 1000UL;
unsigned long offTime = 1000UL;

// injectors pins
const byte INJ1_PIN = 5;
const byte INJ2_PIN = 6;
const byte INJ3_PIN = 7;
const byte INJ4_PIN = 8;

// injectors times
unsigned long inj1uSec;
unsigned long inj1LastuSec;
unsigned long inj2uSec;
unsigned long inj2LastuSec;
unsigned long inj3uSec;
unsigned long inj3LastuSec;
unsigned long inj4uSec;
unsigned long inj4LastuSec;

// injectors sequential times
unsigned long injSequSec;
unsigned long injSeqLastuSec;

boolean inj1State;
boolean inj2State;
boolean inj3State;
boolean inj4State;

// time counters
unsigned long lastWorkingTime = -1;
unsigned long lastTime1SecRunning = 0;
unsigned long lastTime2SecRunning = 0;
unsigned long injSequentialDelay = 0;
unsigned int sequentialInjectorFire = 1;
unsigned int sequentialCycleCount = 0;
unsigned int countdown = 0;
boolean isWorking = false;
boolean isSaveConfig = false;

unsigned int leakTestDuty[5] = {30, 50, 40, 60, 80};
unsigned int leakTestRPM[5] = {800, 1600, 3200, 4000, 6000};
unsigned int leakTestCounter = 0;

// function declararion
void setOutputsToLow(void);
int loadConfig();
void saveConfig();
void pulseInj1(void);
void pulseInj2(void);
void pulseInj3(void);
void pulseInj4(void);
long calculateInjectorOpenTime_us(int rpm, int dc);
long calculateCycleTime_us(int rpm);
void calculateInjectionsTimes(int rpm, int dc);
void timeAndCharOnScreen(long seconds);
void updateSequentialCycleCount();
result saveSettings(eventMask e, prompt &item);
result setupLowRPM(eventMask e, prompt &item);
result setupHighRPM(eventMask e, prompt &item);
result setupManualTest(eventMask e, prompt& item);
result setupLeakTest(eventMask e, prompt& item);

SELECT(injMode, injModeMenu, "Mode", saveSettings, anyEvent, noStyle
  ,VALUE("360", CRANK_360, doNothing, noEvent)
  ,VALUE("720", CAM_720, doNothing, noEvent)
);

SELECT(fireMode, injTypeMenu, "Type", saveSettings, anyEvent, noStyle
  ,VALUE("Batch", BATCH, doNothing, noEvent)
  ,VALUE("Sequential", SEQUENTIAL, doNothing, noEvent)
);

MENU(subMenuSettings, "Settings", doNothing, anyEvent, showTitle
  ,FIELD(numInjectors, "# Injectors", "", 1, 4, 1, 0, saveSettings, anyEvent, wrapStyle)
  ,SUBMENU(injTypeMenu)
  ,SUBMENU(injModeMenu)
  ,EXIT("Back")
);

MENU(subMenuLowRPM, "Idle", doNothing, anyEvent, showTitle
  ,FIELD(workTime, "Duration"," min", 1, 60, 1, 0, doNothing, noEvent, wrapStyle)
  ,OP("Start", setupLowRPM, enterEvent)
  ,EXIT("Back")
);

MENU(subMenuHighRPM, "On Load", doNothing, anyEvent, showTitle
  ,FIELD(workTime, "Duration"," min", 1, 60, 1, 0, doNothing, noEvent, wrapStyle)
  ,OP("Start", setupHighRPM, enterEvent)
  ,EXIT("Back")
);

MENU(subMenuLeakTest, "Leak Test", doNothing, anyEvent, showTitle
  ,OP("Start", setupLeakTest, enterEvent)
  ,EXIT("Back")
);

MENU(subMenuManualTest, "Manual Test", doNothing, anyEvent, showTitle
  ,FIELD(RPM, "RPM","", 500, 8000, 50, 0, saveSettings, anyEvent, wrapStyle)
  ,FIELD(workTime, "Duration"," min", 1, 60, 1, 0, doNothing, noEvent, wrapStyle)
  ,FIELD(duty, "Duty" ," %", 1, 90, 1, 0, saveSettings, anyEvent, wrapStyle)
  ,OP("Start", setupManualTest, enterEvent)
  ,EXIT("Back")
);

MENU(mainMenu, "Main menu", doNothing, anyEvent, noStyle
  ,SUBMENU(subMenuLowRPM)
  ,SUBMENU(subMenuHighRPM)
  ,SUBMENU(subMenuLeakTest)
  ,SUBMENU(subMenuManualTest)
  ,SUBMENU(subMenuSettings)
);

#define MAX_DEPTH 5
MENU_OUTPUTS(out, MAX_DEPTH, LCD_OUT(lcd, {0, 0, 20, 4}), NONE);
NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);//the navigation root object

result working(menuOut& o, idleEvent e) {
  switch(e) {
    case idleStart:
      lastWorkingTime = 0;
      isWorking = true;      
      break;
    case idling:
      o.setCursor(5, 0);
      o.print("Working...");
      o.setCursor(7, 3);
      o.print("[stop]");
      break;
    case idleEnd:
      isWorking = false;
      actualState = STOP;
      setOutputsToLow();
      break;
  }
  return proceed;
}

result setupLowRPM(eventMask e, prompt& item) {
  countdown = workTime * 60;
  calculateInjectionsTimes(800, FACTORY_CONFIGURATION.duty);
  nav.idleOn(working);
  return proceed;
}

result setupHighRPM(eventMask e, prompt& item) {
  countdown = workTime * 60;
  calculateInjectionsTimes(7500, FACTORY_CONFIGURATION.duty);
  nav.idleOn(working);
  return proceed;
}

result saveSettings(eventMask e, prompt& item) {
  if (e == exitEvent) {
    isSaveConfig = true;
  }
  return proceed;
}

result setupManualTest(eventMask e, prompt& item) {
  countdown = workTime * 60;
  calculateInjectionsTimes(RPM, duty);
  nav.idleOn(working);
  return proceed;
}

result setupLeakTest(eventMask e, prompt& item) {
  onTime = 4 * 1000UL;
  offTime = 4 * 1000UL;
  countdown = 10;
  actualState = LEAK;
  nav.idleOn(working);
  return proceed;
}

void setup() {
  pinMode(encBtn, INPUT_PULLUP);
  pinMode(INJ1_PIN, OUTPUT);
  pinMode(INJ2_PIN, OUTPUT);
  pinMode(INJ3_PIN, OUTPUT);
  pinMode(INJ4_PIN, OUTPUT);

  Serial.begin(115200);
  while(!Serial);

  if (loadConfig()) {
    numInjectors = CONFIGURATION.numInjectors;
    workTime = CONFIGURATION.workTime;
    injMode = (injectionModes) CONFIGURATION.injMode;
    RPM = CONFIGURATION.RPM;
    fireMode = (fireModes) CONFIGURATION.fireMode;
    duty = CONFIGURATION.duty;
  } else {
    saveConfig(); // overwrite with the default settings
  }
  
  encoder.begin();
  lcd.begin(20, 4);
  lcd.createChar(0, injectorChar);
  nav.showTitle = false;
  lcd.setCursor(4, 0);
  lcd.print("Injectrino");
  lcd.setCursor(2, 1);
  lcd.print("DVGK Motorsports");
  delay(2500);
  lcd.clear();
}

void loop() {
  unsigned long currentMillis = millis();
  nav.poll();

  if (isSaveConfig) {
    CONFIGURATION.numInjectors = numInjectors;
    CONFIGURATION.workTime = workTime;
    CONFIGURATION.injMode =injMode;
    CONFIGURATION.RPM = RPM;
    CONFIGURATION.fireMode = fireMode;
    CONFIGURATION.duty = duty;
    saveConfig();
    isSaveConfig = false;
  }

  if (isWorking) {
    // 2 seconds timer
    if (currentMillis - lastTime2SecRunning >= (1000UL)) {
      if (actualState == LEAK) {
        calculateInjectionsTimes(leakTestRPM[leakTestCounter], leakTestDuty[leakTestCounter]);
        leakTestCounter++;
        if (leakTestCounter == 4) {
          leakTestCounter = 0;
        }
      }
      lastTime2SecRunning = currentMillis;
    }
    
    // 1 second timer
    if (currentMillis - lastTime1SecRunning >= (1000UL)) {
      countdown--;
      timeAndCharOnScreen(countdown);
      if (countdown == 0) {
        isWorking = false;
        lcd.clear();
        nav.idleOff();
        actualState = STOP;
      }
      lastTime1SecRunning = currentMillis;
    }

    inj1uSec = micros();
    inj2uSec = micros();
    inj3uSec = micros();
    inj4uSec = micros();
    
    if (fireMode == BATCH) {
      pulseInj1();
      pulseInj2();
      pulseInj3();
      pulseInj4();
    } 

    if (fireMode == SEQUENTIAL) {
      switch (sequentialInjectorFire) {
        case 1:
          pulseInj1();
          if (sequentialCycleCount > 1) {  
            sequentialInjectorFire = 2;
            sequentialCycleCount = 0;
          }
          break;
        case 2:
          pulseInj2();
          if (sequentialCycleCount > 1) {  
            sequentialInjectorFire = 3;
            sequentialCycleCount = 0;
          }
          break;
        case 3:
          pulseInj3();
          if (sequentialCycleCount > 1) {  
            sequentialInjectorFire = 4;
            sequentialCycleCount = 0;
          }
          break;
        case 4:
          pulseInj4();
          if (sequentialCycleCount > 1) {  
            sequentialInjectorFire = 1;
            sequentialCycleCount = 0;
          }
          break;
      }
    }
  }
}

void pulseInj1(void) {
  if ((numInjectors > 0) && (inj1uSec - inj1LastuSec > (inj1State ? onTime : offTime))) {
    digitalWrite(INJ1_PIN, inj1State = !inj1State);
    updateSequentialCycleCount();
    inj1LastuSec = inj1uSec;
  }
}

void pulseInj2(void) {
  if ((numInjectors > 1) && (inj2uSec - inj2LastuSec > (inj2State ? onTime : offTime))) {
    digitalWrite(INJ2_PIN, inj2State = !inj2State);
    updateSequentialCycleCount();
    inj2LastuSec = inj2uSec;
  }
}

void pulseInj3(void) {
  if ((numInjectors > 2) && (inj3uSec - inj3LastuSec > (inj3State ? onTime : offTime))) {
    digitalWrite(INJ3_PIN, inj3State = !inj3State);
    updateSequentialCycleCount();
    inj3LastuSec = inj3uSec;
  }
}

void pulseInj4(void) {
  if ((numInjectors > 3) && (inj4uSec - inj4LastuSec > (inj4State ? onTime : offTime))) {
    digitalWrite(INJ4_PIN, inj4State = !inj4State);
    updateSequentialCycleCount();
    inj4LastuSec = inj4uSec;
  }
}

void updateSequentialCycleCount() {
  if (fireMode == SEQUENTIAL) {
    sequentialCycleCount++;
  }
}

void setOutputsToLow() {
  digitalWrite(INJ1_PIN, LOW);
  digitalWrite(INJ2_PIN, LOW);
  digitalWrite(INJ3_PIN, LOW);
  digitalWrite(INJ4_PIN, LOW);
  inj1State = false;
  inj2State = false;
  inj3State = false;
  inj4State = false;
}

int loadConfig() {
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
    EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
    EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2] &&
    EEPROM.read(CONFIG_START + 3) == CONFIG_VERSION[3]) {
      for (unsigned int i = 0; i < sizeof(CONFIGURATION); i++) { 
        * ((char * ) & CONFIGURATION + i) = EEPROM.read(CONFIG_START + i);
      }
      return 1;
    }
  return 0;
}

void saveConfig() {
  for (unsigned int i = 0; i < sizeof(CONFIGURATION); i++) {
    EEPROM.write(CONFIG_START + i, * ((char * ) & CONFIGURATION + i));
  }
}

void calculateInjectionsTimes(int rpm, int dc) {
  unsigned long cycleTime = calculateCycleTime_us(rpm);
  unsigned long injectorOpenTime = calculateInjectorOpenTime_us(rpm, dc);
  unsigned long injectorCloseTime = cycleTime - injectorOpenTime;  
  onTime = injectorOpenTime;
  offTime = injectorCloseTime;
  injSequentialDelay = cycleTime / numInjectors;
}

long calculateCycleTime_us(int rpm) {
  float t = 60.0 / ( (float) rpm / injMode);
  return (long) (t * 1000000L);
}

long calculateInjectorOpenTime_us(int rpm, int dc) {
  return (dc * calculateCycleTime_us(rpm)) / 100;
}

void timeAndCharOnScreen(long seconds) {
  long counter = seconds;

  int hours = counter / 3600;
  counter -= (hours * 3600);
  int mins = counter / 60;
  counter -= (mins * 60);
  int secs = counter;

  // show active injectors on screen
  if (numInjectors > 0) {
    lcd.setCursor(8, 1);
    lcd.print(char(0));
  }
  if (numInjectors > 1) {
    lcd.setCursor(9, 1);
    lcd.print(char(0));
  }
  if (numInjectors > 2) {
    lcd.setCursor(10, 1);
    lcd.print(char(0));
  }
  if (numInjectors > 3) {
    lcd.setCursor(11, 1);
    lcd.print(char(0));
  }

  char buf[100];
  snprintf(buf, 14, "        %0i:%0i", mins, secs);
  lcd.setCursor(0, 2);
  lcd.print(buf);
}