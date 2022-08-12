// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"

RTC_DS3231 rtc;

#include <SdFat.h>
SdFat SD;
SdFile dataFile;

SdFile configFile;


// directory & file
char filename[] = "/DDMMYYYY/hhmmss.log";
char dirname[] = "/DDMMYYYY";

// sd spi cs
const byte chipSelect = 4;

// serial com
#define BAUD_RATE  460800
#define SERIAL2_RX 16
#define SERIAL2_TX 17

// Define packet size, buffer and buffer pointer for SD card writes
const size_t SDPACKET = 512;
const size_t BUFFERSIZE = 206; // n buffer with 512 bytes of SD packet

struct BUFFER {
  bool filled;
  uint8_t buf[SDPACKET];
};

// ****
BUFFER myBuffer[BUFFERSIZE];
uint8_t activeBuffer = 0;
uint16_t activeBufferPointer = 0;
bool overRun = false;

// led log indicator
const int output = LED_BUILTIN;

volatile byte stop_pressed = 0; // Flag to indicate if stop switch was pressed to stop logging

const byte interruptPin = 21;

void stop_log();

unsigned long currentMillis;  // Used for storing the latest time
unsigned long previousMillis = 0;      // Store the last time the loop ran
const unsigned long interval = 20;     // Sample frequency (milliseconds)

void store(void) {
  if (Serial2.available() > 0) {
    //Serial.println(F("logging..."));
    digitalWrite(LED_BUILTIN, HIGH); // Turn the red LED on to indicate SD card write

    myBuffer[activeBuffer].buf[activeBufferPointer++] = Serial2.read();

    if (activeBufferPointer >= SDPACKET) {
      activeBufferPointer = 0;
      myBuffer[activeBuffer++].filled = true;
      activeBuffer %= BUFFERSIZE;
      if (myBuffer[activeBuffer].filled) { // overrunning buffer die!
        overRun = true;
      }
    }
  }
}

static uint8_t nextBuf = 0;

void flushbuffer() {
  if (myBuffer[nextBuf].filled) { //check to see if buffer is filled
    dataFile.write(myBuffer[nextBuf].buf, SDPACKET); // flush it
    myBuffer[nextBuf].filled = false; // mark it clear
    nextBuf = (nextBuf + 1) % BUFFERSIZE; // next buffer in order.
  }
}

void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), stop_log, FALLING);

  // Serial port for debugging purposes
  Serial.begin(115200);

  //Serial2.setRxBufferSize(SERIAL_SIZE_RX);

  pinMode(output, OUTPUT);
  digitalWrite(output, LOW);

  // RTC setup
  Serial.println(F("Initializing RTC ..."));
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    Serial.println(F("Note: press reset or reopen this serial monitor after fixing your issue!"));
    while (true);
  }

  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, let's set the time!"));
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println(F("RTC initialization done."));

  // SD card setup
  Serial.println(F("Initializing SD card ..."));
  // See if the SD card is present and can be initialized
  if (!SD.begin(chipSelect, SPI_HALF_SPEED)) {
    Serial.println(F("Card Mount Failed"));
    Serial.println(F("Note: press reset or reopen this serial monitor after fixing your issue!"));

    while (true);
  }

  Serial.println(F("SD card initialization done."));

  // try open config file
  if (configFile.open("config.txt")) {
    char line[10];
    int rate;
    configFile.fgets(line, sizeof(line));
    rate = atoi(line);
    Serial.print("Using baud rate ");    
    Serial.println(rate);
    Serial2.begin(rate, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX); // configure baud rate and start serial
  } else {
    Serial.println("Using default baud rate 460800");
    Serial2.begin(BAUD_RATE, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX); // configure baud rate and start serial
  }
  
  // Get the RTC time and date
  DateTime now = rtc.now();

  now.toString(filename);
  now.toString(dirname);

  // try to create subdirectory (even if it exists already)
  if (!SD.mkdir(dirname)) {
    Serial.println(F("Create dir failed"));
  }

  // Open the log file
  if (dataFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
    Serial.print(F("Logging to "));
    Serial.println(filename);
  } else {
    Serial.println(F("Error opening log file!"));
    Serial.println(F("Waiting for reset..."));

    // don't do anything more:
    while (1);
  }

  Serial.println(F("logging..."));
  Serial.end();
}

void loop() {
  currentMillis = millis();

  store(); // read serial and if data save it to buffer

  if ((currentMillis - previousMillis) >= interval) {
    flushbuffer(); // flush buffer to SD if filled, inc to next buffer
    previousMillis = currentMillis;
  }

  if (overRun || stop_pressed == 1) { // stop Logging if overrun or we use the interrupt to stop the logging
    while (myBuffer[nextBuf].filled) flushbuffer(); // save all captured data

    dataFile.close();
    digitalWrite(LED_BUILTIN, LOW); // turn red LED off
    while (true);
  }
}

void stop_log() {
  stop_pressed = 1;
}
