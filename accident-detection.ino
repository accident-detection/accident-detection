/*DOCUMENTATION:
1. Includes | Defines |Globals
2. Loop and main
3. Setups
4. Polling
*/


/*------------------------------------------------------------------------------------------------------
1. 1.Includes | 2.Defines | 3.Globals
--------------------------------------------------------------------------------------------------------
*/

//1.1. Includes
//1.1.100 GPS Includes
#include <TinyGPS++.h> // Library for GPS
#include <SoftwareSerial.h> // For additional serial communication
#include <SPI.h> // (Serial Peripheral Interface) for SD card
#include <SD.h> // For SD card

//1.1.200 AD Includes
#include "Wire.h"
#include "HMC5883L.h"

//1.1.600 Display Includes
#include <LiquidCrystal.h>

//1.2. Defines
//1.2.100 GPS Defines
#define pinRX 2 // TX and RX pins for GPS, connect TX-RX, RX-TX
#define pinTX 3

//1.2.200 AD Defines
#define triggerInputFrontAD 7 // Echo and trigger for front and back distance sensor
#define echoOutputFrontAD 6
#define triggerOutputBackAD 9
#define echoOutputBackAD 8
#define criticalDistanceAD 10  // Distance in centimeters for which the distance sensors react
#define criticalGyroUpAD 1.20  // Used in algorithm for determining accident
#define criticalGyroDownAD 0.80 // Changing these two coeficients - further from 1.0 = lower sensitivity, closer to 1.0 = higher sensitivity
#define cycleAD 10  // How often the loop polles systems other than the AD

//1.2.500 RTC Defines
#define i2cRTC 0x68 // I2C address of the RTC module

//1.2.600 Display Defines
#define cancelButton 37  // Pin with cancel button
#define pin1LCD 11  // LCD pins
#define pin2LCD 12
#define pin3LCD 13
#define pin4LCD 14
#define pin5LCD 15
#define pin6LCD 16


//1.3. Globals
//1.3.100 GPS Global
static const unsigned long baudGPS = 9600; // GPS works on 9600 baud
TinyGPSPlus gps; // Instance of TinyGPS
SoftwareSerial ss(pinRX, pinTX); // Serial connection with GPS module
File sdCardObject; // Variable for manipulating SD card

//1.2.300 Ethernet Define
SoftwareSerial net(17, 18); // Serial connection with ethernet module

//1.2.400 SD Define
int chipSelect = 4; // CS pin of the SD card

//1.3.200 AD Global
HMC5883L compassAD;  // Instance of the HMC object
float xNewAD, yNewAD, zNewAD;  // X,Y,Z coordinates used for gyroscope
float xOldAD, yOldAD, zOldAD;
int cycleGlobal = 0;  // Global cycle counter

//1.3.400 SD Global
char* txtFileName = "txtfile.txt";  // Files on SD card
char* csvFileName = "csvfile.csv";

//1.3.600 Display Global
LiquidCrystal lcd(pin1LCD, pin2LCD, pin3LCD, pin4LCD, pin5LCD, pin6LCD);


/*------------------------------------------------------------------------------------------------------
2. Setup and loop
--------------------------------------------------------------------------------------------------------
*/


//2.1. Setup
void setup() {
  Serial.begin(1200); // 9600 baud is used for GPS
  net.begin(19200);
  SetupSD();
  SetupGPS();
  SetupAD();
  pinMode(cancelButton, INPUT); // Button on the prototype which cancels sending data to server when accident happens
  cycleGlobal = 0; // Global cycle counter, used for determining system polling
}


//2.2. Loop
void loop() {
  //2.2.1. Locals
  int codeAD, codeNetwork, codeSD, codeLCD, codeGPS; // Return code of systems
  String dataRTC, dataGPS; // Return data of systems
  int statusWriteTXT = 0, statusWriteCSV = 0; // Variables which check if SD card failed
  int accidentCounter;  // Counts down 20 seconds on accident
  int buttonState;  // Checks if button is pressed
  bool accidentDetected;


  //2.2.2. Polling and decision based on codeAD
  codeAD = PollingAD();
  if (codeAD >= 200 && codeAD <= 204) { //Everything ok - polling other systems and writing to SD
    if (cycleGlobal %  cycleAD == 0) {
      // Polling other systems every N-th time to improve system speed
      dataRTC = PollingRTC();
      dataGPS = PollingGPS(&codeGPS);
      PollingLCD("Status: OK ", 11, (String)cycleGlobal);
      lcd.setCursor(0, 0);
      //Writing to SD
      writeCSVToSD(csvFileName, dataRTC, codeAD, dataGPS);
      if (codeGPS == 100)
        writeTXTToSD(txtFileName);

      //Writing to other arduino for ethernet test
      net.println(dataRTC + ";" + (String)codeAD + ";" + (String)codeGPS + ";" + dataGPS);
    }
  }
  else if (codeAD >= 205 && codeAD <= 207) {
    // Start counting down to 0
    accidentDetected = true;
    accidentCounter = 20;
    buttonState = LOW;
    // Write the last polling information to SD card
    writeCSVToSD(csvFileName, dataRTC, codeAD, dataGPS);

    // Start counting down 20 seconds
    ClearLCD();
    while (accidentCounter > 0) {
      accidentCounter--;
      PollingLCD("Sending in: ", 11, (String)accidentCounter);
      buttonState = digitalRead(cancelButton);
      if (buttonState == HIGH) {
        accidentDetected = false;
        writeCSVToSD(csvFileName, "User pressed button, sending to server canceled!");
        break;
      }
      delay(1000); // Delay to wait 20 seconds
      ClearLCD();
    }

    if (accidentDetected == true) {
      // If we enter in this part it means user did not cancel sending to server in 20 seconds
      writeCSVToSD(csvFileName, "Accident detected, information sent to server!");
      writeCSVToSD(csvFileName, dataRTC, codeAD, dataGPS);
      if (codeGPS == 100)
        writeTXTToSD(txtFileName);
      PollingLCD("Sent to server!");
      delay(5000);
    }

    ClearLCD();
  }

  //2.2.4. Cycle update
  if (cycleGlobal > 100)
    cycleGlobal = 10;
  else
    cycleGlobal++;
}

/*------------------------------------------------------------------------------------------------------
3. Setup sources
--------------------------------------------------------------------------------------------------------
*/

//3.100 - GPS Setup
void SetupGPS() {
  ss.begin(baudGPS);
}

//3.200 - AD Setup
void SetupAD() {
  Wire.begin();
  compassAD = HMC5883L();
  setupHMC5883L();
  pinMode(triggerInputFrontAD, OUTPUT);  //Setting up distance sensors
  pinMode(echoOutputFrontAD, INPUT);
  pinMode(triggerOutputBackAD, OUTPUT);
  pinMode(echoOutputBackAD, INPUT);;

  getHeading();
  xOldAD = xNewAD;
  yOldAD = yNewAD;
  zOldAD = zNewAD;
}

//3.400 - SDCard Setup
void SetupSD() {
  Serial.print("Trying to initialize SD card...");
  pinMode(10, OUTPUT); // Pin 10 used for SD module
  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed.");
    return;
  }
  Serial.println("Initialization completed.");
}


/*------------------------------------------------------------------------------------------------------
4. Polling sources
--------------------------------------------------------------------------------------------------------
*/

/*----------------------------------------
4.100 - GPS Polling
-----------------------------------------
*/
String PollingGPS(int* codeGPS) {
  // Printing data after every NMEA sentence
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      return getGPSData(codeGPS);
    }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10) // GPS not working
  {
    *codeGPS = 104;
    return "104;x;x;x;"; // Return error code
  }
  *codeGPS = 105;
  return "105;x;x;x;";
}
// Function for error print
String getGPSData(int* codeGPS) {
  bool locationIsValid = false;
  bool speedIsValid = false;

  // String which returns status
  String gpsDataString = "";
  String gpsDataCode = "";

  // Checking location and speed
  if (gps.location.isValid()) {
    locationIsValid = true;
    gpsDataString += (String)gps.location.lat() + ";" + (String)gps.location.lng() + ";";
  }
  else {
    gpsDataString += "x;x;";
  }

  if (gps.speed.isValid()) {
    speedIsValid = true;
    gpsDataString += (String)gps.speed.kmph() + ";";
  }
  else {
    gpsDataString += "x;";
  }

  if (locationIsValid && speedIsValid) {
    *codeGPS = 100;
    gpsDataCode = "100;";
  }
  else if (locationIsValid && !speedIsValid) {
    *codeGPS = 102;
    gpsDataCode = "102;";
  }
  else if (!locationIsValid && speedIsValid) {
    *codeGPS = 101;
    gpsDataCode = "101;";
  }
  else {
    *codeGPS = 103;
    gpsDataCode = "103;";
  }

  return gpsDataCode + gpsDataString;
}

/*----------------------------------------
4.200 - AD Polling
-----------------------------------------
*/
int PollingAD() {
  char accidentState = 0;
  AccidentDetector(&accidentState);
  switch (accidentState) {
    case '0':
      //200 - Everything ok
      return 200;
      break;
    case '1':
      //201 - back sensor reacted - object too close
      return 201;
      break;
    case '2':
      //202 - Front sensor reacted - object too close
      return 202;
      break;
    case '3':
      //203 - Both distance sensors reacted - objects on both sides too close
      return 203;
      break;
    case '4':
      //204 - Gyroscope reacted - significant XYZ axis change detected
      return 204;
      break;
    case '5':
      //205 - Back sensor reaction + XYZ change = Back hit detected
      return 205;
      break;
    case '6':
      //206 - Front sensor reaction + XYZ change = Front hit detected
      return 206;
      break;
    case '7':
      //207 - Both distance sensors reaction + XYZ change = Hit while surrounded from both sides
      return 207;
      break;
    default:
      //200 - Everything ok
      return 200;
      break;
  }
}
void AccidentDetector(char* state) {
  bool distanceFront, distanceBack;
  bool gyroWarning;
  //Polling Gyroscope and Distance sensors
  gyroWarning = ReadGyro();
  distanceFront = ReadDistanceFront();
  distanceBack = ReadDistanceBack();
  //Calculating new state
  if (gyroWarning == false) { //0XX
    if (distanceFront == false) { //00X
      if (distanceBack == false) //000
        *state = '0';
      else //001
        *state = '1';
    }
    else { //01X
      if (distanceBack == false) //010
        *state = '2';
      else //011
        *state = '3';
    }
  }
  else { //1XX
    if (distanceFront == false) { //10X
      if (distanceBack == false) //100
        *state = '4';
      else //101
        *state = '5';
    }
    else { //11X
      if (distanceBack == false) //110
        *state = '6';
      else //111
        *state = '7';
    }
  }
  return;
}
bool ReadDistanceFront()
{
  long durationFront, distanceFront;
  digitalWrite(triggerInputFrontAD, LOW);
  digitalWrite(triggerInputFrontAD, HIGH);
  digitalWrite(triggerInputFrontAD, LOW);
  durationFront = pulseIn(echoOutputFrontAD, HIGH);
  distanceFront = (durationFront / 2.) / 29.1;
  return (distanceFront < (long)criticalDistanceAD);
}
bool ReadDistanceBack()
{
  long durationBack, distanceBack;
  digitalWrite(triggerOutputBackAD, LOW);
  digitalWrite(triggerOutputBackAD, HIGH);
  digitalWrite(triggerOutputBackAD, LOW);
  durationBack = pulseIn(echoOutputBackAD, HIGH);
  distanceBack = (durationBack / 2.) / 29.1;
  return (distanceBack < (long)criticalDistanceAD);
}
bool ReadGyro() {
  getHeading();
  //Algorithm for determination of critical gyro change
  //x,y,z are variables for the XYZ axis values
  //criticalGyroUpAD and down are global constants which adjust algorithm sensitivity to movement
  bool gyroChange = false;
  if (abs(xNewAD) > abs(xOldAD * criticalGyroUpAD) ||
      abs(xNewAD) < abs(xOldAD * criticalGyroDownAD) ||
      abs(yNewAD) > abs(yOldAD * criticalGyroUpAD) ||
      abs(yNewAD) < abs(yOldAD * criticalGyroDownAD) ||
      abs(zNewAD) > abs(zOldAD * criticalGyroUpAD) ||
      abs(zNewAD) < abs(zOldAD * criticalGyroDownAD)
     ) {
    gyroChange = true;
  }
  xOldAD = xNewAD;
  yOldAD = yNewAD;
  zOldAD = zNewAD;
  return gyroChange;
}
//Calibration functions
void setupHMC5883L()
{
  compassAD.SetScale(0.88);
  compassAD.SetMeasurementMode(Measurement_Continuous);
}
void getHeading()
{
  MagnetometerRaw raw = compassAD.ReadRawAxis();
  xNewAD = (float)raw.XAxis;
  yNewAD = (float)raw.YAxis;
  zNewAD = (float)raw.ZAxis;
}
float calibrated_values[3];
void transformation(float uncalibrated_values[3]) {
  double calibration_matrix[3][3] =
  {
    { 1.078, 0.009, 0.003 },
    { 0.014, 1.073, -0.162 },
    { 0.038, 0.009, 1.216 }
  };
  double bias[3] =
  {
    -175.886,
    -190.091,
    57.551
  };
  for (int i = 0; i < 3; ++i)
    uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = { 0, 0, 0 };
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i = 0; i < 3; ++i)
    calibrated_values[i] = result[i];
}


/*----------------------------------------
4.400 - SDCard Methods
-----------------------------------------
*/
void writeCSVToSD(char* fileName, String message) {
  sdCardObject = SD.open(fileName, FILE_WRITE);
  if (sdCardObject) {
    sdCardObject.println(message);
    Serial.println(message);
    sdCardObject.close();
  } else {
    Serial.println("Error opening CSV file.");
  }
}

void writeCSVToSD(char* fileName, String dataRTC, int codeAD, String dataGPS) {
  sdCardObject = SD.open(fileName, FILE_WRITE);

  if (sdCardObject) {
    sdCardObject.println(dataRTC + ";" + (String)codeAD + ";" + dataGPS);
    Serial.println(dataRTC + ";" + (String)codeAD + ";" + dataGPS);
    sdCardObject.close();
  } else {
    Serial.println("Error opening CSV file.");
  }
}

void writeTXTToSD(char* fileName) {
  sdCardObject = SD.open(fileName, FILE_WRITE);

  if (sdCardObject) {
    Serial.println(fileName);
    Serial.print("Writing TXT");
    sdCardObject.println((String)gps.location.lat() + ";" + (String)gps.location.lng() + ";");
    sdCardObject.close();
    Serial.println("Finished writing");
  } else {
    Serial.println("Error opening TXT file.");
  }
}
/*----------------------------------------
500 - RTC Polling
-----------------------------------------
*/
byte decToBcd(byte val) {
  return ((val / 10 * 16) + (val % 10));
}
byte bcdToDec(byte val) {
  return ((val / 16 * 10) + (val % 16));
}
void setTime(byte second, byte minute, byte hour,
             byte dayOfWeek, byte dayOfMonth, byte month,
             byte year) {
  Wire.beginTransmission(i2cRTC);
  Wire.write(0);
  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write(decToBcd(dayOfWeek));
  Wire.write(decToBcd(dayOfMonth));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
}
String PollingRTC() {
  String time = "";
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;

  // Getting time from the DS3231
  Wire.beginTransmission(i2cRTC);
  Wire.write(0); // set register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(i2cRTC, 7);

  // Request seven bytes of data starting from register 00h
  second = bcdToDec(Wire.read() & 0x7f);
  minute = bcdToDec(Wire.read());
  hour = bcdToDec(Wire.read() & 0x3f);
  dayOfWeek = bcdToDec(Wire.read());
  dayOfMonth = bcdToDec(Wire.read());
  month = bcdToDec(Wire.read());
  year = bcdToDec(Wire.read());

  // Time
  time.concat(hour);
  time.concat(":");
  if (minute < 10) {
    time.concat("0");
  }
  time.concat(minute);
  time.concat(":");
  if (second < 10) {
    time.concat("0");
  }

  // Date
  time.concat(second);
  time.concat(" ");
  time.concat(dayOfMonth);
  time.concat("/");
  time.concat(month);
  time.concat("/");
  time.concat(year);

  return time;
}
/*----------------------------------------
600 - Display Polling
-----------------------------------------
*/
void PollingLCD(String ulaz) {
  lcd.setCursor(0, 0);
  lcd.print(ulaz);
}

void PollingLCD(String ulaz, int offset, String ulaz2) {
  lcd.print(ulaz);
  lcd.setCursor(offset, 0);
  lcd.print(ulaz2);
}

void PollingLCD(int offset, int line, String ulaz) {
  lcd.setCursor(offset, line);
  lcd.print(ulaz);
  lcd.setCursor(0, 0);
}

void PollingLCD(int offset, String ulaz) {
  lcd.setCursor(offset, 0);
  lcd.print(ulaz);
  lcd.setCursor(0, 0);
}

void ClearLCD() {
  lcd.clear();
}
