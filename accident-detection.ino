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
#include <TinyGPS++.h> // Biblioteka za GPS
#include <SoftwareSerial.h> // Biblioteka za dodatnu serijsku kominkaciju
#include <SPI.h> // Biblioteka (Serial Peripheral Interface) za komunikaciju SD kartice
#include <SD.h> // Biblioteka za SD karticu

//1.1.200 AD Includes
#include "Wire.h"
#include "HMC5883L.h"

//1.1.600 Display Includes
#include <LiquidCrystal.h>

//1.2. Defines
//1.2.100 GPS Defines
#define RXPin 2 // TX i RX pinovi za GPS, spojiti TX-RX, RX-TX
#define TXPin 3

//1.2.200 AD Defines
#define AD_cycles 10
#define AD_triggerInput_front 7
#define AD_echoOutput_front 6
#define AD_triggerInput_back 9
#define AD_echoOutput_back 8
#define AD_critical_distance 20
#define AD_critical_gyro_up 1.20
#define AD_critical_gyro_down 0.80

//1.2.300 Network Defines
#define DEBUG true

//1.2.400 SD Define
#define chipSelect 5 // CS pin SD kartice je spojen na pin 5
#define pinModePin 10

//1.2.500 RTC Defines
#define DS3231_I2C_ADDRESS 0x68

//1.2.600 Display Defines
#define cancelButton 37
#define LCDpin1 11
#define LCDpin2 12
#define LCDpin3 13
#define LCDpin4 14
#define LCDpin5 15
#define LCDpin6 16


//1.3. Globals
//1.3.100 GPS Global
static const unsigned long GPSBaud = 9600; // GPS radi na 9600 bauda
TinyGPSPlus gps; // Instanca TinyGPS objekta
SoftwareSerial ss(RXPin, TXPin); // Serija sa GPS modulom
File sdCardObject; // Varijabla za manipuliranje SD karticom

SoftwareSerial net(17, 18); // Serija sa GPS modulom

//1.3.200 AD Global
HMC5883L AD_compass;
float AD_Xnew, AD_Ynew, AD_Znew;
float AD_Xold, AD_Yold, AD_Zold;
int GLOBAL_cycle = 0;

//1.3.600 Display Global
LiquidCrystal lcd(LCDpin1, LCDpin2, LCDpin3, LCDpin4, LCDpin5, LCDpin6);


/*------------------------------------------------------------------------------------------------------
2. Setup and loop
--------------------------------------------------------------------------------------------------------
*/

//2.1. Setup
void setup() {
  delay(50);
  Serial.begin(9600);
  net.begin(19200);
  setup_GPS();
  setup_AD();
  setup_SDCard();
  pinMode(cancelButton, INPUT);
  Serial.println("Gotov setup");
  GLOBAL_cycle = 0; //Globalna varijabla za polling
}

//2.2. Loop
void loop() {
  //2.2.1. Locals
  int code_AD, code_Network, code_SDCard, code_Display, code_GPS; //Povratni code pojedinih sustava
  String data_RTC, data_GPS; //Povratni podaci pojedinih sustava
  int status_TXTWrite = 0, status_CSVWrite = 0; //Varijable za provjeravanje je li uspjesno zapisano na SD karticu
  bool accident_detected; //Provjerava je li doslo do nesrece
  accident_detected = true;

  //2.2.2. Polling and decision based on code_AD
  code_AD = polling_AD();
  Serial.println(code_AD); //Probni ispis - IZBRISATI NAKON DEBUGGA

  if (code_AD >= 200 && code_AD <= 204) {
    //Everything ok - polling other systems and writing to SD
    if (GLOBAL_cycle %  AD_cycles == 0) {
      //Polling
      data_RTC = polling_RTC();
      data_GPS = polling_GPS(&code_GPS);
      polling_Display("STATUS: OK");

      //Writing to SD
      status_CSVWrite = writeCSVToSD(data_RTC, code_AD, data_GPS);
      Serial.println(status_CSVWrite);
      //if (code_GPS == 100)
//      status_TXTWrite = writeTXTToSD();
      net.println(data_RTC + ";" + (String)code_AD + ";" + (String)code_GPS + ";" + data_GPS);
      Serial.println(data_RTC + ";" + (String)code_AD + ";" + (String)code_GPS + ";" + data_GPS); //Probni ispis - IZBRISATI NAKON DEBUGGA
    }
  }
  else if (code_AD >= 205 && code_AD <= 207) {
    //Zapocni odbrojavanje
    accident_detected = true;
    int accident_counter = 20;
    int buttonState = LOW;
    
    display_Clear();
    while (accident_counter > 0) {
      //Ispisi na LCD poruku korisniku da mora pritisnuti gumb ako je sve ok
      //Ocekuj pritisak gumba za nastavak normalnog rada
      //Ako gumb pritisnut accident_detected = false; break;
      //Ako u 20 sekundi nije pritisnut biti ce i dalje true nakon ove while petlje
      accident_counter--;
      polling_Display("Slanje za: ", 11, (String)accident_counter);
      //polling_Display(1, "Stisni gumb!");
      buttonState = digitalRead(cancelButton);
      if (buttonState == HIGH) {
        accident_detected = false;
        break;
      }
      delay(1000); //Cekamo 20 sekundi
      display_Clear();
    }

    if (accident_detected == true) {
      //Ako tu ulazimo korisnik u 20 sekundi nije stisnuo gumb, dakle nesreca se dogodila
      polling_Display("Poslano serveru!");
      delay(5000);
    }
    
    display_Clear();
  }

  //2.2.4. Cycle update
  if (GLOBAL_cycle > 100)
    GLOBAL_cycle = 0;
  else
    GLOBAL_cycle++;
}

/*------------------------------------------------------------------------------------------------------
3. Setup sources
--------------------------------------------------------------------------------------------------------
*/

//3.100 - GPS Setup
void setup_GPS() {
  ss.begin(GPSBaud);
}

//3.200 - AD Setup
void setup_AD() {
  Wire.begin();
  AD_compass = HMC5883L();
  setupHMC5883L();
  pinMode(AD_triggerInput_front, OUTPUT);
  pinMode(AD_echoOutput_front, INPUT);
  pinMode(AD_triggerInput_back, OUTPUT);
  pinMode(AD_echoOutput_back, INPUT);;
}

//3.400 - SDCard Setup
void setup_SDCard() {
  pinMode(pinModePin, OUTPUT); // Pin 10 mora biti zauzet za SD modul
  SD.begin(chipSelect); // Inicijaliziramo SD karticu i dodijelimo pin
  if (SD.exists("gpsTxtData.txt")) { // Ako postoji gpsData.txt, izbrisat cemo ga i pisati nanovo
    SD.remove("gpsTxtData.txt");
  }
  if (SD.exists("gpsCSVData.csv")) { // Ako postoji gspCSVData.csv, izbrisi i ponovo napravi
    SD.remove("gpsCSVData.csv");
  }
}


/*------------------------------------------------------------------------------------------------------
4. Polling sources
--------------------------------------------------------------------------------------------------------
*/

/*----------------------------------------
4.100 - GPS Polling
-----------------------------------------
*/
String polling_GPS(int* code_GPS) {
  // Nakon svake NMEA recenice ispisuju se podaci
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      return getGPSData(code_GPS);
    }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10) // GPS ne radi
  {
    *code_GPS = 104;
    return "104;x;x;x;"; // Vrati kod za gresku
  }
  *code_GPS = 105;
  return "105;x;x;x;";
}
// Funkcija za ispis podataka
String getGPSData(int* code_GPS) {
  bool locationIsValid = false;
  bool speedIsValid = false;

  // String koji ce biti vracen sa ili bez error kodova
  String gpsDataString = "";
  String gpsDataCode = "";

  // Provjere
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
    *code_GPS = 100;
    gpsDataCode = "100;";
  }
  else if (locationIsValid && !speedIsValid) {
    *code_GPS = 102;
    gpsDataCode = "102;";
  }
  else if (!locationIsValid && speedIsValid) {
    *code_GPS = 101;
    gpsDataCode = "101;";
  }
  else {
    *code_GPS = 103;
    gpsDataCode = "103;";
  }

  return gpsDataCode + gpsDataString;
}

/*----------------------------------------
4.200 - AD Polling
-----------------------------------------
*/
int polling_AD() {
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
  long duration_front, distance_front;
  digitalWrite(AD_triggerInput_front, LOW);
  digitalWrite(AD_triggerInput_front, HIGH);
  digitalWrite(AD_triggerInput_front, LOW);
  duration_front = pulseIn(AD_echoOutput_front, HIGH);
  distance_front = (duration_front / 2.) / 29.1;
  return (distance_front < (long)AD_critical_distance);
}
bool ReadDistanceBack()
{
  long duration_back, distance_back;
  digitalWrite(AD_triggerInput_back, LOW);
  digitalWrite(AD_triggerInput_back, HIGH);
  digitalWrite(AD_triggerInput_back, LOW);
  duration_back = pulseIn(AD_echoOutput_back, HIGH);
  distance_back = (duration_back / 2.) / 29.1;
  return (distance_back < (long)AD_critical_distance);
}
bool ReadGyro() {
  getHeading();
  //Algorithm for determination of critical gyro change
  //xv,yv,zv are variables for the XYZ axis values
  //critical_gyro_up and down are global constants which adjust algorithm sensitivity to movement
  bool gyroChange = false;
  if (abs(AD_Xnew) > abs(AD_Xold * AD_critical_gyro_up) ||
      abs(AD_Xnew) < abs(AD_Xold * AD_critical_gyro_down) ||
      abs(AD_Ynew) > abs(AD_Yold * AD_critical_gyro_up) ||
      abs(AD_Ynew) < abs(AD_Yold * AD_critical_gyro_down) ||
      abs(AD_Znew) > abs(AD_Zold * AD_critical_gyro_up) ||
      abs(AD_Znew) < abs(AD_Zold * AD_critical_gyro_down)
     ) {
    gyroChange = true;
  }
  AD_Xold = AD_Xnew;
  AD_Yold = AD_Ynew;
  AD_Zold = AD_Znew;
  return gyroChange;
}
//Calibration functions
void setupHMC5883L()
{
  AD_compass.SetScale(0.88);
  AD_compass.SetMeasurementMode(Measurement_Continuous);
}
void getHeading()
{
  MagnetometerRaw raw = AD_compass.ReadRawAxis();
  AD_Xnew = (float)raw.XAxis;
  AD_Ynew = (float)raw.YAxis;
  AD_Znew = (float)raw.ZAxis;
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
int writeTXTToSD() {
  sdCardObject = SD.open("gpsTxtData.txt", FILE_WRITE); // Otvaramo gpsData.txt za pisanje
  if (sdCardObject) {
    sdCardObject.print(gps.location.lng(), 6); //Na 6 decimala
    sdCardObject.print(",");
    sdCardObject.print(gps.location.lat(), 6);
    sdCardObject.print(" ");
    sdCardObject.close();
    return 0; //Pisanje proslo ok
  }
  else
    return -1; //Nije uspio otvoriti
}

int writeCSVToSD(String data_RTC, int code_AD, String data_GPS) {
  File sdCardObject2 = SD.open("gpsCSVData.csv", FILE_WRITE); // Otvaramo gpsCSVData.csv za pisanje
  if (sdCardObject2) { //Ako je uspio otvoriti, inace SD.open vraca false
    sdCardObject2.println(data_RTC + ";" + (String)code_AD + + ";" + data_GPS);
    sdCardObject2.close();
    return 0; //Pisanje proslo ok
  }
  else
    return -1; //Nije uspio otvoriti
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
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
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
String polling_RTC() {
  String time = "";
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;

  // Getting time from the DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);

  // request seven bytes of data starting from register 00h
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
void polling_Display(String ulaz) {
  lcd.setCursor(0, 0);
  lcd.print(ulaz);
}

void polling_Display(String ulaz, int offset, String ulaz2) {
  lcd.print(ulaz);
  lcd.setCursor(offset, 0);
  lcd.print(ulaz2);
}

void polling_Display(int line, String ulaz) {
  lcd.setCursor(0, line);
  lcd.print(ulaz);
  lcd.setCursor(0, 0);
}

void display_Clear() {
  lcd.clear();
}
