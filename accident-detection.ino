/*DOCUMENTATION:
1. Includes | Defines |Globals
2. Loop and main
3. Setups
4. Polling
*/


/*------------------------------------------------------------------------------------------------------
1. Includes | Defines |Globals
--------------------------------------------------------------------------------------------------------
*/
//100 GPS Includes
#include <TinyGPS++.h> // Biblioteka za GPS
#include <SoftwareSerial.h> // Biblioteka za dodatnu serijsku kominkaciju
#include <SPI.h> // Biblioteka (Serial Peripheral Interface) za komunikaciju SD kartice
#include <SD.h> // Biblioteka za SD karticu
//200 AD Includes
#include "Wire.h"
#include "HMC5883L.h"
//300 Network Includes
#include <EtherCard.h>
//500 RTC Includes
//600 Display Includes
#include <LiquidCrystal.h>
#include <dht.h>


//100 GPS Defines
#define RXPin 4 // TX i RX pinovi za GPS, spojiti TX-RX, RX-TX
#define TXPin 3
//200 AD Defines
#define AD_triggerInput_front 4
#define AD_echoOutput_front 3
#define AD_triggerInput_back 7
#define AD_echoOutput_back 6
#define AD_critical_distance 20
#define AD_critical_gyro_up 1.20
#define AD_critical_gyro_down 0.80
//300 Network Defines
#define DEBUG true
// 400 SD Define
#define chipSelect 2// CS pin SD kartice je spojen na pin 2
#define pinModePin 10
//500 RTC Defines
#define DS3231_I2C_ADDRESS 0x68
//600 Display Defines
#define DHT11_PIN 6


//100 GPS Global
static const unsigned long GPSBaud = 9600; // GPS radi na 9600 bauda
TinyGPSPlus gps; // Instanca TinyGPS objekta
SoftwareSerial ss(RXPin, TXPin); // Serija sa GPS modulom
File sdCardObject; // Varijabla za manipuliranje SD karticom
//200 AD Global
HMC5883L AD_compass;
float AD_xv, AD_yv, AD_zv;
float AD_xold, AD_yold, AD_zold;
//300 Network Global
static uint32_t timer;
static byte session;
Stash stash;
byte Ethernet::buffer[700];
static byte myip[] = { 172, 16, 2, 2 }; // IP of the device
static byte gwip[] = { 172, 16, 2, 1 }; // Gateway of the device
static byte dnsip[] = { 172, 16, 0, 3 }; // IP of the DNS server
static byte mymac[] = { 0x74, 0x69, 0x69, 0x2D, 0x30, 0x31 }; // MAC address
const char website[] PROGMEM = "adb.dokku.d.h"; // Server address
//400 SDCard Global
//500 RTC Global
//600 Display Global
dht DHT;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);


/*------------------------------------------------------------------------------------------------------
2. Loop and main
--------------------------------------------------------------------------------------------------------
*/

void setup() {
  setup_GPS();
  setup_AD();
  setup_Network();
  setup_SDCard();
  setup_RTC();
  setup_Display();
}
void loop() {
  int code_AD, code_Network, code_SDCard, code_Display;
  String data_RTC, data_GPS;

  //Polling
  data_GPS = polling_GPS();
  code_AD = polling_AD();
  code_Network = polling_Network();
  code_SDCard = polling_SDCard();
  data_RTC = polling_RTC();
  code_Display = polling_Display();

  //Decision
}



/*------------------------------------------------------------------------------------------------------
3. Setup sources
--------------------------------------------------------------------------------------------------------
*/

//100 - GPS Setup
void setup_GPS() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  
  pinMode(pinModePin, OUTPUT); // Pin 10 mora biti zauzet za SD modul
  SD.begin(chipSelect); // Inicijaliziramo SD karticu i dodijelimo pin
  if (SD.exists("gpsTxtData.txt")) { // Ako postoji gpsData.txt, izbrisat cemo ga i pisati nanovo
    SD.remove("gpsTxtData.txt");
  }
  if (SD.exists("gpsCSVData.csv")) { // Ako postoji gspCSVData.csv, izbrisi i ponovo napravi
    SD.remove("gpsCSVData.csv");
  }
}

//200 - AD Setup
void setup_AD() {
  Serial.begin(9600);
  Wire.begin();
  AD_compass = HMC5883L();
  setupHMC5883L();
  pinMode(AD_triggerInput_front, OUTPUT);
  pinMode(AD_echoOutput_front, INPUT);
  pinMode(AD_triggerInput_back, OUTPUT);
  pinMode(AD_echoOutput_back, INPUT);;
}

//300 - Ethernet Setup
void setup_Network() {
}

//400 - SDCard Setup
void setup_SDCard() {
  pinMode(10, OUTPUT); // Pin 10 mora biti zauzet za SD modul
  SD.begin(chipSelect); // Inicijaliziramo SD karticu i dodijelimo pin
  if (SD.exists("gpsTxtData.txt")) { // Ako postoji gpsData.txt, izbrisat cemo ga i pisati nanovo
    SD.remove("gpsTxtData.txt");
  }
  if (SD.exists("gpsCSVData.csv")) { // Ako postoji gspCSVData.csv, izbrisi i ponovo napravi
    SD.remove("gpsCSVData.csv");
  }
}

//500 - RTC Setup
void setup_RTC() {
}

//600 - Display Setup
void setup_Display() {
}


/*------------------------------------------------------------------------------------------------------
4. Polling sources
--------------------------------------------------------------------------------------------------------
*/

/*----------------------------------------
100 - GPS Polling
-----------------------------------------
*/
String polling_GPS() {
  // Nakon svake NMEA recenice ispisuju se podaci
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      return getGPSData();
    }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10) // GPS ne radi
  {
    return "103;103;103;"; // Vrati kod za gresku
  }
}
// Funkcija za ispis podataka
String getGPSData() {
  // String koji ce biti vracen sa ili bez error kodova
  String gpsDataString = "";
  
  // Provjere
  if (gps.location.isValid()){
    gpsDataString += gps.location.lat() + ";" + gps.location.lng() + ";";
    
    if(gps.speed.isValid()){
      writeTXTToSD();
      writeCSVToSD();
    }
  }
  else{
    gpsDataString += "102;102;";
  }
  
  if(gps.speed.isValid){
    gpsDataString += gps.speed.kmph() + ";";
  }
  else{
    gpsDataString += "102;";
  }
  
  return gpsDataString;
}
  
/*----------------------------------------
200 - AD Polling
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
  if (abs(AD_xv) > abs(AD_xold * AD_critical_gyro_up) ||
      abs(AD_xv) < abs(AD_xold * AD_critical_gyro_down) ||
      abs(AD_yv) > abs(AD_yold * AD_critical_gyro_up) ||
      abs(AD_yv) < abs(AD_yold * AD_critical_gyro_down) ||
      abs(AD_zv) > abs(AD_zold * AD_critical_gyro_up) ||
      abs(AD_zv) < abs(AD_zold * AD_critical_gyro_down)
     ) {
    gyroChange = true;
  }
  AD_xold = AD_xv;
  AD_yold = AD_yv;
  AD_zold = AD_zv;
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
  AD_xv = (float)raw.XAxis;
  AD_yv = (float)raw.YAxis;
  AD_zv = (float)raw.ZAxis;
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
300 - Ethernet Polling
-----------------------------------------
*/
int polling_Network() {
}
int sendData(String csv) {
  // Format the CSV data to JSON and send it to the server.
}


/*----------------------------------------
400 - SDCard Polling
-----------------------------------------
*/
int polling_SDCard() {
}
void writeTXTToSD() {
  sdCardObject = SD.open("gpsTxtData.txt", FILE_WRITE); // Otvaramo gpsData.txt za pisanje
  sdCardObject.print(gps.location.lng(), 6);
  sdCardObject.print(",");
  sdCardObject.print(gps.location.lat(), 6);
  sdCardObject.print(" ");
  sdCardObject.close();
}
void writeCSVToSD() {
  sdCardObject = SD.open("gpsCSVData.csv", FILE_WRITE); // Otvaramo gpsCSVData.csv za pisanje
  sdCardObject.print(gps.location.lng(), 6);
  sdCardObject.print(";");
  sdCardObject.print(gps.location.lat(), 6);
  sdCardObject.print(";");
  sdCardObject.print(gps.speed.kmph());
  sdCardObject.print(";");
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
  byte second, minute, hour, dayOfWeek,
       dayOfMonth, month, year;
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
  if (minute < 10)
  {
    time.concat("0");
  }
  time.concat(minute);
  time.concat(":");
  if (second < 10)
  {
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
int polling_Display() {
  int check = DHT.read11(DHT11_PIN);
  int statusSensor;
  String CSV;
  statusSensor = CheckSensorStatus(check); // Dohvaća status senzora
  CSV = CSVFormat(statusSensor); // Pravi CSV format koji će se kasnije slati na server
  lcd.setCursor(13, 0); // Ispis temperature
  lcd.print(DHT.temperature, 1);
  lcd.print(" C");
  lcd.setCursor(13, 1); // Ispis vlažnosti
  lcd.print(DHT.humidity, 1);
  lcd.print(" %");
  lcd.setCursor(15, 3); // Ispis error koda
  lcd.print(statusSensor);
  delay(1000);
}
int CheckSensorStatus(int check) {
  switch (check)
  {
    case DHTLIB_OK: // Sve je u redu s komunikacijom
      lcd.setCursor(0, 3);
      lcd.print("OK");
      return 400;
    case DHTLIB_ERROR_CHECKSUM: // Primljeni su krivi podaci
      lcd.setCursor(0, 3);
      lcd.print("Checksum error");
      return 401;
    case DHTLIB_ERROR_TIMEOUT: // Komunikacija nije uspijela s senzorom
      lcd.setCursor(0, 3);
      lcd.print("Time out error");
      return 402;
    case DHTLIB_ERROR_CONNECT: // Jedan od pin-ova (vrlo vjerovatno data pin) je odspojen
      lcd.setCursor(0, 3);
      lcd.print("Connect error");
      return 403;
    case DHTLIB_ERROR_ACK_L: // Data pin spojen na GND ili niski napon
      lcd.setCursor(0, 3);
      lcd.print("Ack Low error");
      return 404;
    case DHTLIB_ERROR_ACK_H: // Data pin spojen na visoki napon (+3.3V ili vise)
      lcd.setCursor(0, 3);
      lcd.print("Ack High error");
      return 405;
    default:
      lcd.setCursor(0, 3);
      lcd.print("Unknown error");
      return 410;
  }
}
String CSVFormat(int statusSensor) {
  String toReturn;
  toReturn = toReturn + "Temperatura" + ";" + DHT.temperature + ";" + "Vlaznost" + DHT.humidity + ";" + "Status" + statusSensor;
  return toReturn;
}
