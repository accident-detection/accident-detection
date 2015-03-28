/*DOCUMENTATION:
1. Includes and pins
2. Loop and main
3. Setups
4. Polling
*/

/*------------------------------------
  1. Includes and pins
--------------------------------------
*/
//100 GPS Includes
#include <TinyGPS++.h> // Biblioteka za GPS
static const unsigned long GPSBaud = 9600; // GPS radi na 9600 bauda
TinyGPSPlus gps; // Instanca TinyGPS objekta
SoftwareSerial ss(RXPin, TXPin); // Serija sa GPS modulom
File sdCardObject; // Varijabla za manipuliranje SD karticom
#define RXPin 4 // TX i RX pinovi za GPS, spojiti TX-RX, RX-TX
#define TXPin 3
#define chipSelect 2// CS pin SD kartice je spojen na pin 2

//200 AD Includes
#include "Wire.h"
#include "HMC5883L.h"
HMC5883L compass;
float xv, yv, zv;
float xold, yold, zold;
#define triggerInput_front 4
#define echoOutput_front 3
#define triggerInput_back 7
#define echoOutput_back 6
#define critical_distance 20
#define critical_gyro_up 1.20
#define critical_gyro_down 0.80

//300 Network Includes
#include <EtherCard.h>
static uint32_t timer;
static byte session;
Stash stash;
byte Ethernet::buffer[700];
static byte myip[] = { 172, 16, 2, 2 }; // IP of the device
static byte gwip[] = { 172, 16, 2, 1 }; // Gateway of the device
static byte dnsip[] = { 172, 16, 0, 3 }; // IP of the DNS server
static byte mymac[] = { 0x74, 0x69, 0x69, 0x2D, 0x30, 0x31 }; // MAC address
const char website[] PROGMEM = "adb.dokku.d.h"; // Server address
#define DEBUG true

//400 SDCard Includes
#include <SPI.h> // Biblioteka (Serial Peripheral Interface) za komunikaciju SD kartice
#include <SD.h> // Biblioteka za SD karticu
#include <SoftwareSerial.h> // Biblioteka za dodatnu serijsku kominkaciju

//500 RTC Includes
#include "Wire.h"
#define DS3231_I2C_ADDRESS 0x68

//600 Display Includes
#include <LiquidCrystal.h>
#include <dht.h>
dht DHT;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
#define DHT11_PIN 6


/*------------------------------------
  2. Loop and main
--------------------------------------
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
  int code_GPS, code_AD, code_Network, code_SDCard, code_RTC, code_Display;
  //Polling
  code_GPS = polling_GPS();
  code_AD = polling_AD();
  code_Network = polling_Network();
  code_SDCard = polling_SDCard();
  String code_RTC = polling_RTC();
  code_Display = polling_Display();

  //Decision
}


/*------------------------------------
  3. Setup sources
--------------------------------------
*/
//100 - GPS  Setup
void setup_GPS() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  Serial.println(F("*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*"));
  Serial.println(F("Visoka skola za primijenjeno racunarstvo"));
  Serial.println(F("Sustav za detekciju prometne nesrece"));
  Serial.println(F("Testiranje GPS modula"));
  Serial.print(F("TinyGPS++ biblioteka u verziji ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  pinMode(10, OUTPUT); // Pin 10 mora biti zauzet za SD modul
  SD.begin(chipSelect); // Inicijaliziramo SD karticu i dodijelimo pin
  if (SD.exists("gpsTxtData.txt")) { // Ako postoji gpsData.txt, izbrisat cemo ga i pisati nanovo
    SD.remove("gpsTxtData.txt");
  }
  if (SD.exists("gpsCSVData.csv")) { // Ako postoji gspCSVData.csv, izbrisi i ponovo napravi
    SD.remove("gpsCSVData.csv");
  }
}
//200 - AD  Setup
void setup_AD() {
  Serial.begin(9600);
  Wire.begin();
  compass = HMC5883L();
  setupHMC5883L();
  pinMode(triggerInput_front, OUTPUT);
  pinMode(echoOutput_front, INPUT);
  pinMode(triggerInput_back, OUTPUT);
  pinMode(echoOutput_back, INPUT);;
}
//300 - Ethernet Setup
void setup_Network() {

}

int sendData(String csv) {
  // Format the CSV data to JSON and send it to the server.
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


/*------------------------------------
  4. Polling sources
--------------------------------------
*/
//100 - GPS  Polling
int polling_GPS() {
  // Nakon svake NMEA recenice ispisuju se podaci
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      displayInfo();
      writeTXTToSD();
    }
  }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("Veza sa GPS modulom nije uspostavljena."));
    while (true) {};
  }
}

// Funkcija za ispis podataka
void displayInfo() {
  // Koordinate
  Serial.print(F("Lokacija: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("[KOORDINATE NEDOSTUPNE]"));
  }
  // Datum i vrijeme
  Serial.print(F(" Datum/Vrijeme: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.day());
    Serial.print(F("."));
    Serial.print(gps.date.month());
    Serial.print(F("."));
    Serial.print(gps.date.year());
    Serial.print(F("."));
  }
  else
  {
    Serial.print(F("[DATUM NEDOSTUPAN]"));
  }
  Serial.print(F("/"));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour() + 1);
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("[VRIJEME NEDOSTUPNO]"));
  }
  Serial.print(F(" "));
  // Brzina
  if (gps.speed.isValid())
  {
    Serial.print("Brzina: ");
    Serial.print(gps.speed.kmph());
    Serial.print("km/h");
  }
  else
  {
    Serial.print(F("[BRZINA NIJE DOSTUPNA]")) ;
  }
  Serial.print(F(" "));
  // Visina
  if (gps.altitude.isValid())
  {
    Serial.print("Visina: ");
    Serial.print(gps.altitude.meters());
  }
  else
  {
    Serial.print("[VISINA NIJE DOSTUPNA]");
  }
  Serial.print(F(" "));
  // Jacina signala
  if (gps.hdop.isValid())
  {
    Serial.print("Jacina signala (HDOP): ");
    Serial.print(gps.hdop.value());
  }
  else
  {
    Serial.print("[JACINA SIGNALA NIJE DOSTUPNA]");
  }
  Serial.println();
}
//200 - AD  Polling
int polling_AD() {
  char accidentState = 0;
  Serial.println();
  AccidentDetector(&accidentState);
  Serial.println(accidentState);
  switch (accidentState) {
    case '1':
      Serial.println("201 - Back sensor!");
      break;
    case '2':
      Serial.println("202 - Front sensor!");
      break;
    case '3':
      Serial.println("203 - Both sensors!");
      break;
    case '4':
      Serial.println("204 - Gyro reacted!");
      break;
    case '5':
      Serial.println("205 - BACK HIT!");
      Serial.println("205 - BACK HIT!");
      Serial.println("205 - BACK HIT!");
      break;
    case '6':
      Serial.println("206 - FRONT HIT!");
      Serial.println("206 - FRONT HIT!");
      Serial.println("206 - FRONT HIT!");
      break;
    case '7':
      Serial.println("207 - HIT WHILE SURROUNDED!");
      Serial.println("207 - HIT WHILE SURROUNDED!");
      Serial.println("207 - HIT WHILE SURROUNDED!");
      break;
    default:
      Serial.println("200 - Everything ok!");
      break;
  }
}
//300 - Ethernet Polling
int polling_Network() {

}
//400 - SDCard Polling
int polling_SDCard() {
  void writeTXTToSD() {
    if (gps.location.isValid()) { // Zapisujemo samo ako imamo koordinate
      sdCardObject = SD.open("gpsTxtData.txt", FILE_WRITE); // Otvaramo gpsData.txt za pisanje
      sdCardObject.print(gps.location.lng(), 6);
      sdCardObject.print(",");
      sdCardObject.print(gps.location.lat(), 6);
      sdCardObject.print(" ");
      sdCardObject.close();
    }
  }
  void writeCSVToSD() {
    if (gps.location.isValid()) {
      sdCardObject = SD.open("gpsCSVData.csv", FILE_WRITE); // Otvaramo gpsCSVData.csv za pisanje
      sdCardObject.print(gps.location.lng(), 6);
      sdCardObject.print(";");
      sdCardObject.print(gps.location.lat(), 6);
      sdCardObject.print(";");
      sdCardObject.print(gps.speed.kmph());
      sdCardObject.print(";");
      sdCardObject.print(gps.altitude.meters());
      sdCardObject.print(";");
    }
  }
}
//500 - RTC Polling
byte decToBcd(byte val) {
  return ( (val / 10 * 16) + (val % 10) );
}
byte bcdToDec(byte val) {
  return ( (val / 16 * 10) + (val % 16) );
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
//600 - Display Polling
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
int CheckSensorStatus (int check) {
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
String CSVFormat (int statusSensor) {
  String toReturn;
  toReturn = toReturn + "Temperatura" + ";" + DHT.temperature + ";" + "Vlaznost" + DHT.humidity + ";" + "Status" + statusSensor;
  return toReturn;
}
void SetupLCDDisplay () {
  lcd.begin(20, 4);
  lcd.print("Temperatura: ");
  lcd.setCursor(0, 1);
  lcd.print("Vlaznost: ");
}
