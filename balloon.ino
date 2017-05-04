#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

String getFormattedGPSCoord(float coord);

/******************************************************************
 Created with PROGRAMINO IDE for Arduino - 26.04.2017 21:19:47
 Project     : High Altitude Balloon
 Libraries   :
 Author      :
 Description :
******************************************************************/
const int gpsRxPin = 3;
const int gpsTxPin = 2;

SoftwareSerial gpsSerial(gpsRxPin, gpsTxPin);
Adafruit_GPS GPS(&gpsSerial);

void setup() {

  // connect at 115200 baud rate so we can read the GPS fast enough
  Serial.begin(115200);

  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);
  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // Request updates on antenna status
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000); // Delay a second

  gpsSerial.println(PMTK_Q_RELEASE);

  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function above
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  // logCurrentGPSFirmwareVersion()
  // GPSSerial.println(PMTK_Q_RELEASE);
    // write your setup code here, to run once

}

// Interrupt is called once a millisecond, looks for any new GPS data
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

const int ONE_SECOND = 1000;
unsigned long secondTimer = millis();
const int GPS_INTERVAL = 5000; // in milliseconds
unsigned long gpsTimer = millis(); // timer for gps updates

void loop() {

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    GPS.parse(GPS.lastNMEA());
  }

  if(timeToUpdateGPS()) {
    logGPSToSerial(); // Log GPS information to Serial
    //logGPSToAPRS();
    gpsTimer = millis();
  }

  //if(timeToUpdateHamPosition()) {
  //  updateHamPosition();
  //}

  //if(shouldSendSMS()) {
  // sendSMS();
  //}

  //if(timeToTakePicture()) {
  //  takePicture();
  //}


    // write your main code here, to run repeatedly

}

boolean timeToUpdateGPS(void) {
  return millis() - gpsTimer >= GPS_INTERVAL;
}



void logGPSToSerial() {
  Serial.print("Date/Time: ");
  Serial.print(GPS.day, DEC); Serial.print("/");
  Serial.print(GPS.month, DEC); Serial.print("/");
  Serial.print(GPS.year, DEC); Serial.print(" ");
  Serial.print(GPS.hour, DEC); Serial.print(":");
  Serial.print(GPS.minute, DEC); Serial.print(":");
  Serial.print(GPS.seconds, DEC);

  Serial.println();
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

  if(GPS.fix) {
    Serial.print("Lat/Lon: ");
    Serial.print(getFormattedGPSCoord(GPS.latitude)); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(getFormattedGPSCoord(GPS.longitude)); Serial.println(GPS.lon);

    Serial.print("Lat/Lon Fixed : ");
    Serial.print(GPS.latitude_fixed); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude_fixed); Serial.println(GPS.lon);

    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    Serial.println();
  }
}

String getFormattedGPSCoord(float coord) {
  String coordStr = String(coord);
  String degrees = coordStr.substring(0 ,coordStr.indexOf(".") - 2);
  String minutes = coordStr.substring(coordStr.indexOf(".") - 2, coordStr.length());
  return degrees + " " + minutes;
}
