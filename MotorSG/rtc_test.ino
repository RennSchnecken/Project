// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"


RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {"Sonntag", "Montag", "Dienstag", "Mittwoch", "Donnerstag", "Freitag", "Samstag"};

void setup () {

  Serial.begin(9600);

  delay(3000); // wait for console opening

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC auf Initialwert gesetzt");  //Bei Spannungsverlust auf Initialwert
    rtc.adjust(DateTime(__DATE__, __TIME__));   //Datum des Sketches als Initialwert
  }
}

void loop () {
  DateTime now = rtc.now();
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(", ");
    Serial.print(now.day(), DEC);
    Serial.print(".");
    Serial.print(now.month(), DEC);
    Serial.print(".");
    Serial.print(now.year(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println("Uhr");

    // calculate a date which is 7 days and 30 seconds into the future
    /*DateTime future (now + TimeSpan(3,2,11,33)); // tage, stunden, minuten, Sekunden
    
    Serial.print(future.year(), DEC);
    Serial.print('/');
    Serial.print(future.month(), DEC);
    Serial.print('/');
    Serial.print(future.day(), DEC);
    Serial.print(' ');
    Serial.print(future.hour(), DEC);
    Serial.print(':');
    Serial.print(future.minute(), DEC);
    Serial.print(':');
    Serial.print(future.second(), DEC);
    Serial.println();
    */
    delay(1000);
}
