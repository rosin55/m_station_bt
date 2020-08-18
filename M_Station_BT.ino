/***************************************************************************
Скетч для измерения и отображения на LCD1602 и по Bluetooth температуры, давления и влажности.
Температура и давление измеряется датчиком BMP280 (подключен по I2C), влажность - DHT11.
Имеется возможность просмотра данных через Bluetooth на телефоне. Соединение по Bluetooth
осуществляется с помощью модуля HC-05.

Автор - Андрей Сахно
Создан -  15.04.2018
Изменен - 18.08.2020
***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#define BMP280_ADDRESS (0x76) // явное задание адреса датчика для I2C
#define BMP_SCK 13    //
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#include "DHT.h"
#define DHTPIN A3 // пин подключения контакта DATA
#define DHTTYPE DHT11 // DHT 11

#include <LiquidCrystal.h>

#include <SoftwareSerial.h>

#define VERSION "20.08.18"

// инициализация с указанием контактов подключения
LiquidCrystal lcd(9, 8, 7, 6, 5, 4);
byte Gradus[8] = {
  0b00110,
  0b01001,
  0b01001,
  0b00110,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};
SoftwareSerial mySerial(2, 3); // RX, TX
char data; // данные принятые с BT
// измеренные параметры
float te; // температура воздуха
float pr; // атм. давление
float hu; // влажность воздуха
// время с момента старта программы
int minut =0;
int sekund =0;
int chasov =0;
String stroka = ""; // строка для отображения времени
long int timer = 0; // текущее время

const int analogInPin = A0; // номер аналогового входа для измерения напряжения батареи
int sensorValue = 0;        // величина напряжения после делителя
float outputValue = 0;      // значение напряжения питания для вывода на экран
float k ;                   // коэффициент делителя напряжения

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
DHT dht(DHTPIN, DHTTYPE);

//############################################################################
void setup() {
  Serial.begin(9600);
  // устанавливаем скорость передачи данных для последовательного порта, созданного
  // библиотекой SoftwareSerial
   mySerial.begin(9600);
   mySerial.println("Hi! This is M_Station_BT!");

  analogReference(INTERNAL); // внутренний опорный источник
  k = 8.1 / 682;

  dht.begin();
  Serial.println("Hi! This is M_Station_BT!");
  delay(1000);
  lcd.begin(16,2); // режим работы
  delay(100);
 // создание знака "градус"
  lcd.createChar(0, Gradus);
  delay(100);
  lcd.clear();     // очистка дисплея
  lcd.print("Start v");
  lcd.print(VERSION);
  delay(2000);
  IzmerBatarei(); // измерить и показать напряжение батареи питания

  lcd.clear();     // очистка дисплея
  lcd.setCursor(2,0);
  // опрос датчика давления и температуры
  if (!bmp.begin(BMP280_ADDRESS)) {
    Serial.println("Ошибка получения данных с датчика BMP280");
    lcd.print("Error BMP280");
    while (1);
  }
  // опрос датчика влажности
  float h = dht.readHumidity();
  if (isnan(h)) {// ошибка получения данных
    Serial.println("Ошибка получения данных с датчика DHT11");
    lcd.print("Error DHT11");
    while (1);
  }
}

//############################################################################
void loop() {
  if (millis() - timer > 2000) {
    timer = millis();
    Izmerenie ();
//    OtobrMonitor (te, pr, hu);
    VremyaToStroka();
    OtobrLCD (te, pr, hu);
    ZaprDann();
  }
}

//############################################################################
// **************************************************
void Izmerenie () {
  te = bmp.readTemperature();
  pr = bmp.readPressure() * 0.0075; // перевод в мм.рт.ст.
  hu = dht.readHumidity();
}
// ***************************************************
void OtobrMonitor (float t, float p, float h) {
    Serial.print("Температура: ");
    Serial.print(t);
    Serial.println(" *C");

    Serial.print("Давление = ");
    Serial.print(p);
    Serial.println(" mm Hg");

    Serial.print("Влажность = ");
    Serial.print(h);
    Serial.println(" %");
    Serial.println (stroka);

    Serial.println();
}
// ***************************************************
void OtobrLCD (float t, float p, float h) {
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("H:");lcd.print(h,0); lcd.print("%");
  lcd.setCursor(8,0);
  lcd.print("T:"); lcd.print(t,1);
  lcd.write(byte(0)); // печать знака градуса Цельсия
  lcd.print("C");
  lcd.setCursor(0,0);
  lcd.print("P:");lcd.print(p,0); lcd.print("mm");
  // print the number of seconds since reset:
  lcd.setCursor(8,1);
  lcd.println (stroka);  // отображение времени с момента запуска прибора
//  lcd.print(millis() / 1000);

}
void VremyaToStroka(){   // перевод времени в строку символов
  int time = millis()/1000;
  sekund = time%60;
  chasov = time/3600;
  minut =  time/60 - chasov*60;
  if (chasov < 10) {stroka = '0';}
  stroka = stroka + chasov + ':';
  if (minut < 10) {stroka += '0';}
  stroka = stroka + minut + ':';
  if (sekund < 10) {stroka +='0';}
  stroka = stroka + sekund;
}

void IzmerBatarei(){
  sensorValue = analogRead(analogInPin);
  outputValue = sensorValue * k ;
  Serial.print(sensorValue); Serial.println(outputValue);
  lcd.setCursor(0,1);
  lcd.print("U bat = "); lcd.print(outputValue);
  delay(3000);
}

void ZaprDann(){
  if(mySerial.available() )  {
    data = mySerial.read();
    switch (data){
      case '?':
        mySerial.println("A - температура, атм. давление, влажность.");
        mySerial.println("T - температура.");
        mySerial.println("P - атм. давление.");
        mySerial.println("H - влажность.");
        mySerial.println("V - время с момента включения.");
        mySerial.println("B - напряжение питания (В вольтах)");
      break;
      case 'A':
        mySerial.print("t= "); mySerial.print(te); mySerial.println(" *C");
        mySerial.print("p= ");mySerial.print(pr); mySerial.println(" mm Hg");
        mySerial.print("h= ");mySerial.print(hu); mySerial.println(" %");
        mySerial.println(stroka);
      break;
      case 'T':
        mySerial.print("t= "); mySerial.print(te); mySerial.println(" *C");
      break;
      case 'P':
        mySerial.print("p= "); mySerial.print(pr); mySerial.println(" mm Hg");
      break;
      case 'H':
        mySerial.print("h= "); mySerial.print(hu); mySerial.println(" %");
      break;
      case 'B':
        sensorValue = analogRead(analogInPin);
        outputValue = sensorValue * k ;
        mySerial.print("U bat = ");mySerial.print(outputValue); mySerial.println(" V");
      break;
      case 'V':
        mySerial.println(stroka);
      break;
      default:
        mySerial.print(data);mySerial.println(" is not a command!");
      break;
    }
  }
}
