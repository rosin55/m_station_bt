/***************************************************************************
Скетч для измерения и отображения на LCD1602 и по Bluetooth температуры, давления и влажности.
Температура и давление измеряется датчиком BMP280 (подключен по I2C), влажность - DHT11.
Имеется возможность просмотра данных через Bluetooth на телефоне. Соединение по Bluetooth
осуществляется с помощью модуля HC-05.

Автор - Андрей Сахно
Создан -  15.04.2018
Изменен - 17.09.2020
***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#define BMP280_ADDRESS (0x76) // явное задание адреса датчика для I2C
// датчик BMP 280 подключен к встроенному I2c интерфейсу:
// SCL - A5
// SDA - A4

// #define BMP_SCK 13
// #define BMP_MISO 12
// #define BMP_MOSI 11
// #define BMP_CS 10

#include "DHT.h"
#define DHTPIN A3 // пин подключения контакта DATA
#define DHTTYPE DHT11 // DHT 11

#include <LiquidCrystal.h>

#include <SoftwareSerial.h>

#define VERSION "25.09.20"
#define VERSION2 "(1.0.3)"

// расширенное считывание нажатий кнопок от Alex Gyver (Алекса Гайвера)
#include <GyverButton.h> // библиотека работы с кнопками
// кнопка подключена сюда (BTN_PIN --- КНОПКА --- GND)
#define RESET_BTN_PIN 11
#define MODE_SELECT_BTN_PIN 10

GButton btnReset(RESET_BTN_PIN);
GButton btnModeSelect(MODE_SELECT_BTN_PIN);


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

// минимальные и максимальные значения
float teMin;
float prMin;
float huMin;
float prMax;
float huMax;
float teMax;

int lcdRezhim = 1; // режим отображения

// время с момента старта программы
int minut =0;
int sekund =0;
int chasov =0;
int dney =0;

String stroka = ""; // строка для отображения времени
long int timer = 0; // таймер отображения времни
long int disp_timer = 0; // таймер отображения параметров

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
  lcd.setCursor(0, 1);
  lcd.print(VERSION);
  lcd.print(" ");
  lcd.print(VERSION2);
  delay(4000);
  lcd.clear();
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

  // Задание начальных минимальных и максимальных значений
  Izmerenie();
  resetMaxMin(te, pr, hu);
}

//############################################################################
void loop() {

  btnReset.tick();
  btnModeSelect.tick();

  if (btnModeSelect.isSingle()) {
    lcdRezhim += 1;
    if (lcdRezhim > 3)
    {
      lcdRezhim = 1;
    }
    OtobrLCD(te, pr, hu);
  }

  if (btnReset.isSingle())
  {
    resetMaxMin(te, pr, hu);
  }

  if (millis() - timer > 1000) {  // инкремент времени раз в секунду
    timer = millis();
    CountTime();
    if (lcdRezhim == 1)  { PrintTime(); } // время только в основном режиме
  }
  if (millis() - disp_timer > 30000) {    // вывод параметров раз в 30-ть сек
    disp_timer = millis();
    Izmerenie();
    CheckMaxMin(te, pr, hu);
    OtobrLCD(te, pr, hu);
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
  if (lcdRezhim == 1){
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
  }
  else if (lcdRezhim == 2){
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("H:");lcd.print(huMin,0); lcd.print("%");
    lcd.setCursor(8,0);
    lcd.print("T:"); lcd.print(teMin,1);
    lcd.write(byte(0)); // печать знака градуса Цельсия
    lcd.print("C");
    lcd.setCursor(0,0);
    lcd.print("P:");lcd.print(prMin,0); lcd.print("mm");
    lcd.setCursor(8,1);
    lcd.println("Minimum ");
  }
  else if (lcdRezhim == 3){
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("H:");lcd.print(huMax,0); lcd.print("%");
    lcd.setCursor(8,0);
    lcd.print("T:"); lcd.print(teMax,1);
    lcd.write(byte(0)); // печать знака градуса Цельсия
    lcd.print("C");
    lcd.setCursor(0,0);
    lcd.print("P:");lcd.print(prMax,0); lcd.print("mm");
    lcd.setCursor(8,1);
    lcd.println("Maximum ");
  }
}

// ***************************************************
void IzmerBatarei(){
  sensorValue = analogRead(analogInPin);
  outputValue = sensorValue * k ;
  Serial.print(sensorValue); Serial.println(outputValue);
  lcd.setCursor(0,0);
  lcd.print("U bat = "); lcd.print(outputValue);
  delay(3000);
}
// ***************************************************
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
// ***************************************************
void CheckMaxMin(float t, float p, float h) {
  if (teMin > t){
    teMin = t;
  }
  else if (teMax < t){
    teMax = t;
  }

  if (prMin > p){
    prMin = p;
  }
  else if (prMax < p){
    prMax = p;
  }

  if (huMin > h){
    huMin = h;
  }
  else if (huMax < h){
    huMax = h;
  }
}
// ***************************************************
void resetMaxMin(float t, float p, float h) {
  teMin = t;
  teMax = t;
  prMin = p;
  prMax = p;
  huMin = h;
  huMax = h;

  OtobrLCD(t, p, h);
}

void CountTime( ){  // вызывается кажудую секунду,
  //                   считает время и переводит в строку  
  String ch_dney = "";
  String ch_chasov = "";
  String ch_minut = "";
  String ch_sekund = "";
  sekund++;
  //Serial.println(sekund);
  if (sekund > 59) {
    sekund = 0;
    minut++;
  }
  if (minut > 59) {
    minut = 0;
    chasov++;
  }
  if (chasov > 23) {
    chasov = 0;
    dney++;
  }
  if ( sekund < 10 ) {ch_sekund = '0';} // добавление лидирующего нуля
  if ( minut < 10 ) {ch_minut = '0';}
  if ( chasov < 10 ) {ch_chasov = '0';}
  ch_dney = String(dney, DEC);
  ch_chasov += String(chasov, DEC);
  ch_minut += String(minut, DEC);
  ch_sekund += String(sekund, DEC);
  stroka = ch_dney + 'd' + ch_chasov + ':' + ch_minut + ':' + ch_sekund;
}

void PrintTime() {
    lcd.setCursor(16-(stroka.length()),1);
    lcd.println (stroka);  // отображение времени с момента запуска прибора
}
