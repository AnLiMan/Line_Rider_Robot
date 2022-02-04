//-----Библиотеки---------
#include <avr/eeprom.h> // Для работы с EEPROM памятью

//-------------------Все постоянные-----------------

//---Постоянные автомобиля---
#define L 0.02 //Расстояние между передним и задним колесом (м)
#define dr 0.01 // Расстояние между задними колесами (м)
#define lr 0.01 // Расстояние между задним колесом и центром тяжести (м)
#define r 0.003 // Радиус колеса (м)
#define K 0.01 // Расстояние между правым и левым kingpin (м)

#define MinAngleSensetivity -5 // Минимальный угол нечувствительности руля в градусах
#define MaxAngleSensetivity 5 // Максимальный угол нечувствительности руля в градусах

#define BlinkPeriodForTurnSignal 200 //Период моргания поворотников
#define MaxModes 4 //Количество режимов
#define idleSpeedLineAndMemory 105 // Скорость для режима работы следования по линии и движения по памяти

//-----Выводы------

#define Button1 0 // Кнопка 1 (перключение режимов)
#define Button2 7 // Кнопка 2 (разрешение работы)

#define EncoderRight 2 // Энкодер правого двигателя
#define EncoderLeft 3 // Энкодер левого двигателя

#define LeftTurnSignal 4 // Левый поворотник
#define RightTurnSignal 5 // Правый поворотник
#define RearDimensionsAndHeadlights 6 // Задние габариты и передние фары

#define LeftMotor 9 // Левый двигатель
#define RightMotor 10 // Правый двигатель
#define RightMotorAdj 0.956 // Подстройка правого двигателя

#define BlueChannelRGB 11 // Синий канал RGB-светодиода
#define GrennChannelRGB 12 // Зелёный канал RGB-светодиода
#define RedChannelRGB 13 // Красный канал RGB-светодиода

#define StearingValue A0 // Сигнал с потенциометра руля
#define Accelerator A1 // Установка скорости (педаль газа)

#define InfraredSensorLeft A2 // Инфракрасный датчик слева 
#define InfraredSensorRight A3 // Инфракрасный датчик справа
#define InfraredSensorStraight A4 // Инфракрасный датчик по середине

//---------------Все переменные------------

bool Buttflag1 = false; //Флаг нажатия кнопки 1
bool Buttflag2 = false; //Флаг нажатия кнопки 2
volatile int counter = 0;  // Переменная-счётчик для переключения режимов
volatile bool OnWork = false;  // Переменная разрешения на работу

//Для электронного дифференциала
float Omega1; //Скорость вращения 1
float Omega2; //Скорость вращения 2
float LinearIdleSpeed = 0.25; //Скорость машинки, линейная (м/c) по дефолту указана максимальная
int idleSpeed; //Скорость движения машинки

uint32_t myTimer1; // Переменная хранения времени 1
uint32_t myTimer2; // Переменная хранения времени 2

//Для энкодеров
volatile uint32_t LeftMotorTurnCounts; //Количество тиков с левого энкодера
volatile uint32_t RightMotorTurnCounts; //Количество тиков с правого энкодера

int DeltaT = 100; // Переменная частоты снятия показаний с энкодеров (миллисекунды)
byte ArrayOfTheWay[] = {0, 0, 70, 70, 70, 70, 70, 50, 70, 50, 70, 70, 50, 70, 70, 0, 70, 50, 70, 50, 70, 70, 0, 70, 70, 0, 70, 70};
int NumberOfEntriesInEEPROM = 0; // Количество записей в EPPROM-память
bool StartRecord = false; //Перемнная разрешения записи
byte RightMotorTurnCountsEEPROM; // Значение для записи в EEPROM для правого колеса
byte LeftMotorTurnCountsEEPROM; // Значение для записи в EEPROM для левого колеса

/* Переменная частоты снятия показаний с энкодеров "DeltaT", напрямую влияет на то, как долго мы сможем писать
  значения в EEPROM-память, формулы для расчёта приведены ниже
  tmax = 1021 / 1сек - Максимальное время записи
  1сек = 1 / DeltaT * 2 - "Стоимость записи одной секнды"
  Пара готовых значений:
  DeltaT = 100, tmax = 51 сек
  DeltaT = 250, tmax = 128 сек
  DeltaT = 500, tmax = 256 сек
*/

//----------------------//
//--------Setup-------
//---------------------//

void setup() {

  //Serial.begin(9600); //Для отладки

  //Законфигурируем порты
  pinMode(Button1, INPUT_PULLUP);
  pinMode(Button2, INPUT_PULLUP);
  pinMode(LeftTurnSignal, OUTPUT);
  pinMode(RightTurnSignal, OUTPUT);
  pinMode(RearDimensionsAndHeadlights, OUTPUT);
  pinMode(LeftMotor, OUTPUT);
  pinMode(RightMotor, OUTPUT);
  pinMode(BlueChannelRGB, OUTPUT);
  pinMode(GrennChannelRGB, OUTPUT);
  pinMode(RedChannelRGB, OUTPUT);

  // Расгоняем ШИМ на пинах D9 и D10 до 31.4 кГц, разрядность 8 бит
  TCCR1A = 0b00000001;  // 8bit
  TCCR1B = 0b00000001;  // x1 phase correct

  //Выключаем двигатели
  digitalWrite(LeftMotor, LOW);
  digitalWrite(RightMotor, LOW);

  //Подключаем прерывания
  attachInterrupt(digitalPinToInterrupt(Button1), Button1Tick, FALLING); //Подключаем прерывание для кнопки 1
  attachInterrupt(digitalPinToInterrupt(Button2), Button2Tick, FALLING); //Подключаем прерывание для кнопки 2
  attachInterrupt(digitalPinToInterrupt(EncoderLeft), EncoderLeftTick, RISING); //Подключаем прерывание для левого энкодера
  attachInterrupt(digitalPinToInterrupt(EncoderRight), EncoderRightTick, RISING); //Подключаем прерывание для правого энкодера
}

//---------------------------//
//-------Основной цикл--------
//---------------------------//

void loop() {
  Buttflag1 = false; //Сброс флага кнопки 1
  Buttflag2 = false; //Сброс флага кнопки 2
  CurrentMode(counter); //Отображение текущего режима

  //Работа разрешена
  if (OnWork) {
    //Включение разных режимов работы, в зависимости от значения текущей переменной
    switch (counter) {
      //1.1. Ручное управление. Индикатор - красный цвет светодиода
      case 0:
        digitalWrite(RearDimensionsAndHeadlights, HIGH);
        ManualControl();
        break;
      // 1.2. Движение по линии. Индикатор - зелёный цвет светодиода
      case 1:
        digitalWrite(RearDimensionsAndHeadlights, HIGH);
        idleSpeed = map(analogRead(Accelerator), 0, 1022, 0, 255); // Преобразуем входное значение сопротивления к новому диапазону
        LineRider();
        break;
      //1.3. Движение по памяти. Индикатор - синий цвет светодиода
      case 2:
        idleSpeed = map(analogRead(Accelerator), 0, 1022, 0, 255); // Преобразуем входное значение сопротивления к новому диапазону
        digitalWrite(RearDimensionsAndHeadlights, HIGH); // Включим индикацию
        RideByMemory();
        break;
      //1.4. Запись в память. Индикатор - мигающий синий
      case 3:
        digitalWrite(RearDimensionsAndHeadlights, HIGH);
        idleSpeed = map(analogRead(Accelerator), 0, 1022, 0, 255); // Преобразуем входное значение сопротивления к новому диапазону
        WriteWayInMemory();
        break;
    }
  }
  //Если работа не разрешена, переходим в режим ожидания
  else {
    StandBy();
  }
}

//--------------------------------------------------------------//
//-----------------1. Основные режимы работы-------------
//--------------------------------------------------------------//

//------------1.1. Ручное управление-----------------
void ManualControl() {
  idleSpeed = map(analogRead(Accelerator), 0, 1022, 0, 255); // Преобразуем входное значение сопротивления к новому диапазону
  int SteeringAngle = map(analogRead(StearingValue), 1023, 0, -60, 60); // Преобразуем входное значение сопротивления к новому диапазону

  //Если едем прямо
  if (MinAngleSensetivity < SteeringAngle && SteeringAngle < MaxAngleSensetivity) {
    analogWrite(RightMotor, int (RightMotorAdj * idleSpeed));
    analogWrite(LeftMotor, idleSpeed);
    //Выключаем поворотники
    digitalWrite(LeftTurnSignal, LOW);
    digitalWrite(RightTurnSignal, LOW);
  }

  //Поворот вправо
  else if (SteeringAngle > MaxAngleSensetivity) {
    analogWrite(LeftMotor, idleSpeed);
    ElectronicDiff(SteeringAngle); //Расчитаем скорости
    int RightMotorSpeed = idleSpeed * (Omega1 / Omega2) * RightMotorAdj; //Корректируем скорость правого двигателя на Omega1/Omega2
    analogWrite(RightMotor, RightMotorSpeed);
    BlinkTurnLight(LeftTurnSignal, RightTurnSignal); //Добавим индикации
  }

  //Поворот налево
  else if (SteeringAngle < MinAngleSensetivity) {
    analogWrite(RightMotor, int (idleSpeed * RightMotorAdj));
    ElectronicDiff(SteeringAngle); //Расчитаем скорости
    int LeftMotorSpeed = idleSpeed *  (Omega1 / Omega2); //Корректируем скорость левого двигателя на Omega1/Omega2
    analogWrite(LeftMotor, LeftMotorSpeed);
    BlinkTurnLight(RightTurnSignal, LeftTurnSignal); //Добавим индикации
  }
}

//------------1.2. Движение по линии-----------------
void LineRider() {

  //Считаем значения с датчиков
  byte InfraredSensorRightState = digitalRead(InfraredSensorRight); // 1 - белый, чёрный - 0
  byte InfraredSensorLeftState = digitalRead(InfraredSensorLeft); // 1 - белый, чёрный - 0
  byte InfraredSensorStraightState = digitalRead(InfraredSensorStraight); // 1 - белый, чёрный - 0

  // Если датчик посередине на черной линии
  if (InfraredSensorStraightState == 0) {
    analogWrite(LeftMotor, idleSpeed);
    analogWrite(RightMotor, int (idleSpeed * RightMotorAdj));
  }

  // Иначе
  else {
    //Поворот налево
    if (InfraredSensorRightState == 1 && InfraredSensorLeftState == 0) {
      digitalWrite(LeftMotor, 0);
    }

    //Поворот направо
    else if (InfraredSensorRightState == 0 && InfraredSensorLeftState == 1) {
      digitalWrite(RightMotor, 0);
    }

    //Оба датчика на чёрном
    else if (InfraredSensorRightState == 0 && InfraredSensorLeftState == 0) {
      analogWrite(LeftMotor, idleSpeed);
      analogWrite(RightMotor, int (idleSpeed * RightMotorAdj));
    }
  }
}

//------------1.3. Движение по памяти-----------------
void RideByMemory () {
  //idleSpeed = map(analogRead(Accelerator), 0, 1022, 0, 255); // Преобразуем входное значение сопротивления к новому диапазону

  for (int i = 0; i <= eeprom_read_word(1022);) {

    // Если едем прямо
    if (eeprom_read_byte(i) == eeprom_read_byte(i + 1)) {
      analogWrite(LeftMotor, idleSpeed);
      analogWrite(RightMotor, int (idleSpeed * RightMotorAdj));
      analogWrite(LeftMotor, idleSpeed);
      digitalWrite(LeftTurnSignal, LOW);
      digitalWrite(RightTurnSignal, LOW);
      if (millis() - myTimer2 >= DeltaT) {
        myTimer2 += DeltaT;
        i += 2;
      }
    }

    //Иначе считаем корректировку
    else {
      //Если записан поворот направо
      if (eeprom_read_byte(i) > eeprom_read_byte(i + 1)) {
        analogWrite(LeftMotor, idleSpeed);
        int RightMotorSpeed = idleSpeed * eeprom_read_byte(i + 1) / eeprom_read_byte(i) * RightMotorAdj; //Корректируем скорость правого двигателя
        analogWrite(RightMotor, RightMotorSpeed);
        digitalWrite(LeftTurnSignal, LOW);
        digitalWrite(RightTurnSignal, HIGH);
        if (millis() - myTimer2 >= DeltaT) {
          myTimer2 += DeltaT;
          i += 2;
        }
      }

      //Если записан поворот налево
      else if (eeprom_read_byte(i + 1) > eeprom_read_byte(i)) {
        analogWrite(RightMotor, int (idleSpeed * RightMotorAdj));
        int LeftMotorSpeed = idleSpeed * eeprom_read_byte(i) / eeprom_read_byte(i + 1); //Корректируем скорость левого двигателя
        analogWrite(LeftMotor, LeftMotorSpeed);
        digitalWrite(LeftTurnSignal, HIGH);
        digitalWrite(RightTurnSignal, LOW);
        if (millis() - myTimer2 >= DeltaT) {
          myTimer2 += DeltaT;
          i += 2;
        }
      }
    }
  }
}

//------------1.4. Запись в память-----------------
void WriteWayInMemory () {

  //Считаем значения с датчиков
  byte InfraredSensorRightState = digitalRead(InfraredSensorRight); // 1 - белый, чёрный - 0
  byte InfraredSensorLeftState = digitalRead(InfraredSensorLeft); // 1 - белый, чёрный - 0
  byte InfraredSensorStraightState = digitalRead(InfraredSensorStraight); // 1 - белый, чёрный - 0

  // Если датчик посередине на черной линии
  if (InfraredSensorStraightState == 0) {
    analogWrite(LeftMotor, idleSpeed);
    analogWrite(RightMotor, int (idleSpeed * RightMotorAdj));
  }

  // Иначе
  else {
    //Поворот налево
    if (InfraredSensorRightState == 1 && InfraredSensorLeftState == 0) {
      digitalWrite(LeftMotor, 0);
    }

    //Поворот направо
    else if (InfraredSensorRightState == 0 && InfraredSensorLeftState == 1) {
      digitalWrite(RightMotor, 0);
    }

    //Оба датчика на чёрном
    else if (InfraredSensorRightState == 0 && InfraredSensorLeftState == 0) {
      analogWrite(LeftMotor, idleSpeed);
      analogWrite(RightMotor, int (idleSpeed * RightMotorAdj));
      StartRecord = false;
    }
  }

  //Пришло время для записи
  if (millis() - myTimer2 >= DeltaT) {
    int Max = max(LeftMotorTurnCounts, RightMotorTurnCounts); // Найдём наибольшее значение

    // --------Смаштабируем полученые значения----

    //Если тиков с левого датчика больше
    if (Max == LeftMotorTurnCounts) {
      LeftMotorTurnCountsEEPROM = 255;
      RightMotorTurnCountsEEPROM = 255 * RightMotorTurnCounts / LeftMotorTurnCounts;
      NumberOfEntriesInEEPROM += 2;
    }

    //Если тиков с правого датчика больше
    if (Max == RightMotorTurnCounts) {
      RightMotorTurnCountsEEPROM = 255;
      LeftMotorTurnCountsEEPROM = 255 * LeftMotorTurnCounts / RightMotorTurnCounts;
      NumberOfEntriesInEEPROM += 2;
    }

    //Если одинаково
    else {
      RightMotorTurnCountsEEPROM = 255;
      LeftMotorTurnCountsEEPROM = 255;
      NumberOfEntriesInEEPROM += 2;
    }

    //Заипсываем данные в память
    if (NumberOfEntriesInEEPROM <= 1021 && StartRecord) {
      eeprom_update_byte(NumberOfEntriesInEEPROM, LeftMotorTurnCountsEEPROM); // Значение с левого энкодера (смасштабированные)
      eeprom_update_byte(NumberOfEntriesInEEPROM - 1, RightMotorTurnCountsEEPROM); // Значение с правого энкодера (смасштабированные)
      LeftMotorTurnCounts = 0;
      RightMotorTurnCounts = 0;
      myTimer2 += DeltaT;
    }

    //Если превышен лимит
    if (NumberOfEntriesInEEPROM > 1021) {
      NumberOfEntriesInEEPROM = 0;
    }

    //Зписываем количество значений
    if (StartRecord == false) {
      eeprom_update_word(1022, NumberOfEntriesInEEPROM); // Значение с правого энкодера (смасштабированные)
    }
  }
}

//--------------------------------------------------------------//
//----------------2 Функции отработки прерываний--------------
//--------------------------------------------------------------//

// 2.1. Обработка прерываний с левого энкодера
void EncoderLeftTick() {
  LeftMotorTurnCounts++;
}

// ---2.2. Обработка прерываний с правого энкодера---
void EncoderRightTick() {
  RightMotorTurnCounts++;
}

// --2.3. Отработка нажатия первой кнопки (для переключения режимов)---
void Button1Tick () {
  if (!Buttflag1) {
    Buttflag1 = true;
    counter++;
    if (counter >= MaxModes) {
      counter = 0;
    }
  }
}

//--2.4. Отработка нажатия второй кнопки (включение/отключение текущего режима)---
void Button2Tick () {
  if (!Buttflag2) {
    Buttflag2 = true;
    OnWork = !OnWork; //Инвертируем переменную разрешения работы
    StartRecord = true;
  }
}

//--------------------------------------------------------------//
//-----------------3. Иные вспомогательные функциии--------------
//--------------------------------------------------------------//

//-----3.1. Функция отображения текущего режима----
void CurrentMode(int currentMode) {
  switch (counter) {
    case 0:
      digitalWrite(RedChannelRGB, HIGH);
      digitalWrite(GrennChannelRGB, LOW);
      digitalWrite(BlueChannelRGB, LOW);
      break;
    case 1:
      digitalWrite(RedChannelRGB, LOW);
      digitalWrite(GrennChannelRGB, HIGH);
      digitalWrite(BlueChannelRGB, LOW);
      break;
    case 2:
      digitalWrite(RedChannelRGB, LOW);
      digitalWrite(GrennChannelRGB, LOW);
      digitalWrite(BlueChannelRGB, HIGH);
      break;
    case 3:
      digitalWrite(RedChannelRGB, LOW);
      digitalWrite(GrennChannelRGB, LOW);

      if (millis() - myTimer1 >= BlinkPeriodForTurnSignal) {
        myTimer1 += BlinkPeriodForTurnSignal;
        digitalWrite(BlueChannelRGB, !digitalRead(BlueChannelRGB));
      }
      break;
  }
}

//------3.2. Функция электронного дифференциала----
void ElectronicDiff(int Angle) {
  float AngleRad = Angle / 57.32; //Переведём градусы в радианы
  float tanAngle = tan(fabs(AngleRad));
  float delta1 = atan((L / (L / tanAngle - K / 2)));
  float delta2 = atan((L / (L / tanAngle + K / 2)));
  float R1 = L / sin(delta1);
  float R2 = L / sin(delta2);
  float R3 = L / tanAngle - dr / 2;
  float Rcg = sqrtf(R3 + dr * dr / 4 + lr * lr);
  Omega1 = (LinearIdleSpeed * R1) / (Rcg * r);
  Omega2 = (LinearIdleSpeed * R2) / (Rcg * r);
}

//------3.3. Функция моргания поворотниками-----
void BlinkTurnLight (int LightOff, int LightOn) {
  digitalWrite(LightOff, LOW);
  if (millis() - myTimer1 >= BlinkPeriodForTurnSignal) {
    myTimer1 += BlinkPeriodForTurnSignal;
    digitalWrite(LightOn, !digitalRead(LightOn));
  }
}

//------3.4. Функция отключения текущего режима и выход в ожидание-----
void StandBy() {
  //Выключаем свет
  digitalWrite(RearDimensionsAndHeadlights, LOW);
  digitalWrite(LeftTurnSignal, LOW);
  digitalWrite(RightTurnSignal, LOW);

  //Выключаем двигатели
  digitalWrite(LeftMotor, LOW);
  digitalWrite(RightMotor, LOW);
}
