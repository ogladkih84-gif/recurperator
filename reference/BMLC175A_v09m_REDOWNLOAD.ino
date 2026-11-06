/*
  BMLC-175A: v0.9 + команда m (диагностическая модификация v0.9m)
  Arduino Nano / ATmega328P, 16 МГц.

  ОСНОВА: неизменённые параметры и шестишаговая коммутация v0.9:
  - ALIGN 250 мс;
  - RAMP 6000 мс, линейный рост частоты от шага 100 мс до 20 мс;
  - HOLD 2000 мс;
  - PWM ~31,37 кГц на нижних MOSFET, 20..30% по A4;
  - реверс: кнопка D8 -> GND, считывается при запуске.

  ДОБАВЛЕНО:
  - m: при ВСЕХ выключенных MOSFET записать 160 наборов U,V,W,N
       с периодом 6 мс; длительность ~0,96 с;
  - d: напечатать запись в CSV.

  Serial Monitor: 115200 бод, окончание строки Newline.
  Команды: t, s, ?, i20, i15, i10, m, d.

  ВНИМАНИЕ: нет измерения тока и обнаружения срыва синхронизации.
  Нужен ограниченный по току лабораторный БП и предохранитель.
  Перед m проверьте, что напряжения A0..A3 в пределах 0..5 В.
*/

#include <Arduino.h>      // Стандартные функции Arduino, millis(), analogRead() и т. д.
#include <stdlib.h>       // strtol(): преобразование текстового числа.
#include <string.h>       // strcmp(): сравнение команд.

// ==================== 1. РАСПИНОВКА ====================

const uint8_t UH = 7;    // Верхний P-MOS фазы U через NPN: HIGH = включить.
const uint8_t VH = 4;    // Верхний P-MOS фазы V через NPN.
const uint8_t WH = 2;    // Верхний P-MOS фазы W через NPN.

const uint8_t UL = 11;   // Нижний N-MOS фазы U; вывод PWM Timer2.
const uint8_t VL = 10;   // Нижний N-MOS фазы V; вывод PWM Timer1.
const uint8_t WL = 9;    // Нижний N-MOS фазы W; вывод PWM Timer1.

const uint8_t POT = A4;  // Потенциометр ШИМ; если его нет, A4 на GND.
const uint8_t REV = 8;   // Кнопка реверса на GND, внутренняя подтяжка к +5 В.

// ИМЕННО ВАШЕ ПОСЛЕДНЕЕ переназначение измерительных входов.
const uint8_t BEMF_U = A3; // Делитель фазы U.
const uint8_t BEMF_V = A2; // Делитель фазы V.
const uint8_t BEMF_W = A1; // Делитель фазы W.
const uint8_t BEMF_N = A0; // Искусственная нейтраль через резисторы.

// ==================== 2. НАСТРОЙКИ v0.9 ====================

const uint8_t POLE_PAIRS = 2;         // 4 магнитных полюса ротора = 2 пары.
const uint16_t DEAD_US = 100;        // Пауза между отключением и включением ключей.
const uint32_t ALIGN_MS = 250;       // Позиционирование перед разгоном.
const uint32_t RAMP_MS = 6000;       // Продолжительность принудительного разгона.
const uint32_t HOLD_MS = 2000;       // Продолжительность удержания.
const uint32_t START_STEP_US = 100000UL; // Стартовый шаг 100 мс, как в v0.9.
uint16_t targetStepMs = 20;          // Финишный шаг 20 мс, как в v0.9.
const uint16_t MIN_STEP_MS = 10;     // Минимальная настройка командой i.
const uint16_t MAX_STEP_MS = 40;     // Максимальная настройка командой i.
const uint8_t PWM_MIN = 51;         // 51/255 = 20%.
const uint8_t PWM_MAX = 77;         // 77/255 ~ 30%.

// ==================== 3. НОВЫЙ РЕЖИМ m ====================

const uint8_t COAST_SAMPLES = 160;           // Ровно 160 строк измерений.
const uint32_t COAST_PERIOD_US = 6000UL;     // Примерно одна строка каждые 6 мс.
const uint32_t COAST_BLANK_US = 2000UL;      // 2 мс после отключения моста.

// 4 * uint16_t = 8 байт на запись; буфер занимает 1280 байт из 2048 SRAM.
struct CoastRecord {
  uint16_t u; // ADC 0..1023 для фазы U.
  uint16_t v; // ADC 0..1023 для фазы V.
  uint16_t w; // ADC 0..1023 для фазы W.
  uint16_t n; // ADC 0..1023 для виртуальной нейтрали.
};

CoastRecord coastLog[COAST_SAMPLES]; // Буфер, заполнится после команды m.
uint8_t coastCount = 0;              // Сколько записей действительно получено.
uint32_t coastStartUs = 0;           // Момент начала измерительного режима.
uint32_t nextCoastUs = 0;            // Время следующего набора измерений.

// ==================== 4. ТАБЛИЦА КОММУТАЦИИ ====================

// Индексы выходов: 0=UH, 1=UL, 2=VH, 3=VL, 4=WH, 5=WL.
const uint8_t pins[6] = { UH, UL, VH, VL, WH, WL };

// 1 = ключ должен быть включён; 0 = выключен.
// Одновременно активны ровно один верхний и один нижний ключ разных фаз.
const uint8_t steps[6][6] = {
  {1,0,0,1,0,0}, // Шаг 1: U+ V-, фаза W свободна.
  {1,0,0,0,0,1}, // Шаг 2: U+ W-, фаза V свободна.
  {0,0,1,0,0,1}, // Шаг 3: V+ W-, фаза U свободна.
  {0,1,1,0,0,0}, // Шаг 4: V+ U-, фаза W свободна.
  {0,1,0,0,1,0}, // Шаг 5: W+ U-, фаза V свободна.
  {0,0,0,1,1,0}  // Шаг 6: W+ V-, фаза U свободна.
};

// ==================== 5. СОСТОЯНИЯ ====================

enum MotorState {
  STOPPED,   // Все силовые ключи выключены.
  ALIGNING,  // Позиционирование одним коммутационным шагом.
  RAMPING,   // Принудительное увеличение частоты шагов.
  HOLDING,   // Фиксированная частота шагов.
  CAPTURING  // НОВОЕ: ключи выключены, идёт запись обратной ЭДС.
};

MotorState motorState = STOPPED; // После включения питания двигатель остановлен.
uint8_t currentStep = 0;        // Индекс текущего шага: 0..5.
uint8_t active[6] = {0,0,0,0,0,0}; // Какие ключи считаются включёнными.
uint8_t pwmDuty = PWM_MIN;      // Текущая скважность нижнего MOSFET.
bool reverseDir = false;        // Направление, зафиксированное при старте.

uint32_t stateStartMs = 0;      // Начало ALIGN, RAMP или HOLD в миллисекундах.
uint32_t lastStepUs = 0;        // Время последней коммутации в микросекундах.
uint32_t lastPotMs = 0;         // Время последнего чтения A4.
uint32_t lastInfoMs = 0;        // Время последнего сообщения разгона.
uint32_t currentIntervalUs = START_STEP_US; // Текущий интервал между шагами.

char commandBuffer[20];         // Буфер команды Serial.
uint8_t commandLength = 0;      // Число принятых символов команды.

// ==================== 6. НАСТРОЙКА PWM ====================

void setupPWM() {
  TCCR1A = _BV(WGM10); // Timer1: 8-битный Phase Correct PWM.
  TCCR1B = _BV(CS10);  // Timer1: делитель = 1; D9 и D10 ~31,37 кГц.
  TCCR2A = _BV(WGM20); // Timer2: 8-битный Phase Correct PWM.
  TCCR2B = _BV(CS20);  // Timer2: делитель = 1; D11 ~31,37 кГц.
  OCR1A = 0;           // Исходная скважность D9 = 0.
  OCR1B = 0;           // Исходная скважность D10 = 0.
  OCR2A = 0;           // Исходная скважность D11 = 0.
}

// ==================== 7. СИЛОВЫЕ ВЫХОДЫ ====================

void allOff() {
  analogWrite(UL, 0);           // Принудительно закрыть нижний MOSFET U.
  analogWrite(VL, 0);           // Закрыть нижний MOSFET V.
  analogWrite(WL, 0);           // Закрыть нижний MOSFET W.
  digitalWrite(UH, LOW);        // Убрать управление верхнего P-MOS U.
  digitalWrite(VH, LOW);        // Убрать управление верхнего P-MOS V.
  digitalWrite(WH, LOW);        // Убрать управление верхнего P-MOS W.
  for (uint8_t i = 0; i < 6; i++) active[i] = 0; // Обнулить карту ключей.
}

void disableOutput(uint8_t i) {
  if (i & 1) {                   // Нечётный индекс соответствует нижнему ключу.
    analogWrite(pins[i], 0);    // Для нижнего ключа отключить PWM.
  } else {
    digitalWrite(pins[i], LOW); // Для верхнего ключа установить LOW.
  }
  active[i] = 0;                // Отметить ключ как выключенный.
}

void enableOutput(uint8_t i) {
  if (i & 1) {                      // Нечётный индекс: нижний MOSFET.
    analogWrite(pins[i], pwmDuty); // Подать PWM с заданной скважностью.
  } else {
    digitalWrite(pins[i], HIGH);   // Чётный индекс: включить верхний P-MOS.
  }
  active[i] = 1;                   // Отметить ключ как включённый.
}

bool validStep(uint8_t s) {
  if (s >= 6) return false;       // Защититься от выхода за таблицу.
  uint8_t highCount = 0;          // Счётчик включённых верхних ключей.
  uint8_t lowCount = 0;           // Счётчик включённых нижних ключей.
  for (uint8_t i = 0; i < 6; i += 2) { // Пройти пары U, V, W.
    if (steps[s][i] && steps[s][i + 1]) return false; // Нет сквозного тока по команде.
    highCount += steps[s][i];     // Подсчитать верхние ключи.
    lowCount += steps[s][i + 1];  // Подсчитать нижние ключи.
  }
  return highCount == 1 && lowCount == 1; // Ровно 1 верхний и 1 нижний.
}

void applyStep(uint8_t s) {
  if (!validStep(s)) {            // Проверить таблицу прежде, чем включить ключи.
    allOff();                     // При ошибке немедленно снять управление.
    motorState = STOPPED;         // Запретить дальнейшую коммутацию.
    Serial.println(F("FAULT: INVALID STEP")); // Сообщение об ошибке.
    return;                       // Не выполнять опасный шаг.
  }
  for (uint8_t i = 0; i < 6; i++) { // Сначала пройти все прежние ключи.
    if (active[i] && !steps[s][i]) disableOutput(i); // Отключить ненужные.
  }
  delayMicroseconds(DEAD_US);     // Пауза на закрывание MOSFET.
  for (uint8_t i = 0; i < 6; i++) { // Затем пройти требуемые ключи.
    if (!active[i] && steps[s][i]) enableOutput(i); // Включить новые.
  }
}

void nextStep() {
  if (!reverseDir) {                          // Прямое направление.
    currentStep = (currentStep + 1) % 6;      // 0,1,2,3,4,5,0...
  } else {                                    // Обратное направление.
    currentStep = (currentStep + 5) % 6;      // 0,5,4,3,2,1,0...
  }
  applyStep(currentStep);                     // Физически переключить MOSFET.
}

// ==================== 8. PWM ПО ПОТЕНЦИОМЕТРУ ====================

void updatePWM() {
  int adc = analogRead(POT);                  // Прочитать A4: 0..1023.
  uint8_t newDuty = (uint8_t)map(adc, 0, 1023, PWM_MIN, PWM_MAX); // 20..30%.
  if (newDuty == pwmDuty) return;             // Ничего не менять, если то же значение.
  pwmDuty = newDuty;                          // Сохранить новую скважность.
  if (motorState == STOPPED || motorState == CAPTURING) return; // В m не включать ключи!
  for (uint8_t i = 1; i < 6; i += 2) {       // Проверить три нижних ключа.
    if (active[i]) analogWrite(pins[i], pwmDuty); // Обновить только активный.
  }
}

// ==================== 9. ЗАКОН РАЗГОНА v0.9 ====================

uint32_t rampIntervalUs(uint32_t elapsedMs) {
  if (elapsedMs >= RAMP_MS) return (uint32_t)targetStepMs * 1000UL; // Конец разгона.
  const uint32_t startFreqMilliHz = 1000000000UL / START_STEP_US; // 10 Гц = 10000 мГц.
  const uint32_t endFreqMilliHz = 1000000UL / targetStepMs;       // Для 20 мс = 50 Гц.
  uint32_t freqMilliHz = startFreqMilliHz +
    (endFreqMilliHz - startFreqMilliHz) * elapsedMs / RAMP_MS; // Линейный рост ЧАСТОТЫ.
  return 1000000000UL / freqMilliHz; // Обратно перевести частоту в мкс/шаг.
}

// ==================== 10. НОВОЕ: ЗАПИСЬ ЭДС ====================

uint16_t readADCStable(uint8_t pin) {
  analogRead(pin);          // Первое чтение после смены канала отбросить.
  return analogRead(pin);   // Второе чтение использовать для журнала.
}

void beginCoastCapture() {
  allOff();                 // ГЛАВНОЕ: гарантированно снять все управляющие сигналы.
  coastCount = 0;           // Удалить старую запись.
  coastStartUs = micros();  // Зафиксировать нулевой момент измерений.
  nextCoastUs = coastStartUs + COAST_BLANK_US; // Дать мосту 2 мс успокоиться.
  motorState = CAPTURING;   // В этом состоянии коммутация запрещена.
  Serial.println(F("COAST CAPTURE: ALL MOSFETS OFF")); // Сигнал о старте записи.
}

void captureCoastSample() {
  if (coastCount >= COAST_SAMPLES) return; // Не выйти за массив.
  CoastRecord &r = coastLog[coastCount];   // Получить ссылку на новую строку.
  r.u = readADCStable(BEMF_U);            // Прочитать U через делитель A3.
  r.v = readADCStable(BEMF_V);            // Прочитать V через делитель A2.
  r.w = readADCStable(BEMF_W);            // Прочитать W через делитель A1.
  r.n = readADCStable(BEMF_N);            // Прочитать виртуальную нейтраль A0.
  coastCount++;                           // Отметить ещё одну готовую запись.
}

void dumpCoastLog() {
  if (motorState != STOPPED) {             // Дамп запрещён при активной работе.
    Serial.println(F("STOP FIRST"));
    return;
  }
  if (coastCount == 0) {                   // Не отправлять пустой CSV.
    Serial.println(F("NO COAST DATA: SEND m"));
    return;
  }
  Serial.println();                        // Отступ перед таблицей.
  Serial.println(F("BEMF COAST LOG v0.9m")); // Заголовок.
  Serial.println(F("ms,U,V,W,N,U-N,V-N,W-N")); // Названия столбцов.
  for (uint8_t i = 0; i < coastCount; i++) { // Пройти все накопленные записи.
    const CoastRecord &r = coastLog[i];    // Ссылка на текущую запись.
    uint32_t ms = (COAST_BLANK_US + (uint32_t)i * COAST_PERIOD_US) / 1000UL;
    // Время выше — плановое, с шагом 6 мс, не индивидуальная аппаратная метка.
    Serial.print(ms); Serial.print(',');   // Миллисекунды от старта m.
    Serial.print(r.u); Serial.print(',');  // Напряжение U в отсчётах АЦП.
    Serial.print(r.v); Serial.print(',');  // Напряжение V.
    Serial.print(r.w); Serial.print(',');  // Напряжение W.
    Serial.print(r.n); Serial.print(',');  // Напряжение N.
    Serial.print((int)r.u - (int)r.n); Serial.print(','); // Разность U-N.
    Serial.print((int)r.v - (int)r.n); Serial.print(','); // Разность V-N.
    Serial.println((int)r.w - (int)r.n); // Разность W-N и конец строки.
  }
  Serial.println(F("END LOG"));          // Маркер завершения таблицы.
}

// ==================== 11. СЛУЖЕБНЫЕ КОМАНДЫ ====================

void printSettings() {
  Serial.println();
  Serial.println(F("BMLC-175A v0.9m (v0.9 + coast m)"));
  Serial.print(F("Final step: ")); Serial.print(targetStepMs); Serial.println(F(" ms"));
  Serial.print(F("Target sync RPM: "));
  Serial.println(60000UL / (6UL * POLE_PAIRS * targetStepMs)); // Только расчётная скорость поля.
  Serial.print(F("PWM duty: ")); Serial.print((uint16_t)pwmDuty * 100UL / 255UL);
  Serial.println(F("%"));
  Serial.print(F("Ramp: ")); Serial.print(RAMP_MS); Serial.println(F(" ms"));
  Serial.print(F("Hold: ")); Serial.print(HOLD_MS); Serial.println(F(" ms"));
  Serial.println(F("Commands: t s i20 i15 i10 m d ?"));
  Serial.println();
}

void stopMotor() {
  if (motorState == STOPPED) { allOff(); return; } // Повторная остановка безопасна.
  allOff();                                       // Обязательно выключить все ключи.
  motorState = STOPPED;                           // Остановить алгоритм.
  Serial.println(F("MOTOR STOP"));               // Сообщить оператору.
}

void startMotor() {
  if (motorState != STOPPED) {                    // Во время m запуск запрещён.
    Serial.println(F("ALREADY RUNNING"));
    return;
  }
  allOff();                                       // Сброс предыдущих состояний.
  reverseDir = (digitalRead(REV) == LOW);        // Запомнить направление.
  currentStep = 0;                               // Шаг 1: U+ V-.
  currentIntervalUs = START_STEP_US;             // Начало с 100 мс/шаг.
  updatePWM();                                    // Считать установленную скважность.
  applyStep(currentStep);                        // Включить позиционирующий шаг.
  stateStartMs = millis();                       // Начать отсчёт ALIGN.
  lastPotMs = stateStartMs;                       // Время обновления PWM.
  lastInfoMs = stateStartMs;                      // Время сообщения статуса.
  motorState = ALIGNING;                         // Перейти в позиционирование.
  Serial.println(F("ALIGN START"));
}

void processCommand(char *cmd) {
  if (strcmp(cmd, "t") == 0) { startMotor(); return; } // Обычный пуск v0.9.
  if (strcmp(cmd, "s") == 0) { stopMotor(); return; }  // Остановка также прерывает m.
  if (strcmp(cmd, "?") == 0) { printSettings(); return; } // Показать параметры.
  if (strcmp(cmd, "m") == 0) {                         // НОВАЯ команда записи ЭДС.
    if (motorState != STOPPED) {                       // Разрешено только после STOP.
      Serial.println(F("STOP MOTOR FIRST"));
    } else {
      beginCoastCapture();                            // Без единого силового импульса.
    }
    return;
  }
  if (strcmp(cmd, "d") == 0) { dumpCoastLog(); return; } // Выдать CSV.
  if (cmd[0] == 'i') {                                 // Прежняя настройка шага v0.9.
    if (motorState != STOPPED) {
      Serial.println(F("STOP MOTOR FIRST"));
      return;
    }
    char *p = cmd + 1;                                // Всё после буквы i.
    while (*p == ' ') p++;                            // Пропустить пробелы.
    if (*p == '\0') { Serial.println(F("USE i10..i40")); return; }
    char *endPtr = NULL;                              // Куда strtol запишет конец числа.
    long requested = strtol(p, &endPtr, 10);          // Прочитать десятичное число.
    while (*endPtr == ' ') endPtr++;                  // Разрешить пробелы в конце.
    if (*endPtr != '\0' || requested < MIN_STEP_MS || requested > MAX_STEP_MS) {
      Serial.println(F("INTERVAL RANGE: 10..40 ms"));
      return;
    }
    targetStepMs = (uint16_t)requested;                // Сохранить новую уставку.
    Serial.print(F("NEW TARGET: ")); Serial.print(targetStepMs); Serial.println(F(" ms"));
    printSettings();                                  // Показать новые параметры.
    return;
  }
  Serial.println(F("UNKNOWN COMMAND"));             // Неизвестная команда.
}

void handleSerial() {
  while (Serial.available()) {                       // Пока в UART есть байты.
    char c = Serial.read();                          // Прочитать один байт.
    if (c == '\r' || c == '\n') {                   // Получен конец строки.
      if (commandLength > 0) {                      // Не обрабатывать пустые строки.
        commandBuffer[commandLength] = '\0';         // Завершить строку нулём.
        processCommand(commandBuffer);               // Выполнить команду.
        commandLength = 0;                           // Освободить буфер.
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;             // Добавить очередной символ.
    } else {
      commandLength = 0;                             // Сбросить слишком длинную команду.
      Serial.println(F("COMMAND TOO LONG"));
    }
  }
}

// ==================== 12. SETUP ====================

void setup() {
  for (uint8_t i = 0; i < 6; i++) {                 // Поочерёдно настроить выходы моста.
    digitalWrite(pins[i], LOW);                     // Сначала безопасный LOW.
    pinMode(pins[i], OUTPUT);                       // Затем переключить на выход.
  }
  pinMode(POT, INPUT);                             // A4: аналоговый вход.
  pinMode(REV, INPUT_PULLUP);                      // D8: кнопка реверса к земле.
  allOff();                                        // Принудительно отключить мост.
  setupPWM();                                      // Подготовить таймеры 1 и 2.
  analogReference(DEFAULT);                       // Опора АЦП: AVcc, обычно +5 В.
  Serial.begin(115200);                            // Последовательный порт.
  updatePWM();                                     // Считать текущую ручку до запуска.
  printSettings();                                 // Показать стартовые параметры.
}

// ==================== 13. ОСНОВНОЙ ЦИКЛ ====================

void loop() {
  handleSerial();                                  // Всегда принимать команды.
  if (motorState == STOPPED) return;              // Пока ничего не запущено — выход.

  if (motorState == CAPTURING) {                  // Новый режим независимой записи.
    uint32_t nowUs = micros();                   // Узнать текущее время в мкс.
    if (coastCount >= COAST_SAMPLES) {            // Собрали все 160 записей?
      motorState = STOPPED;                       // Оставить мост выключенным.
      Serial.println(F("CAPTURE COMPLETE"));
      Serial.println(F("Send d for CSV"));
      return;
    }
    if ((int32_t)(nowUs - nextCoastUs) >= 0) {   // Наступило время выборки?
      captureCoastSample();                       // Прочитать четыре канала АЦП.
      nextCoastUs += COAST_PERIOD_US;             // Назначить следующую выборку.
    }
    return;                                       // Не заходить в управление двигателем!
  }

  uint32_t nowMs = millis();                      // Текущее время для моторных состояний.
  if ((uint32_t)(nowMs - lastPotMs) >= 20) {     // Раз в 20 мс обновлять PWM.
    lastPotMs = nowMs;                            // Запомнить момент обновления.
    updatePWM();                                  // Прочитать потенциометр A4.
  }

  if (motorState == ALIGNING) {                  // Стадия позиционирования.
    if ((uint32_t)(nowMs - stateStartMs) >= ALIGN_MS) {
      motorState = RAMPING;                       // Позиционирование завершено.
      stateStartMs = millis();                    // Нулевой момент разгона.
      lastStepUs = micros();                      // Нулевой момент коммутации.
      lastInfoMs = millis();                      // Нулевой момент сообщений.
      currentIntervalUs = START_STEP_US;         // Первый шаг через 100 мс.
      Serial.println(F("RAMP START"));
    }
    return;                                       // Во время ALIGN шаги не меняются.
  }

  if (motorState == RAMPING) {                   // Стадия принудительного разгона.
    uint32_t elapsed = (uint32_t)(nowMs - stateStartMs); // Сколько мс прошло.
    if (elapsed >= RAMP_MS) {                    // Шесть секунд закончились?
      motorState = HOLDING;                      // Перейти к удержанию.
      stateStartMs = millis();                   // Отсчёт двух секунд HOLD.
      lastStepUs = micros();                     // ИСХОДНОЕ поведение v0.9 (сброс).
      currentIntervalUs = (uint32_t)targetStepMs * 1000UL; // Интервал HOLD.
      Serial.println(F("HOLD START"));
      return;
    }
    currentIntervalUs = rampIntervalUs(elapsed); // Рассчитать интервал текущего шага.
    uint32_t nowUs = micros();                   // Время с микросекундной точностью.
    if ((uint32_t)(nowUs - lastStepUs) >= currentIntervalUs) {
      lastStepUs = nowUs;                        // ИСХОДНОЕ поведение v0.9.
      nextStep();                                // Переключить коммутацию.
    }
    if ((uint32_t)(nowMs - lastInfoMs) >= 1000) {
      lastInfoMs = nowMs;                        // Отчёт раз в секунду.
      Serial.print(F("RAMP step_us="));
      Serial.println(currentIntervalUs);        // Показать требуемый интервал.
    }
    return;
  }

  if (motorState == HOLDING) {                   // Удержание конечной частоты.
    uint32_t elapsed = (uint32_t)(nowMs - stateStartMs); // Время HOLD.
    if (elapsed >= HOLD_MS) {                    // Две секунды истекли?
      stopMotor();                               // Выключить все MOSFET.
      return;
    }
    uint32_t nowUs = micros();                   // Время для следующего шага.
    if ((uint32_t)(nowUs - lastStepUs) >= currentIntervalUs) {
      lastStepUs = nowUs;                        // ИСХОДНОЕ поведение v0.9.
      nextStep();                                // Перейти к следующему сектору.
    }
    return;
  }
}
