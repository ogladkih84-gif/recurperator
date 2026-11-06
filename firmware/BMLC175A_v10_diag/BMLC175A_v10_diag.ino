/*
  BMLC-175A v1.0diag, based on the user's BMLC175A_v09m_REDOWNLOAD.ino.
  Arduino Nano classic / ATmega328P, 16 MHz. EXPERIMENTAL OPEN-LOOP TEST.
  Pin mapping, PWM 31.37 kHz, duty 20..30%, ALIGN/RAMP/HOLD durations unchanged.
  No BEMF commutation, RPM feedback or current sensing in this diagnostic version.

  Serial 115200, newline: t = original test; b = test + automatic coast capture;
  s = off; i10..i40 = target sector ms while stopped; m = capture with bridge off;
  d = dump capture; v = select opposite direction while stopped; ? = settings.
  D8 button toggles NEXT direction only while stopped (debounced).
  Opposite-direction restart requires at least 5 s after bridge shutdown.
  Operator must still verify rotor stopped: no standstill sensor is implemented.

  Changes: sector deadlines survive RAMP -> HOLD, no catch-up bursts,
  automatic coast capture b, 128 timestamped samples in 1280 bytes SRAM.
  ADC channels are sampled sequentially, not simultaneously; timestamps have
  100 us resolution and mark the start of each four-channel record.
  Use a current-limited supply; verify actual gate dead time on the scope.
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

const uint8_t COAST_SAMPLES = 128;           // 128 * 10 = 1280 байт SRAM.
const uint32_t COAST_PERIOD_US = 6000UL;     // Примерно одна строка каждые 6 мс.
const uint32_t COAST_BLANK_US = 2000UL;      // 2 мс после отключения моста.

// 5 * uint16_t = 10 байт; 128 записей занимают 1280 байт из 2048 SRAM.
struct CoastRecord {
  uint16_t time100us; // Фактическое начало выборки, единица 100 мкс.
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
uint32_t nextStepUs = 0;        // Абсолютный срок следующей коммутации.
bool captureAfterTest = false; // b: записать выбег автоматически после HOLD.
bool pendingReverse = false;   // Направление следующего теста; кнопка переключает в STOP.
bool lastButtonRaw = HIGH, stableButton = HIGH;
uint32_t buttonChangedMs = 0;
uint32_t maxLatenessUs = 0;     // Максимальное опоздание планировщика текущего теста.
uint32_t powerOffAtMs = 0;
bool hasRun = false;
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
  powerOffAtMs = millis();
  coastCount = 0;           // Удалить старую запись.
  coastStartUs = micros();  // Зафиксировать нулевой момент измерений.
  nextCoastUs = coastStartUs + COAST_BLANK_US; // Дать мосту 2 мс успокоиться.
  motorState = CAPTURING;   // В этом состоянии коммутация запрещена.
  Serial.println(F("COAST CAPTURE: ALL MOSFETS OFF")); // Сигнал о старте записи.
}

void captureCoastSample() {
  if (coastCount >= COAST_SAMPLES) return; // Не выйти за массив.
  CoastRecord &r = coastLog[coastCount];   // Получить ссылку на новую строку.
  r.time100us = (micros() - coastStartUs) / 100UL;
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
  Serial.println(F("BEMF COAST LOG v1.0diag"));
  Serial.println(F("us,U,V,W,N,U-N,V-N,W-N"));
  for (uint8_t i = 0; i < coastCount; i++) { // Пройти все накопленные записи.
    const CoastRecord &r = coastLog[i];    // Ссылка на текущую запись.
    Serial.print((uint32_t)r.time100us * 100UL); Serial.print(',');
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
  Serial.println(F("BMLC-175A v1.0diag (open loop, NO rotor feedback)"));
  Serial.print(F("Final step: ")); Serial.print(targetStepMs); Serial.println(F(" ms"));
  Serial.print(F("Target sync RPM: "));
  Serial.println(60000UL / (6UL * POLE_PAIRS * targetStepMs)); // Только расчётная скорость поля.
  Serial.print(F("PWM duty: ")); Serial.print((uint16_t)pwmDuty * 100UL / 255UL);
  Serial.println(F("%"));
  Serial.print(F("Ramp: ")); Serial.print(RAMP_MS); Serial.println(F(" ms"));
  Serial.print(F("Hold: ")); Serial.print(HOLD_MS); Serial.println(F(" ms"));
  Serial.print(F("Next direction: ")); Serial.println(pendingReverse ? F("REV") : F("FWD"));
  Serial.print(F("Max scheduler lateness_us: ")); Serial.println(maxLatenessUs);
  Serial.println(F("Commands: t b s i10..i40 m d v ?"));
  Serial.println();
}

void stopMotor() {
  captureAfterTest = false;
  if (motorState == STOPPED) { allOff(); return; } // Повторная остановка безопасна.
  allOff();                                       // Обязательно выключить все ключи.
  if (motorState != CAPTURING) powerOffAtMs = millis();
  motorState = STOPPED;                           // Остановить алгоритм.
  Serial.println(F("MOTOR STOP"));               // Сообщить оператору.
}

void startMotor() {
  if (motorState != STOPPED) {                    // Во время m запуск запрещён.
    Serial.println(F("ALREADY RUNNING"));
    return;
  }
  if (hasRun && pendingReverse != reverseDir && millis() - powerOffAtMs < 5000UL) {
    captureAfterTest = false;
    Serial.println(F("REVERSE: WAIT 5s AND VERIFY ROTOR STOPPED"));
    return;
  }
  allOff();                                       // Сброс предыдущих состояний.
  reverseDir = pendingReverse;                   // Зафиксировать выбранное направление.
  hasRun = true;
  maxLatenessUs = 0;
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
  if (strcmp(cmd, "t") == 0 || strcmp(cmd, "b") == 0) {
    if (motorState != STOPPED) { Serial.println(F("STOP MOTOR FIRST")); return; }
    captureAfterTest = (cmd[0] == 'b');
    startMotor(); return;
  }
  if (strcmp(cmd, "v") == 0) {
    if (motorState != STOPPED) { Serial.println(F("STOP MOTOR FIRST")); return; }
    pendingReverse = !pendingReverse;
    printSettings(); return;
  }
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

// Button selects the NEXT test direction; never reverses an energized bridge.
void updateReverseButton() {
  bool raw = digitalRead(REV);
  uint32_t now = millis();
  if (raw != lastButtonRaw) { lastButtonRaw = raw; buttonChangedMs = now; }
  if (now - buttonChangedMs >= 30 && raw != stableButton) {
    stableButton = raw;
    if (raw == LOW) {
      if (motorState == STOPPED) {
        pendingReverse = !pendingReverse;
        Serial.println(pendingReverse ? F("NEXT: REV") : F("NEXT: FWD"));
      } else Serial.println(F("BUTTON IGNORED: STOP FIRST"));
    }
  }
}

// Never issue a burst of catch-up steps: a delay >= one sector aborts the test.
void serviceCommutation(bool ramping) {
  uint32_t now = micros();
  if ((int32_t)(now - nextStepUs) < 0) return;
  uint32_t late = now - nextStepUs;
  if (late > maxLatenessUs) maxLatenessUs = late;
  if (late >= currentIntervalUs) {
    stopMotor(); Serial.println(F("FAULT: COMMUTATION DEADLINE MISSED")); return;
  }
  nextStep();
  if (motorState == STOPPED) return;
  lastStepUs = now;
  if (ramping) currentIntervalUs = rampIntervalUs(millis() - stateStartMs);
  nextStepUs += currentIntervalUs;
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
  updateReverseButton();
  handleSerial();                                  // Всегда принимать команды.
  if (motorState == STOPPED) return;              // Пока ничего не запущено — выход.

  if (motorState == CAPTURING) {                  // Новый режим независимой записи.
    uint32_t nowUs = micros();                   // Узнать текущее время в мкс.
    if (coastCount >= COAST_SAMPLES) {            // Собрали все 128 записей?
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
      nextStep();                                // Выравнивание уже выдержало шаг 0.
      lastStepUs = micros();
      lastInfoMs = millis();                      // Нулевой момент сообщений.
      currentIntervalUs = START_STEP_US;         // Первый шаг через 100 мс.
      nextStepUs = lastStepUs + currentIntervalUs;
      Serial.println(F("RAMP START"));
    }
    return;                                       // Во время ALIGN шаги не меняются.
  }

  if (motorState == RAMPING) {                   // Стадия принудительного разгона.
    uint32_t elapsed = (uint32_t)(nowMs - stateStartMs); // Сколько мс прошло.
    if (elapsed >= RAMP_MS) {                    // Шесть секунд закончились?
      motorState = HOLDING;                      // Перейти к удержанию.
      stateStartMs = millis();                   // Отсчёт двух секунд HOLD.
      // Preserve the already scheduled sector: no extra pause at RAMP -> HOLD.
      currentIntervalUs = (uint32_t)targetStepMs * 1000UL;
      serviceCommutation(false);
      Serial.println(F("HOLD START"));
      return;
    }
    serviceCommutation(true);
    if (motorState == STOPPED) return;
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
      if (captureAfterTest) {
        captureAfterTest = false;
        beginCoastCapture();
      } else stopMotor();
      return;
    }
    serviceCommutation(false);
    return;
  }
}
