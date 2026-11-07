// Передача управления по измеренным трём ЭДС: краткий выбег, угол, первый вектор и подтверждение компаратором.
// Из предыдущих версий взята распиновка; алгоритм и начальные настройки написаны заново. 
// Сам скетч не выбирает плату в среде разработки: этот выбор выполняется ДО компиляции. 
// Файл sketch.yaml рядом со скетчем выбирает Нано по умолчанию для командной утилиты Ардуино. 
// В Ардуино ИДЕ нужно выбрать: Инструменты → Плата → Ардуино Нано → ATmega328P. 
// Обязательное соединение: виртуальная нейтраль А0 через резистор 1 кОм подключается к Д6. 
// Пуск с обратной связью — t; тест разгона без неё — o; медленные шаги — j; остановка — s. 
// Команды f и l выбирают быстрый и прежний разгон; g выводит диагностику компаратора; d выводит частоту выбега после шести секунд; q печатает состояние; e меняет полярность ЭДС; знаки + и - меняют стартовый ШИМ только при остановке. 
// Команда p задаёт тест: ШИМ 60, 3921 Гц, быстрый разгон, направление 1, инверсия 0; сама двигатель не запускает.
// После включения сначала повернуть ручку в ноль; после остановки новый пуск остаётся ручным. 
// Ручка регулирует мощность, а не заданные обороты; программного измерения тока здесь нет. 
// Это стендовый вариант: параметры запуска и достаточность паузы ключей проверяются на моторе. 
#include <math.h> // Подключаем преобразование трёх ЭДС в электрический угол.
#include <Arduino.h> // Подключаем стандартные функции и определения Ардуино. 
#if !defined(ARDUINO_AVR_NANO) && !defined(BLDC_MODEL) // Проверяем выбранную плату; исключение разрешено только программной модели. 
#error "Выберите плату Arduino Nano; скетч не может изменить плату в настройках IDE." // Останавливаем сборку при неверном выборе платы. 
#endif // Завершаем проверку имени платы. 
#if !defined(__AVR_ATmega328P__) && !defined(BLDC_MODEL) // Проверяем именно контроллер ATmega328P, а не другую разновидность Нано. 
#error "Требуется процессор ATmega328P; выберите его в настройках платы Nano." // Останавливаем сборку для несовместимого процессора. 
#endif // Завершаем проверку модели контроллера. 
#if F_CPU != 16000000UL // Проверяем тактовую частоту процессора. 
#error "Требуется классическая Arduino Nano с частотой 16 МГц." // Останавливаем сборку при неподходящей частоте. 
#endif // Завершаем проверку частоты. 
const uint8_t TOP[3]={7,4,2}; // Задаём выходы верхних ключей фаз У, В, Ш; высокий уровень включает ключ. 
const uint8_t BOTTOM[3]={11,10,9}; // Задаём выходы нижних ключей фаз У, В, Ш с аппаратным ШИМ. 
const uint8_t PHASE[3]={3,2,1}; // По уточнённому монтажу назначаем фазам У, В, Ш каналы А3, А2, А1 соответственно. 
const uint8_t BUTTON=8,NEUTRAL=6,HANDLE=A4; // Назначаем кнопку, положительный вход компаратора и потенциометр. 
const uint8_t DRIVE[6][3]={{0,1,2},{0,2,1},{1,2,0},{1,0,2},{2,0,1},{2,1,0}}; // Для каждого сектора задаём верхнюю, нижнюю и свободную фазу. 
const uint16_t DEAD_TIME_US=120; // Задаём исходную паузу закрывания ключей; её достаточность нужно измерить. 
const uint16_t FAST_RAMP=980,SLOW_RAMP=990; // Задаём уменьшение сектора на два или один процент за шаг для быстрого и прежнего разгона.
const uint32_t SEARCH_TIME_MS=15000,OPEN_TIME_MS=15000; // Ограничиваем подтверждение обратной связи пятнадцатью секундами и сохраняем такую же длительность теста без неё.
bool slowPwm=false; // По умолчанию сохраняем ШИМ около 31 кГц; команда b включает диагностический ШИМ около 3,9 кГц.
uint16_t rampFactor=FAST_RAMP; // По умолчанию выбираем разгон примерно вдвое быстрее прежнего без повышения мощности.
uint32_t searchStartedMs=0; // Отмеряем поиск обратной связи временем, а не числом переключений.
const uint16_t POSITION_MS=600; // Задаём время начального позиционирования неподвижного ротора. 
const uint32_t FIRST_SECTOR_US=100000,FINAL_SECTOR_US=20000; // Задаём начальную и конечную длительности принудительного сектора. 
const uint32_t SHORTEST_US=1800,LONGEST_US=50000; // Ограничиваем допустимый период работы по обратной связи. 
const uint8_t INITIAL_PWM=54,MAXIMUM_PWM=140; // Ограничиваем начальное и максимальное заполнение отсчётами из 255. 
const uint16_t ZERO_HANDLE=35,START_HANDLE=75; // Задаём отдельные пороги остановки и разрешения пуска для ручки. 
const uint16_t COAST_MS=6000; // Задаём минимальную паузу выбега; она не доказывает полную остановку ротора. 
const uint8_t CLEAN_SECTORS=9; // Требуем девять последовательных подходящих секторов перед захватом. 
const uint16_t SIGN_TIME_US=72; // Требуем устойчивый знак компаратора в течение нескольких периодов ШИМ. 
enum Mode:uint8_t {IDLE,POSITION,ACCELERATE,ACQUIRE,FOLLOW,COAST,TEST_RUN,JOG,MEASURE,WAIT_VECTOR,FIRST_VECTOR,VERIFY}; // Объявляем состояния алгоритма.
Mode mode=IDLE; // После включения оставляем мост отключённым. 
uint8_t vectorIndex=0,pwm=INITIAL_PWM,requestedPwm=INITIAL_PWM,startupPwm=INITIAL_PWM; // Храним текущий сектор и значения заполнения. 
uint8_t launchType=0,jogSteps=0; // Выбираем обычный пуск, тест без ЭДС или медленную пошаговую проверку.
uint32_t traceMs=0; // Ограничиваем частоту диагностических строк во время принудительного разгона.
uint32_t testStartedMs=0; // Ограничиваем время принудительного вращения без обратной связи.
int8_t direction=1; // Задаём направление обхода таблицы; знак меняется только при отключении моста. 
bool invert=false,zeroSeen=false,comparatorActive=false,operated=false; // Храним настройку полярности и признаки готовности. 
uint32_t enteredMs=0,startedUs=0,deadlineUs=0,sectorUs=FIRST_SECTOR_US; // Храним метки состояния, сектора и следующего переключения. 
uint32_t offMs=0,handleMs=0,slewMs=0,lastCrossUs=0; // Храним метки выключения, ручки, изменения мощности и предыдущего пересечения. 
uint16_t handleValue=0,goodTotal=0; // Храним показание потенциометра и число принятых пересечений. 
uint8_t confidence=0,errorCode=0; // Храним длину серии, число попыток захвата и код последней остановки. 
bool initialSign=false,crossingFound=false,candidateKnown=false,previousCrossKnown=false; // Храним признаки фильтра и истории пересечений. 
uint8_t candidateSign=0,repeats=0; // Храним кандидат знака и число его подтверждений. 
uint32_t candidateUs=0,crossingUs=0,measuredUs=FINAL_SECTOR_US; // Храним начало кандидата, принятое пересечение и оценку периода. 
uint32_t history[3]={FINAL_SECTOR_US,FINAL_SECTOR_US,FINAL_SECTOR_US}; // Храним три периода для медианного фильтра. 
uint8_t historyIndex=0; // Храним позицию следующего элемента истории периодов. 
bool rawButton=HIGH,stableButton=HIGH; // Храним мгновенное и подтверждённое состояния кнопки. 
uint32_t buttonMs=0; // Храним момент изменения кнопки для подавления дребезга. 
const uint8_t ROTOR_PAIRS=2,COAST_ROWS=60; // Задаём заявленные две пары полюсов ротора и объём журнала выбега.
struct CoastRecord{uint16_t ms,period100us,peak;}; // Храним время, период электрического оборота и пиковое отклонение сигнала одного интервала.
CoastRecord coastLog[COAST_ROWS]; // Выделяем 360 байт для шести секунд выбега с записями каждые сто миллисекунд.
uint8_t coastCount=0; // Храним число готовых строк последнего выбега.
uint32_t coastOrigin=0,coastSampleDue=0,coastRowDue=0,coastRise=0,coastPeriod=0; // Храним временные метки выборок, строк и переходов ЭДС.
uint16_t coastPeak=0; // Накапливаем максимальную абсолютную разность фазы У и нейтрали за текущий интервал.
bool coastArmed=false,coastRiseKnown=false; // Требуем обе полярности сигнала перед регистрацией очередного полного периода.
uint32_t senseBlankUs=0,senseWindowEndUs=0,senseBaseEndUs=0,senseCrossEarlyUs=0,senseCrossLateUs=0; // Вычисляем временные границы один раз на сектор вместо дорогого деления при каждой выборке.
const uint8_t SENSE_ROWS=18; // Храним диагностику последних восемнадцати секторов без печати во время коммутации.
struct SenseRecord{uint16_t tries,samples,ones,firstEdge;uint8_t sector,wanted,flags;}; // Разделяем отсутствие выборок, постоянный уровень и отбраковку перехода.
SenseRecord senseLog[SENSE_ROWS],senseNow={}; // Выделяем небольшой кольцевой журнал и текущую запись.
uint8_t senseHead=0,senseCount=0,sensePrevious=0; // Храним позицию журнала, число строк и предыдущий сырой уровень.
bool senseOpen=false,sensePreviousKnown=false; // Не сохраняем один сектор дважды и отличаем первое чтение от перехода.
const uint16_t FIELD_SETTLE_MS=2000; // Даём ротору две секунды установиться на конечной принудительной частоте.
const uint8_t ANGLE_SAMPLES=8; // Оцениваем движение по восьми снимкам, а не по одному шумному измерению.
const uint32_t QUIET_BLANK_US=4000,SNAPSHOT_GAP_US=4000,QUIET_LIMIT_US=70000; // Ограничиваем гашение переходных процессов, шаг снимков и полное время без управления.
const float PI_F=3.14159265f,SECTOR_RAD=1.04719755f; // Задаём полный полупериод и угол одного электрического сектора в радианах.
struct AngleSample{uint32_t at;uint16_t phase[3],neutral;float angle;}; // Храним АЦП всех фаз, нейтраль, время и развёрнутый черновой угол по АЦП.
AngleSample angleLog[ANGLE_SAMPLES]; // Сохраняем исходные данные для диагностики передачи управления командой k.
bool clippedInputs=false;float fitAdcRms=0,fitAdcPeak=0; // Учитываем нижнее ограничение АЦП и сохраняем ошибку подбора трёхфазной модели.
uint32_t fitDurationUs=0,fitFinishedUs=0; // Сохраняем длительность расчёта и его завершение относительно отключения моста.
bool planValid=false; // Отличаем рассчитанный план от пустых или неудачных измерений.
uint8_t angleCount=0,plannedVector=0; // Храним число снимков и выбранный по ЭДС начальный силовой вектор.
uint32_t quietStartedUs=0,snapshotDueUs=0,plannedPowerUs=0,firstBoundaryUs=0; // Храним сроки измерения, включения вектора и первого расчётного переключения.
uint32_t anglePeriodUs=0,firstDurationUs=0; // Сохраняем измеренный период и длительность первого неполного сектора для отчёта.
float angleSlope=0,angleIntercept=0; // Храним угловую скорость в радианах за миллисекунду и начальный угол модели.
void saveSense(){ // Объявляем завершение диагностической записи перед сменой сектора или отключением.
 if(!senseOpen)return; // Пропускаем режимы без наблюдения компаратора и уже завершённые записи.
 if(initialSign)senseNow.flags|=1; // Отмечаем подтверждённый начальный знак.
 if(crossingFound)senseNow.flags|=2; // Отмечаем принятый переход ЭДС.
 senseLog[senseHead]=senseNow;senseHead=(senseHead+1)%SENSE_ROWS;if(senseCount<SENSE_ROWS)senseCount++; // Добавляем сектор в кольцевой журнал.
 senseOpen=false; // Запрещаем повторную запись того же сектора.
} // Завершаем сохранение диагностики сектора.
bool dueNow(uint32_t now,uint32_t due){return (int32_t)(now-due)>=0;} // Сравниваем времена с учётом переполнения счётчика микросекунд. 
void bridgeOff(){ // Объявляем отключение всех силовых выходов. 
 for(uint8_t i=0;i<3;i++){analogWrite(BOTTOM[i],0);digitalWrite(TOP[i],LOW);} // Снимаем ШИМ и команды верхних ключей всех фаз. 
} // Завершаем отключение моста. 
void configurePwm(bool slow){ // Объявляем переключение частоты только при отключённом мосте.
 bridgeOff();slowPwm=slow; // Сначала снимаем все сигналы и сохраняем выбранный режим.
 TCCR1A=_BV(WGM10);TCCR1B=slow?_BV(CS11):_BV(CS10); // Меняем делитель таймера 1 с одного на восемь для удлинения импульса.
 TCCR2A=_BV(WGM20);TCCR2B=slow?_BV(CS21):_BV(CS20); // Устанавливаем такой же делитель для таймера 2.
 OCR1A=0;OCR1B=0;OCR2A=0; // Оставляем заполнение всех нижних ключей нулевым до явного пуска.
} // Завершаем безопасную настройку частоты ШИМ.
void adcMode(){ // Объявляем возврат измерительного узла в режим АЦП. 
 ACSR=_BV(ACD)|_BV(ACI); // Выключаем компаратор, запрещаем его прерывание и сбрасываем флаг. 
 ADCSRB&=(uint8_t)~_BV(ACME); // Отключаем аналоговый мультиплексор от компаратора. 
 ADCSRA|=_BV(ADEN); // Разрешаем работу АЦП для потенциометра. 
 comparatorActive=false; // Запоминаем, что компаратор сейчас не используется. 
} // Завершаем настройку режима АЦП. 
void sensingMode(){ // Объявляем включение компаратора для свободной фазы. 
 ADCSRA&=(uint8_t)~_BV(ADEN); // Отключаем АЦП, иначе мультиплексор компаратора не заработает. 
 ADMUX=_BV(REFS0)|PHASE[DRIVE[vectorIndex][2]]; // Выбираем аналоговый канал свободной фазы. 
 ADCSRB|=_BV(ACME); // Подключаем выбранный канал к отрицательному входу компаратора. 
 ACSR=_BV(ACI); // Выбираем внешнюю нейтраль на положительном входе и включаем компаратор без прерываний. 
 comparatorActive=true; // Запоминаем активный режим сравнения фаз. 
} // Завершаем настройку компаратора. 
void beginCoastMeasurement(){ // Объявляем сброс измерения сразу после отключения силового моста.
 coastOrigin=micros();coastSampleDue=coastOrigin+4000;coastRowDue=coastOrigin+100000; // Пропускаем первые четыре миллисекунды переходных процессов и назначаем первую строку.
 coastCount=0;coastPeak=0;coastPeriod=0;coastRiseKnown=false;coastArmed=false; // Удаляем старый журнал и признаки пересечений.
} // Завершаем начало нового измерения выбега.
void measureCoast(){ // Объявляем измерение частоты при полностью отключённых ключах.
 uint32_t now=micros(); // Получаем время для очередной выборки и записи.
 if(dueNow(now,coastSampleDue)){ // Выполняем пару измерений примерно раз в миллисекунду.
  analogRead(A0);uint16_t n=analogRead(A0);analogRead(A0+PHASE[0]);uint16_t u=analogRead(A0+PHASE[0]); // Последовательно измеряем нейтраль и фазу У, отбрасывая первые преобразования.
  now=micros();coastSampleDue=now+1000;int16_t difference=(int16_t)u-(int16_t)n; // Привязываем выборку к фактическому времени и вычисляем ЭДС относительно нейтрали.
  uint16_t amplitude=difference<0?-difference:difference;if(amplitude>coastPeak)coastPeak=amplitude; // Запоминаем наибольшую амплитуду текущего интервала.
  if(difference<=-4)coastArmed=true; // Сначала требуем отрицательный уровень за пределами шумового порога.
  if(coastArmed&&difference>=4){ // Регистрируем только восходящий переход через нейтраль с гистерезисом.
   coastArmed=false; // Запрещаем повторный переход без нового отрицательного полупериода.
   if(coastRiseKnown){ // Вычисляем период только между двумя полными восходящими переходами.
    uint32_t interval=now-coastRise; // Получаем длительность одного электрического оборота, а не шестой его части.
    if(interval>=8000&&interval<=1000000&&(coastPeriod==0||(interval>=coastPeriod/2&&interval<=coastPeriod*2)))coastPeriod=interval;else coastPeriod=0; // Отбрасываем неправдоподобные периоды и резкие скачки.
   } // Завершаем проверку полного периода.
   coastRise=now;coastRiseKnown=true; // Сохраняем переход даже при сбросе оценки, чтобы разрешить повторное измерение.
  } // Завершаем обработку восходящего перехода.
 } // Завершаем очередную выборку выбега.
 if(coastCount<COAST_ROWS&&dueNow(now,coastRowDue)){ // Сохраняем сводку каждые сто миллисекунд без вывода в порт во время измерения.
  CoastRecord &r=coastLog[coastCount++];r.ms=(now-coastOrigin)/1000;r.peak=coastPeak;coastPeak=0; // Записываем фактическое время строки и амплитуду последнего интервала.
  uint32_t stale=coastPeriod*2+50000;if(stale<250000)stale=250000; // Определяем срок устаревания последнего измеренного периода.
  r.period100us=coastPeriod&&now-coastRise<=stale?coastPeriod/100:0;coastRowDue+=100000; // Сохраняем период либо признак отсутствия измерения, не объявляя нулевые обороты.
 } // Завершаем запись очередной строки выбега.
} // Завершаем измерение частоты выбега.
void dumpCoast(){ // Объявляем вывод измеренного выбега после завершения записи.
 if(mode!=IDLE){Serial.println(F("Дождитесь окончания записи выбега."));return;} // Не мешаем измерению длинным выводом таблицы.
 Serial.println(F("мс,электрическая_Гц,об_мин,пик_АЦП")); // Печатаем названия столбцов для сохранения таблицы.
 for(uint8_t i=0;i<coastCount;i++){ // Последовательно выдаём накопленные строки.
  CoastRecord &r=coastLog[i];Serial.print(r.ms);Serial.print(','); // Выводим время от отключения моста.
  if(r.period100us){ // Пересчитываем только подтверждённый и неустаревший период.
   uint32_t hz100=1000000UL/r.period100us;Serial.print(hz100/100);Serial.print('.');if(hz100%100<10)Serial.print('0');Serial.print(hz100%100); // Выводим электрическую частоту с двумя знаками после точки без вычислений с плавающей точкой.
   Serial.print(',');Serial.print(600000UL/((uint32_t)r.period100us*ROTOR_PAIRS)); // Переводим электрический период в механические обороты с учётом пар полюсов ротора.
  }else Serial.print(F("нет,нет")); // Отсутствие сигнала обозначаем явно, не подменяя его доказанной остановкой.
  Serial.print(',');Serial.println(r.peak); // Выводим амплитуду разности в отсчётах АЦП для оценки качества сигнала.
 } // Завершаем выдачу таблицы.
} // Завершаем вывод журнала выбега.
void halt(uint8_t code){ // Объявляем остановку с необязательным кодом причины. 
 if(mode==COAST||mode==IDLE){bridgeOff();return;} // Повторная остановка не стирает уже записываемый или готовый выбег.
 bridgeOff();saveSense(); // Сначала физически снимаем управление мостом, затем сохраняем последний наблюдавшийся сектор. 
 adcMode(); // Только после снятия управления возвращаем АЦП. 
 beginCoastMeasurement(); // Начинаем измерение только после отключения всех ключей и возврата АЦП.
 offMs=millis();mode=COAST;zeroSeen=false;errorCode=code; // Запускаем паузу выбега и требуем новое возвращение ручки в ноль. 
 Serial.print(F("Остановка; код="));Serial.println(code); // Сообщаем причину после отключения силовых выходов. 
} // Завершаем обработку остановки. 
void readHandle(){ // Объявляем чтение потенциометра с восстановлением компаратора. 
 bool restore=comparatorActive;adcMode(); // Запоминаем прежний режим и временно разрешаем АЦП. 
 uint8_t saved=ADCSRA;ADCSRA=(saved&(uint8_t)~7)|_BV(ADPS2)|_BV(ADPS0); // Выбираем делитель АЦП 32 для короткого измерения. 
 analogRead(HANDLE);handleValue=analogRead(HANDLE); // Отбрасываем первое преобразование и сохраняем второе. 
 ADCSRA=saved; // Возвращаем прежнюю настройку АЦП. 
 requestedPwm=startupPwm; // По умолчанию сохраняем минимальное рабочее заполнение. 
 if(handleValue>START_HANDLE)requestedPwm=map(handleValue,START_HANDLE,1023,startupPwm,MAXIMUM_PWM); // Переводим положение ручки в желаемую мощность. 
 if(restore)sensingMode(); // Возвращаем компаратор на свободную фазу, если он был включён. 
 handleMs=millis(); // Запоминаем время чтения ручки. 
 if(handleValue<=ZERO_HANDLE)zeroSeen=true; // Отмечаем явное возвращение ручки в ноль. 
} // Завершаем чтение органа управления. 
void clearDetector(){ // Объявляем очистку фильтра после смены силового вектора.
 senseBlankUs=sectorUs/10;if(senseBlankUs<180)senseBlankUs=180;if(senseBlankUs>800)senseBlankUs=800; // Один раз рассчитываем прежнее время подавления переходных процессов.
 senseWindowEndUs=sectorUs*4/5;senseBaseEndUs=sectorUs*2/5; // Один раз рассчитываем прежние границы наблюдения и начального уровня.
 senseCrossEarlyUs=sectorUs/4;senseCrossLateUs=sectorUs*3/4; // Один раз рассчитываем прежнее допустимое окно пересечения ЭДС. 
 initialSign=false;crossingFound=false;candidateKnown=false;repeats=0; // Сбрасываем признаки текущего сектора, сохраняя историю периодов. 
 senseNow={};senseNow.sector=vectorIndex+1;senseNow.wanted=((vectorIndex&1)?0:1)^(direction<0)^invert; // Начинаем запись с номером сектора и ожидаемым знаком после пересечения.
 senseOpen=comparatorActive;sensePreviousKnown=false; // Включаем статистику только при действительном наблюдении компаратора.
} // Завершаем очистку фильтра. 
void energize(){ // Объявляем безопасное включение выбранного сектора. 
 for(uint8_t i=0;i<3;i++)analogWrite(BOTTOM[i],0); // Перед изменением фаз выключаем все нижние ключи. 
 for(uint8_t i=0;i<3;i++)if(i!=DRIVE[vectorIndex][0])digitalWrite(TOP[i],LOW); // Выключаем ненужные верхние ключи, сохраняя общий ключ соседних секторов. 
 delayMicroseconds(DEAD_TIME_US); // Выдерживаем исходную паузу закрывания ключей. 
 digitalWrite(TOP[DRIVE[vectorIndex][0]],HIGH); // Включаем верхний ключ выбранной положительной фазы. 
 analogWrite(BOTTOM[DRIVE[vectorIndex][1]],pwm); // Включаем ШИМ только на нижнем ключе другой фазы. 
 startedUs=micros();clearDetector(); // Фиксируем реальное начало нового силового вектора и очищаем фильтр. 
 if(comparatorActive)sensingMode(); // Переводим компаратор на новую свободную фазу. 
 if(millis()-handleMs>=40)readHandle(); // Читаем ручку внутри интервала подавления помех после коммутации. 
 if(handleValue<=ZERO_HANDLE){halt(0);return;} // При нулевой ручке прекращаем подачу мощности. 
 if(mode==FOLLOW&&millis()-slewMs>=25){ // Разрешаем плавную регулировку мощности только после захвата ротора. 
  if(pwm<requestedPwm)pwm++;else if(pwm>requestedPwm)pwm--; // Изменяем заполнение не более чем на один отсчёт за обновление. 
  analogWrite(BOTTOM[DRIVE[vectorIndex][1]],pwm);slewMs=millis(); // Применяем новое заполнение только к активному нижнему ключу. 
 } // Завершаем плавную регулировку мощности. 
 if((mode==ACCELERATE||mode==TEST_RUN||mode==ACQUIRE)&&millis()-traceMs>=1000&&Serial.availableForWrite()>=48){ // Печатаем короткую строку не чаще раза в секунду и только без ожидания свободного места в порту.
  traceMs=millis();Serial.print(mode==ACCELERATE?F("Разгон "):(mode==TEST_RUN?F("Тест "):F("Выдержка "))); // Указываем этап, на котором сейчас находится программа, а не измеренное движение ротора.
  Serial.print(F("мкс="));Serial.print(sectorUs);Serial.print(F(" ШИМ="));Serial.println(pwm); // Показываем заданный период сектора и фактическое заполнение для сопоставления со срывом.
 } // Завершаем неблокирующую диагностику принудительного вращения.
} // Завершаем включение сектора. 
void stepForward(){saveSense();vectorIndex=(vectorIndex+(direction>0?1:5))%6;energize();} // Продвигаем силовой вектор на один сектор в выбранном направлении. 
uint32_t medianPeriod(){ // Объявляем медианный фильтр трёх последних интервалов ЭДС. 
 uint32_t a=history[0],b=history[1],c=history[2],t; // Копируем историю во временные переменные. 
 if(a>b){t=a;a=b;b=t;} // Упорядочиваем первую пару. 
 if(b>c){t=b;b=c;c=t;} // Упорядочиваем вторую пару. 
 if(a>b){t=a;a=b;b=t;} // Повторно упорядочиваем первую пару. 
 return b; // Возвращаем среднее по порядку значение без влияния одиночного выброса. 
} // Завершаем медианный фильтр. 
bool sampleSign(uint8_t &sign){ // Объявляем выборку с коротким ограниченным ожиданием устойчивой части ШИМ.
 uint8_t sink=DRIVE[vectorIndex][1],margin=pwm>28?pwm-28:0; // Отступ от фронтов равен 28 тактам: около 1,75 мкс обычно или 14 мкс в диагностическом режиме.
 uint8_t bit=sink==0?PB3:(sink==1?PB2:PB1); // Заранее определяем бит управляющего порта выбранного нижнего ключа.
 uint8_t limit=slowPwm?192:32; // Ограничиваем ожидание плато с учётом восьмикратного увеличения периода ШИМ.
 for(uint8_t attempt=0;attempt<limit;attempt++){ // Выполняем ограниченное число проверок вместо неограниченного ожидания при неисправности сигнала.
  uint8_t before=sink==0?TCNT2:TCNT1L; // Читаем счётчик активного аппаратного ШИМ непосредственно перед выборкой.
  uint8_t status=ACSR,actual=PINB; // Сразу снимаем компаратор и фактический логический уровень управляющего выхода.
  uint8_t after=sink==0?TCNT2:TCNT1L; // Проверяем, что во время чтения не приблизились к фронту ШИМ.
  if(before<margin&&after<margin&&(actual&_BV(bit))){sign=(status&_BV(ACO))?1:0;return true;} // Принимаем только подтверждённое включённое плато с прежними критериями.
 } // Завершаем ограниченное ожидание, сохраняя возможность обслужить остановку и сроки коммутации.
 return false; // При отсутствии подходящего плато пропускаем выборку, не читая заведомо неподходящий сигнал.
} // Завершаем получение знака ЭДС.
void observeRotor(){ // Объявляем поиск пересечения ЭДС для текущего сектора. 
 if(!comparatorActive||crossingFound)return; // После принятого пересечения больше не ищем переходы в этом секторе. 
 uint32_t age=micros()-startedUs; // Вычисляем только возраст сектора без повторного деления временных границ.
 if(age<senseBlankUs||age>senseWindowEndUs)return; // Используем прежнее окно, заранее рассчитанное при коммутации.
 if(senseNow.tries<65535)senseNow.tries++; // Считаем попытки чтения до проверки допустимого плато ШИМ.
 if((ADCSRA&_BV(ADEN))||!(ADCSRB&_BV(ACME)))senseNow.flags|=32; // Отмечаем неправильное состояние АЦП или мультиплексора при наблюдении.
 uint8_t sign;if(!sampleSign(sign))return; // Получаем знак только при подходящем состоянии ШИМ. 
 uint32_t now=micros(); // Присваиваем короткой выборке временную метку.
 if(senseNow.samples<65535)senseNow.samples++; // Считаем принятые выборки без переполнения.
 if(sign&&senseNow.ones<65535)senseNow.ones++; // Считаем единицы; число нулей равно разности счётчиков.
 if(sensePreviousKnown&&sign!=sensePrevious&&senseNow.firstEdge==0)senseNow.firstEdge=now-startedUs; // Запоминаем первый сырой переход, ещё не считая его достоверным пересечением ЭДС.
 sensePrevious=sign;sensePreviousKnown=true; // Сохраняем уровень для поиска следующего сырого перехода. 
 if(!candidateKnown||candidateSign!=sign){candidateKnown=true;candidateSign=sign;candidateUs=now;repeats=1;return;} // Начинаем подтверждение нового устойчивого знака. 
 if(repeats<255)repeats++; // Увеличиваем число подтверждений без переполнения. 
 if(repeats<3||now-candidateUs<SIGN_TIME_US)return; // Отбрасываем короткие всплески и случайные одиночные чтения. 
 uint8_t wanted=((vectorIndex&1)?0:1)^(direction<0)^invert; // Вычисляем ожидаемый знак после пересечения с учётом направления и настройки. 
 if(!initialSign){ // Сначала проверяем, что наблюдалась противоположная полярность. 
  if(sign!=wanted&&now-startedUs<senseBaseEndUs)initialSign=true;else senseNow.flags|=4; // Принимаем исходный знак только до середины сектора. 
  return; // До подтверждения исходного знака не принимаем пересечение. 
 } // Завершаем проверку исходной полярности. 
 if(sign!=wanted)return; // Ждём перехода к ожидаемому знаку. 
 uint32_t at=candidateUs,offset=at-startedUs; // Используем первую выборку нового знака, компенсируя задержку подтверждения. 
 if(offset<senseCrossEarlyUs||offset>senseCrossLateUs){senseNow.flags|=8;return;} // Принимаем пересечение только в центральной половине сектора. 
 if(previousCrossKnown){ // При наличии истории проверяем новый полный интервал между пересечениями. 
  uint32_t interval=at-lastCrossUs,reference=(mode==FOLLOW||mode==VERIFY)?measuredUs:sectorUs; // Сравниваем реальный интервал с текущей оценкой.
  if(interval<reference*7/10||interval>reference*13/10){senseNow.flags|=16;if(mode==FOLLOW||mode==VERIFY)halt(3);return;} // При резком нарушении периода снимаем управление вместо слепой коммутации.
  history[historyIndex]=interval;historyIndex=(historyIndex+1)%3; // Добавляем новый интервал в кольцевую историю. 
  measuredUs=medianPeriod(); // Получаем устойчивую оценку периода электрического сектора. 
 }else{for(uint8_t i=0;i<3;i++)history[i]=sectorUs;measuredUs=sectorUs;} // Первое пересечение инициализирует историю заданным периодом. 
 if(measuredUs<SHORTEST_US||measuredUs>LONGEST_US){halt(4);return;} // Останавливаем мост при выходе скорости за настроенный диапазон. 
 crossingUs=at;lastCrossUs=at;previousCrossKnown=true;crossingFound=true; // Сохраняем принятое пересечение и его связь с предыдущим. 
 if(goodTotal<65535)goodTotal++; // Наращиваем диагностический счётчик без переполнения. 
 if(mode==FOLLOW||mode==VERIFY){ // При замкнутой коммутации назначаем следующее переключение по положению ротора.
  deadlineUs=at+measuredUs/2-DEAD_TIME_US; // Отсчитываем тридцать электрических градусов и учитываем основную паузу ключей. 
  if(dueNow(micros(),deadlineUs))halt(5); // Не пытаемся выполнить уже опоздавшую коммутацию. 
 } // Завершаем назначение переключения по ЭДС. 
} // Завершаем наблюдение положения ротора. 
float wrapAngle(float value){ // Объявляем приведение угла к интервалу от минус пи до плюс пи.
 while(value>PI_F)value-=2*PI_F; // Убираем полные положительные обороты без дорогостоящей операции остатка.
 while(value<=-PI_F)value+=2*PI_F; // Убираем полные отрицательные обороты.
 return value; // Возвращаем угол для сравнения соседних снимков и центра вектора.
} // Завершаем нормализацию угла.
void beginHandover(){ // Объявляем начало измерения положения после принудительной выдержки.
 bridgeOff();saveSense();adcMode(); // Полностью отключаем мост и только затем разрешаем измерение трёх фаз АЦП.
 quietStartedUs=micros();snapshotDueUs=quietStartedUs+QUIET_BLANK_US; // Выдерживаем четыре миллисекунды без мощности перед первым снимком.
 angleCount=0;anglePeriodUs=0;firstDurationUs=0;fitDurationUs=fitFinishedUs=0;planValid=false;clippedInputs=false;fitAdcRms=fitAdcPeak=0;mode=MEASURE; // Сбрасываем прежний результат и запрещаем любые силовые векторы на время измерений.
 Serial.println(F("Измерение трёх ЭДС.")); // Обозначаем отдельную фазу выбега, не объявляя успешный захват.
} // Завершаем начало передачи управления.
void takeAngleSample(){ // Объявляем симметричное чтение фаз для уменьшения ошибки последовательного АЦП.
 uint16_t sum[4]={0,0,0,0};uint32_t before=micros();uint8_t saved=ADCSRA; // Создаём накопители и запоминаем начало измерения и настройку АЦП.
 ADCSRA=(saved&(uint8_t)~7)|_BV(ADPS2)|_BV(ADPS0); // Выбираем делитель тридцать два на короткое время чтения снимка.
 for(uint8_t pass=0;pass<2;pass++){ // Измеряем один раз вперёд и один раз назад по каналам.
  for(uint8_t j=0;j<4;j++){ // Обходим три фазных входа и нейтраль.
   uint8_t index=pass?3-j:j,pin=index<3?A0+PHASE[index]:A0; // Во втором проходе обращаем порядок каналов для общей средней временной метки.
   analogRead(pin);sum[index]+=analogRead(pin); // После переключения канала отбрасываем первое преобразование и суммируем второе.
  } // Завершаем один проход по входам.
 } // Завершаем симметричное чтение снимка.
 ADCSRA=saved;uint32_t after=micros(); // Возвращаем АЦП и фиксируем конец измерения.
 AngleSample &r=angleLog[angleCount];r.angle=0;r.at=before+(after-before)/2; // Относим усреднённые фазные измерения к середине интервала с учётом переполнения времени.
 for(uint8_t j=0;j<3;j++)r.phase[j]=(sum[j]+1)/2; // Усредняем два результата каждой фазы без изменения назначения выводов.
 r.neutral=(sum[3]+1)/2; // Сохраняем аппаратную нейтраль для проверки измерительного узла.
 bool clipped=false;for(uint8_t j=0;j<3;j++)if(r.phase[j]<=2)clipped=true; // Отличаем нижнее ограничение АЦП от достоверного нулевого напряжения фазы.
 if(clipped)clippedInputs=true; // Отмечаем наличие обрезанных отрицательных полуволн в серии снимков.
 for(uint8_t j=0;j<3;j++)if(r.phase[j]>=1020){angleCount++;halt(12);return;} // Не используем снимок при насыщении верхней границы входного диапазона.
 float u=(int16_t)r.phase[0]-(int16_t)r.neutral,v=(int16_t)r.phase[1]-(int16_t)r.neutral,w=(int16_t)r.phase[2]-(int16_t)r.neutral; // Вычисляем три ЭДС в отсчётах АЦП.
 angleCount++; // Сохраняем и неудачный снимок, чтобы его можно было прочитать после остановки.
 if(!clipped&&fabsf((u+v+w)/3)>8){halt(11);return;} // При несогласованной нейтрали запрещаем силовую передачу управления.
 float alpha=(2*u-v-w)/3,beta=(v-w)*0.57735027f; // Преобразуем трёхфазную ЭДС в неподвижные координаты без зависимости от общего уровня.
 float largest=u;if(v>largest)largest=v;if(w>largest)largest=w; // Оцениваем доступный сигнал по наибольшей положительной фазе, в том числе при обрезанных отрицательных фазах.
 if((!clipped&&alpha*alpha+beta*beta<64)||(clipped&&largest<8)){halt(7);return;} // Не определяем угол по слишком слабому сигналу, меньшему примерно восьми отсчётов.
 float raw=atan2f(beta,alpha); // Находим электрический угол вектора ЭДС; он отличается от угла магнитного потока ротора.
 r.angle=angleCount==1?raw:angleLog[angleCount-2].angle+wrapAngle(raw-angleLog[angleCount-2].angle); // Разворачиваем угол через границу полного оборота.
} // Завершаем получение одного снимка.
bool fitWaveform(uint32_t period,float &outC,float &outS,float &outRms,float &outPeak){ // Объявляем подбор общей гармоники по достоверным отсчётам трёх фаз.
 float omega=direction*SECTOR_RAD*1000/period,step=omega*(SNAPSHOT_GAP_US/1000.0f); // Задаём пробную скорость вблизи принудительной частоты и шаг между кадрами.
 float cs=cosf(step),sn=sinf(step),c=1,q=0;int16_t coeffX[ANGLE_SAMPLES][3],coeffY[ANGLE_SAMPLES][3]; // Вычисляем тригонометрию один раз на пробную частоту для ограниченного времени расчёта.
 int32_t accCC=0,accSS=0,accCross=0,accCY=0,accSY=0;uint8_t valid=0; // Накапливаем уравнения только для отсчётов выше нижнего ограничения АЦП.
 for(uint8_t j=0;j<ANGLE_SAMPLES;j++){ // Готовим общую временную модель для каждого снимка.
  float actual=(uint32_t)(angleLog[j].at-angleLog[0].at)/1000.0f,nominal=j*(SNAPSHOT_GAP_US/1000.0f),correction=omega*(actual-nominal); // Учитываем небольшой джиттер реального времени относительно равномерного шага.
  int16_t ca=(int16_t)((c-q*correction)*1024),sa=(int16_t)((q+c*correction)*1024); // Корректируем фазу до первого порядка; допуск опоздания ограничен отдельно.
  for(uint8_t phase=0;phase<3;phase++){ // Используем известный сдвиг фаз на сто двадцать электрических градусов.
   int16_t cp=phase==0?1024:-512,sp=phase==0?0:(phase==1?887:-887); // Задаём фазовые коэффициенты с масштабом тысяча двадцать четыре.
   int16_t x=((int32_t)ca*cp+(int32_t)sa*sp)/1024,y=((int32_t)sa*cp-(int32_t)ca*sp)/1024; // Один раз считаем коэффициенты каждой фазы целочисленно для обоих проходов.
   coeffX[j][phase]=x;coeffY[j][phase]=y; // Сохраняем коэффициенты, исключая повторный расчёт при проверке ошибки.
   if(angleLog[j].phase[phase]<=2)continue; // Не считаем обрезанную отрицательную полуволну истинным нулём ЭДС.
   int16_t z=(int16_t)angleLog[j].phase[phase]-(int16_t)angleLog[j].neutral; // Вычисляем достоверную ЭДС относительно измеренной нейтрали.
   accCC+=(int32_t)x*x;accSS+=(int32_t)y*y;accCross+=(int32_t)x*y;accCY+=(int32_t)x*z;accSY+=(int32_t)y*z;valid++; // Накапливаем систему без программной арифметики дробных чисел в горячем цикле.
  } // Завершаем уравнения фаз текущего кадра.
  float next=c*cs-q*sn;q=q*cs+c*sn;c=next; // Переходим к следующему номинальному времени без повторных вызовов синуса.
 } // Завершаем накопление системы измерений.
 float cc=accCC/1048576.0f,ss=accSS/1048576.0f,cross=accCross/1048576.0f,cy=accCY/1024.0f,sy=accSY/1024.0f; // Один раз переводим пять сумм в физический масштаб перед решением системы.
 float det=cc*ss-cross*cross,total=cc+ss; // Проверяем независимость информации об угле и амплитуде.
 if(valid<8||det<0.005f*total*total)return false; // Отвергаем недостаточные или вырожденные измерения.
 outC=(cy*ss-sy*cross)/det;outS=(sy*cc-cy*cross)/det; // Находим амплитудно-фазовые коэффициенты для этой пробной скорости.
 float sumError=0;outPeak=0; // Подготавливаем независимую проверку всех отсчётов, включая обрезанные.
 for(uint8_t j=0;j<ANGLE_SAMPLES;j++){ // Сопоставляем восстановленную гармонику с исходными АЦП.
  for(uint8_t phase=0;phase<3;phase++){ // Проверяем каждую из трёх фаз каждого кадра.
   float predicted=angleLog[j].neutral+(outC*coeffX[j][phase]+outS*coeffY[j][phase])/1024; // Проверяем модель по уже рассчитанным фазовым коэффициентам без повторной тригонометрии.
   float error=angleLog[j].phase[phase]<=2?(predicted>2?predicted-2:0):predicted-angleLog[j].phase[phase]; // Для нижнего ограничения проверяем не равенство нулю, а совместимость с отсечением.
   sumError+=error*error;if(fabsf(error)>outPeak)outPeak=fabsf(error); // Сохраняем среднюю и максимальную ошибку, не скрывая плохие кадры усреднением.
  } // Завершаем проверку фаз кадра.
 } // Завершаем проверку всей трёхфазной модели.
 outRms=sqrtf(sumError/(3*ANGLE_SAMPLES));return true; // Возвращаем качество в отсчётах АЦП.
} // Завершаем подбор одной пробной частоты.
void planFirstVector(){ // Объявляем независимую оценку положения и назначение первого силового вектора.
 float displacement=angleLog[ANGLE_SAMPLES-1].angle-angleLog[0].angle; // Используем развёрнутый порядок движения фаз только для проверки направления и наличия движения.
 if(displacement*direction<=0){halt(9);return;} // Не включаем мост при обратном или неопределённом движении.
 if(fabsf(displacement)<PI_F/12){halt(8);return;} // Отбрасываем практически неподвижную картину вместо подбора угла по постоянному напряжению.
 uint32_t fitStartedUs=micros(); // Начинаем замер фактического времени подбора на Нано.
 float best=1e20f,bestC=0,bestS=0,bestPeak=0;uint32_t bestPeriod=0; // Начинаем ограниченный поиск скорости в окрестности конца разгона.
 for(uint8_t candidate=0;candidate<7;candidate++){ // Проверяем семь частот без неограниченного численного поиска.
  uint32_t period=FINAL_SECTOR_US*(70UL+candidate*10UL)/100;float a,b,rms,peak; // Перебираем периоды от семидесяти до ста тридцати процентов уставки.
  if(period<SHORTEST_US||period>LONGEST_US||!fitWaveform(period,a,b,rms,peak))continue; // Не рассматриваем скорость вне рабочих ограничений или вырожденную модель.
  float bias=(float)period/FINAL_SECTOR_US-1,score=rms*rms+0.15f*bias*bias; // При почти одинаковой ошибке предпочитаем скорость ближе к известной частоте поля.
  if(score<best){best=score;bestC=a;bestS=b;bestPeak=peak;bestPeriod=period;fitAdcRms=rms;} // Сохраняем лучший проверенный вариант, а не последний вычисленный.
 } // Завершаем ограниченный подбор скорости.
 fitDurationUs=micros()-fitStartedUs;fitFinishedUs=micros()-quietStartedUs; // Сохраняем время подбора даже при последующей остановке по сроку.
 if(!bestPeriod){halt(8);return;} // Не подаём мощность, если подходящую модель определить не удалось.
 float amplitude=sqrtf(bestC*bestC+bestS*bestS);fitAdcPeak=bestPeak; // Получаем амплитуду общей ЭДС и наибольшую ошибку исходных отсчётов.
 if(amplitude<8){halt(7);return;} // Не используем угол слабой или вырожденной гармоники.
 if(fitAdcRms>3||fitAdcPeak>6||fitAdcPeak>amplitude*0.3f){halt(8);return;} // Отбрасываем шумную, несогласованную или слишком несинусоидальную ЭДС.
 anglePeriodUs=bestPeriod;angleSlope=direction*SECTOR_RAD*1000/bestPeriod;angleIntercept=atan2f(-bestS,bestC); // Оцениваем скорость и начальный угол восстановленной ЭДС, включая обрезанные отрицательные полуволны.
 measuredUs=sectorUs=anglePeriodUs;for(uint8_t j=0;j<3;j++)history[j]=measuredUs; // Инициализируем последующую коммутацию оценённым периодом.
 previousCrossKnown=false;confidence=0; // Заново подтверждаем реальные пересечения после включения выбранного вектора.
 readHandle();if(handleValue<=ZERO_HANDLE){halt(0);return;} // Проверяем ручку до повторной подачи мощности.
 if(micros()-quietStartedUs>QUIET_LIMIT_US-5000){halt(10);return;} // Не включаем мост по устаревшей оценке при слишком долгом расчёте.
 Serial.println(F("Угол измерен.")); // Отмечаем завершённую оценку, не подменяя её сообщением об успешном захвате.
 plannedPowerUs=micros()+3000; // Оставляем время на назначение начального вектора при выключенном мосте.
 for(uint8_t retry=0;retry<2;retry++){ // При попадании около границы один раз переносим включение за неё.
  float theta=angleIntercept+angleSlope*((uint32_t)(plannedPowerUs-angleLog[0].at)/1000.0f); // Предсказываем электрический угол ЭДС на момент включения.
  while(theta<0)theta+=2*PI_F; // Убираем отрицательные полные обороты.
  while(theta>=2*PI_F)theta-=2*PI_F; // Убираем лишние положительные полные обороты.
  plannedVector=((uint8_t)(theta/SECTOR_RAD)+1)%6; // Выбираем ближайший вектор тока с положительной мощностью относительно ЭДС.
  float center=plannedVector*SECTOR_RAD-SECTOR_RAD/2,delta=wrapAngle(theta-center); // Определяем положение внутри выбранного сектора.
  float remaining=(SECTOR_RAD/2-direction*delta)/fabsf(angleSlope)*1000; // Рассчитываем время до следующей границы по измеренному положению.
  if(remaining>=1500){firstDurationUs=(uint32_t)(remaining+0.5f);break;} // Исключаем слишком короткий первый силовой импульс.
  plannedPowerUs+=(uint32_t)(remaining>0?remaining:0)+200; // Ждём ближайшую границу и выбираем следующий подходящий сектор.
 } // Завершаем назначение первого вектора.
 if(!firstDurationUs||dueNow(micros(),plannedPowerUs-DEAD_TIME_US)){halt(10);return;} // Отвергаем опоздавший или неопределённый план до включения ключей.
 firstBoundaryUs=plannedPowerUs+firstDurationUs;deadlineUs=plannedPowerUs-DEAD_TIME_US;planValid=true;mode=WAIT_VECTOR; // Разделяем моменты первого включения и первой коммутации.
} // Завершаем план передачи управления.
void dumpAngles(){ // Объявляем выдачу исходных снимков после завершения опыта.
 if(mode!=IDLE){Serial.println(F("Дождитесь окончания выбега."));return;} // Не нарушаем измерение и коммутацию длинным выводом.
 Serial.println(F("снимок,мкс,U,V,W,N,угол_АЦП_0.1град,угол_модели_0.1град")); // Выводим исходные АЦП и развёрнутый электрический угол.
 for(uint8_t j=0;j<angleCount;j++){ // Выдаём только действительно снятые данные.
  AngleSample &r=angleLog[j];Serial.print(j+1);Serial.print(',');Serial.print((uint32_t)(r.at-quietStartedUs)); // Показываем номер и время от отключения моста.
  for(uint8_t phase=0;phase<3;phase++){Serial.print(',');Serial.print(r.phase[phase]);} // Выдаём измеренные фазы в фиксированном порядке У, В, Ш.
  Serial.print(',');Serial.print(r.neutral);Serial.print(',');Serial.print((int16_t)(r.angle*1800/PI_F)); // Добавляем нейтраль и черновой угол по АЦП; при обрезке он используется лишь для проверки направления.
  Serial.print(',');if(anglePeriodUs){float model=angleLog[0].angle+wrapAngle(angleIntercept-angleLog[0].angle)+angleSlope*((uint32_t)(r.at-angleLog[0].at)/1000.0f);Serial.println((int16_t)(model*1800/PI_F));}else Serial.println(F("нет")); // Показываем восстановленный угол отдельно, не выдавая черновое значение за результат модели.
 } // Завершаем выдачу исходных снимков.
 Serial.print(F("Расчёт, мкс="));Serial.print(fitDurationUs);Serial.print(F("; конец от отключения, мкс="));Serial.println(fitFinishedUs); // Позволяем отличить медленный расчёт от запоздавшего снимка и срока включения.
 Serial.print(F("Период сектора, мкс="));Serial.print(anglePeriodUs);Serial.print(F("; RMS, 0.1АЦП="));Serial.print((uint16_t)(fitAdcRms*10));Serial.print(F("; обрезка="));Serial.println(clippedInputs); // Показываем измеренную скорость и качество аппроксимации.
 if(!planValid){Serial.println(F("План включения не сформирован."));return;} // Не выдаём старый или нулевой вектор за успешное планирование.
 Serial.print(F("Выбег до включения, мкс="));Serial.println((uint32_t)(plannedPowerUs-quietStartedUs)); // Позволяем оценить реальную длительность расчёта на процессоре Nano.
 Serial.print(F("Первый вектор="));Serial.print(plannedVector+1);Serial.print(F("; длительность, мкс="));Serial.println(firstDurationUs); // Даём параметры перехода без объявления успешного захвата.
} // Завершаем выдачу диагностики передачи управления.
void dumpSense(){ // Объявляем выдачу статистики компаратора после отключения моста.
 if(mode!=IDLE){Serial.println(F("Дождитесь окончания выбега."));return;} // Не задерживаем ни коммутацию, ни измерение выбега длинной таблицей.
 Serial.println(F("сектор,ожидаемый_знак,попытки,выборки,единицы,первый_фронт_мкс,флаги")); // Даём столбцы для различения причин отсутствия захвата.
 uint8_t index=(senseHead+SENSE_ROWS-senseCount)%SENSE_ROWS; // Начинаем с наиболее старой из сохранённых записей.
 for(uint8_t i=0;i<senseCount;i++){ // Выдаём журнал в хронологическом порядке.
  SenseRecord &r=senseLog[index];Serial.print(r.sector);Serial.print(',');Serial.print(r.wanted);Serial.print(','); // Выводим номер сектора и требуемый конечный знак.
  Serial.print(r.tries);Serial.print(',');Serial.print(r.samples);Serial.print(',');Serial.print(r.ones);Serial.print(','); // Показываем наличие окна чтения и обоих логических уровней.
  Serial.print(r.firstEdge);Serial.print(',');Serial.println(r.flags);index=(index+1)%SENSE_ROWS; // Выводим возраст первого сырого фронта и причину отбраковки.
 } // Завершаем выдачу строк журнала.
 Serial.println(F("Биты:1=база;2=принят;4=знак/поздняя база;8=время;16=период;32=АЦП/мультиплексор.")); // Расшифровываем суммируемые диагностические флаги.
} // Завершаем выдачу диагностики компаратора.
void report(){ // Объявляем компактную печать настроек и результата последнего опыта.
 if(mode!=IDLE&&mode!=COAST){Serial.println(F("Сначала s."));return;} // Не задерживаем работающую коммутацию выводом настроек.
 Serial.print(F("ОС3 | ШИМ="));Serial.print(startupPwm);Serial.print(F("/255 @"));Serial.print(slowPwm?3921:31372);Serial.print(F("Гц | сектор="));Serial.print(FINAL_SECTOR_US); // Показываем мощность, частоту и конечный сектор в микросекундах.
 Serial.print(F("мкс | разгон="));Serial.print(rampFactor==FAST_RAMP?F("быстрый"):F("прежний"));Serial.print(F(" | напр="));Serial.print(direction);Serial.print(F(" инв="));Serial.print(invert); // Показываем профиль, направление и полярность без повторения справки.
 Serial.print(F(" | код="));Serial.print(errorCode);Serial.print(F(" ZC="));Serial.println(goodTotal); // Завершаем единственную строку кодом остановки и числом принятых пересечений.
} // Завершаем компактный отчёт.
void testPreset(){ // Объявляем единый набор настроек для повторяемого стендового опыта.
 if(mode!=IDLE){Serial.println(F("Набор p: дождитесь остановки и окончания выбега."));return;} // Запрещаем перенастройку при мощности и во время записи выбега.
 configurePwm(true);rampFactor=FAST_RAMP;startupPwm=60;direction=1;invert=false; // Одной командой выбираем диагностический ШИМ, быстрый разгон, мощность шестьдесят и прямое направление.
 report(); // Подтверждаем все настройки одной строкой без автоматического запуска и очистки журналов.
} // Завершаем установку стендового набора.
void help(){ // Объявляем отдельную справку, которую выводим только по запросу.
 if(mode!=IDLE){Serial.println(F("Справка: после окончания выбега."));return;} // Не задерживаем измерения и силовую коммутацию длинной справкой.
 Serial.println(F("U=A3 V=A2 W=A1; N=A0 через 1кОм на D6; ОС: три ЭДС и компаратор.")); // Напоминаем распиновку и алгоритм только в справке.
 Serial.println(F("p: набор теста; t: пуск ОС; o: без ОС; j: шаги; s: стоп; q: статус; k/g/d: угол/компаратор/выбег.")); // Перечисляем основные команды без повторения при каждом изменении настройки.
 Serial.println(F("b/h: 3.9/31кГц; f/l: быстрый/прежний разгон; +/-: ШИМ; r: реверс; e: полярность; ?: справка.")); // Сохраняем доступ к прежним индивидуальным настройкам.
} // Завершаем отдельную справку.
void start(){ // Объявляем ручной пуск с проверкой условий. 
 if(mode!=IDLE)return; // Запрещаем повторный пуск при уже активном управлении или выбеге. 
 readHandle(); // Перед стартом получаем актуальное положение потенциометра. 
 if(!zeroSeen||handleValue<START_HANDLE)return; // Требуем предварительный ноль ручки и последующее увеличение мощности. 
 if(operated&&millis()-offMs<COAST_MS)return; // Не запускаем ротор повторно до завершения минимального выбега. 
 vectorIndex=0;pwm=startupPwm;sectorUs=FIRST_SECTOR_US; // Начинаем с фиксированного сектора и умеренной стартовой мощности. 
 goodTotal=0;errorCode=0;confidence=0;angleCount=0;anglePeriodUs=0;firstDurationUs=0;fitDurationUs=fitFinishedUs=0;planValid=false;clippedInputs=false;fitAdcRms=fitAdcPeak=0;previousCrossKnown=false;traceMs=millis();senseCount=0;senseHead=0; // Очищаем результаты предыдущего опыта.
 mode=POSITION;enteredMs=millis();operated=true;energize(); // Включаем позиционирование и фиксируем начало опыта. 
 Serial.println(F("Позиционирование ротора.")); // Сообщаем о старте при малом потоке диагностических данных. 
} // Завершаем ручной запуск. 
void reverseRequest(){halt(0);direction=-direction;} // Сначала отключаем мост, затем меняем направление только следующего ручного пуска. 
void controls(){ // Объявляем обработку команд и кнопки с ограниченным временем выполнения. 
 if(Serial.available()){ // За один проход принимаем не более одного символа команды. 
  char c=Serial.read(); // Получаем следующий символ последовательного порта. 
  if(c=='p')testPreset();else if(c=='?')help();else if(c=='s')halt(0);else if(c=='r')reverseRequest();else if(c=='q')report();else if(c=='d')dumpCoast();else if(c=='g')dumpSense();else if(c=='k')dumpAngles(); // Выполняем основные однобуквенные команды без ожидания конца строки.
  else if((c=='t'||c=='o'||c=='j')&&mode==IDLE){launchType=c=='t'?0:(c=='o'?1:2);start();} // Выбираем обычный запуск или отдельный диагностический опыт.
  else if((c=='+'||c=='-')&&mode==IDLE){if(c=='+'&&startupPwm<90)startupPwm+=6;else if(c=='-'&&startupPwm>42)startupPwm-=6;report();} // Меняем стартовую мощность в пределах 42..90 из 255 без автоматического пуска.
  else if((c=='b'||c=='h')&&mode==IDLE){configurePwm(c=='b');report();} // Выбираем диагностический медленный или обычный быстрый ШИМ только при отключённом мосте.
  else if((c=='f'||c=='l')&&mode==IDLE){rampFactor=c=='f'?FAST_RAMP:SLOW_RAMP;report();} // Выбираем быстрый или прежний разгон только при остановленном двигателе.
  else if(c=='e'&&mode==IDLE){invert=!invert;report();} // Разрешаем изменение полярности только при отключённом мосте. 
 } // Завершаем обработку последовательного порта. 
 bool pressed=digitalRead(BUTTON);uint32_t now=millis(); // Получаем состояние кнопки и текущее время. 
 if(pressed!=rawButton){rawButton=pressed;buttonMs=now;} // Фиксируем начало возможного изменения кнопки. 
 if(pressed!=stableButton&&now-buttonMs>=35){stableButton=pressed;if(pressed==LOW)reverseRequest();} // После подавления дребезга выполняем один запрос реверса на нажатие. 
} // Завершаем обработку органов управления. 
void setup(){ // Объявляем однократную настройку после включения питания. 
 for(uint8_t i=0;i<3;i++){digitalWrite(TOP[i],LOW);pinMode(TOP[i],OUTPUT);digitalWrite(BOTTOM[i],LOW);pinMode(BOTTOM[i],OUTPUT);} // Сначала задаём безопасные уровни, затем разрешаем выходы. 
 pinMode(BUTTON,INPUT_PULLUP);pinMode(HANDLE,INPUT); // Включаем подтяжку кнопки и аналоговый вход потенциометра. 
 digitalWrite(NEUTRAL,LOW);pinMode(NEUTRAL,INPUT);DIDR1|=_BV(AIN0D); // Оставляем Д6 аналоговым входом без подтяжки. 
 configurePwm(false);adcMode(); // После включения выбираем обычный быстрый ШИМ и оставляем мост отключённым.
 Serial.begin(115200);readHandle();Serial.println(F("ОС3. p — набор теста, t — пуск, s — стоп, ? — справка."));report(); // Запускаем монитор порта, читаем ручку и выводим короткое приветствие и состояние.
} // Завершаем однократную настройку. 
void loop(){ // Объявляем основной неблокирующий цикл управления. 
 uint32_t now=micros(); // Получаем время до выполнения менее срочных действий. 
 if((mode==FOLLOW||mode==VERIFY)&&crossingFound&&dueNow(now,deadlineUs)){ // В первую очередь обслуживаем назначенную коммутацию по ЭДС.
  if(now-deadlineUs>sectorUs/6){halt(1);return;} // Отключаем мост вместо слишком позднего переключения. 
  if(mode==VERIFY&&confidence<255)confidence++; // Подтверждаем только реальные завершённые секторы управления по ЭДС.
  sectorUs=measuredUs;stepForward(); // Применяем следующий вектор уже по положению ротора.
  if(mode==VERIFY&&confidence>=CLEAN_SECTORS){mode=FOLLOW;Serial.println(F("Обратная связь захвачена."));} // Объявляем успешный захват только после требуемой непрерывной серии.
  return; // Завершаем срочную коммутацию до обработки порта.
 } // Завершаем срочную обработку коммутации. 
 controls();uint32_t ms=millis();now=micros(); // Обслуживаем управление и обновляем временные метки. 
 if(mode==IDLE||mode==COAST||mode==POSITION){ // Читаем ручку вне компараторного режима при остановке и позиционировании. 
  if(ms-handleMs>=40)readHandle(); // Ограничиваем частоту опроса потенциометра. 
 } // Завершаем фоновый опрос ручки. 
 if(mode==IDLE)return; // При отключённом двигателе ждём явной команды пуска. 
 if(mode==COAST){measureCoast();if(millis()-offMs>=COAST_MS){mode=IDLE;Serial.println(F("Выбег записан; команда d — частота и обороты."));}return;} // Выдерживаем выбег, не выполняя автоматического повторного запуска. 
 if(mode==POSITION){ // Удерживаем начальный вектор перед разгоном. 
  if(handleValue<=ZERO_HANDLE){halt(0);return;} // Нулевая ручка прекращает позиционирование. 
  if(ms-enteredMs>=POSITION_MS){mode=launchType==2?JOG:ACCELERATE;if(mode==JOG){sectorUs=250000;jogSteps=1;}stepForward();deadlineUs=startedUs+sectorUs-DEAD_TIME_US;} // После выдержки начинаем движение поля с первого реального шага. 
  return; // До окончания позиционирования другие режимы не выполняем. 
 } // Завершаем обработку позиционирования. 
 if(mode==ACCELERATE&&dueNow(now,deadlineUs)){ // Ускоряем поле только по завершении предыдущего сектора. 
  if(now-deadlineUs>sectorUs/6){halt(1);return;} // Отключаем мост при существенном опоздании программы. 
  sectorUs=sectorUs*rampFactor/1000; // Сокращаем сектор по выбранному профилю, не изменяя стартовую мощность и конечную скорость. 
  if(sectorUs<=FINAL_SECTOR_US){sectorUs=FINAL_SECTOR_US;if(launchType==1){mode=TEST_RUN;testStartedMs=ms;}else{mode=ACQUIRE;searchStartedMs=ms;}} // После разгона выбираем длительный тест или короткую выдержку перед измерением угла.
  stepForward();deadlineUs=startedUs+sectorUs-DEAD_TIME_US;return; // Коммутируем и назначаем следующий принудительный сектор. 
 } // Завершаем очередной шаг разгона. 
 if(mode==TEST_RUN||mode==JOG){ // Выполняем ограниченный диагностический опыт без компаратора и захвата ЭДС.
  if(mode==TEST_RUN&&ms-testStartedMs>=OPEN_TIME_MS){halt(0);return;} // Завершаем принудительное вращение по окончании настроенной выдержки после разгона.
  if(dueNow(micros(),deadlineUs)){ // Переключаем очередной тестовый вектор по таймеру.
   if(micros()-deadlineUs>sectorUs/6){halt(1);return;} // Отключаем мост при недопустимом опоздании даже в диагностике.
   if(mode==JOG&&jogSteps>=12){halt(0);return;} // Завершаем двенадцать медленных перемещений вместо длительного удержания током.
   stepForward();if(mode==COAST)return;deadlineUs=startedUs+sectorUs-DEAD_TIME_US; // Выполняем шаг и учитываем остановку по потенциометру.
   if(mode==JOG){jogSteps++;Serial.print(F("Шаг="));Serial.println(vectorIndex+1);} // Печатаем номер сектора только при медленной диагностике.
  } // Завершаем обслуживание тестового сектора.
  return; // Диагностика не выполняет поиск ЭДС и не может завершиться кодом отсутствия захвата.
 } // Завершаем ограниченный тест без обратной связи.
 if(mode==ACQUIRE){ // Даём ротору установиться по таймеру перед независимым измерением положения.
  if(ms-searchStartedMs>=FIELD_SETTLE_MS){beginHandover();return;} // После двух секунд прекращаем принудительную подачу мощности и измеряем три ЭДС.
  if(dueNow(now,deadlineUs)){ // Продолжаем только ограниченную начальную выдержку на конечной частоте.
   if(now-deadlineUs>sectorUs/6){halt(1);return;} // Сохраняем защиту от пропуска принудительного срока.
   stepForward();deadlineUs=startedUs+sectorUs-DEAD_TIME_US; // Назначаем очередной сектор без фиктивного поиска пересечения.
  } // Завершаем очередную коммутацию выдержки.
  return; // Не передаём управление компаратору без оценки положения.
 } // Завершаем установившуюся принудительную выдержку.
 if(mode==MEASURE||mode==WAIT_VECTOR){ // В этих состояниях мост остаётся полностью отключённым.
  if(now-quietStartedUs>QUIET_LIMIT_US){halt(10);return;} // Ограничиваем длительность измерения и запрещаем включение по устаревшему углу.
  if(mode==MEASURE&&dueNow(now,snapshotDueUs)){ // Берём снимки с заданным временным разносом без блокирующей паузы.
   if(now-snapshotDueUs>700){halt(10);return;} // Не заменяем измерение скорости серией запоздалых снимков подряд.
   takeAngleSample();if(mode!=MEASURE)return; // Учитываем слабую ЭДС или несогласованную нейтраль до любых силовых действий.
   if(angleCount==ANGLE_SAMPLES)planFirstVector();else snapshotDueUs=quietStartedUs+QUIET_BLANK_US+(uint32_t)angleCount*SNAPSHOT_GAP_US; // После восьмого снимка рассчитываем положение, иначе ждём следующий срок.
  } // Завершаем обслуживание очередного снимка.
  if(mode==WAIT_VECTOR&&dueNow(micros(),deadlineUs)){ // Включаем выбранный вектор только в рассчитанный момент.
   if(micros()-deadlineUs>sectorUs/12){halt(10);return;} // Не применяем устаревший план включения.
   vectorIndex=plannedVector;mode=FIRST_VECTOR;energize(); // Включаем вектор, выбранный по трём ЭДС, а не следующий по старой таблице разгона.
   deadlineUs=firstBoundaryUs-DEAD_TIME_US; // Первую неполную длительность назначаем по углу, даже если нулевой переход уже прошёл.
  } // Завершаем расчётное включение мощности.
  return; // Не выполняем обычную коммутацию внутри выбега или ожидания первого вектора.
 } // Завершаем этап измерения и ожидания.
 if(mode==FIRST_VECTOR){ // Первое переключение назначено по измеренному углу и ещё не требует нового нулевого перехода.
  if(dueNow(now,deadlineUs)){ // Достигаем расчётной границы первого неполного сектора.
   if(now-deadlineUs>sectorUs/12){halt(10);return;} // Снимаем мощность при запоздавшей первой коммутации.
   mode=VERIFY;searchStartedMs=millis();sensingMode();stepForward(); // Начинаем полный сектор с ожидаемым пересечением в его середине.
   if(mode==VERIFY)Serial.println(F("Проверка ОС.")); // Отделяем пробную работу по ЭДС от подтверждённого захвата.
  } // Завершаем первое расчётное переключение.
  return; // Не ищем повторно уже прошедшее пересечение в первом секторе.
 } // Завершаем первый неполный сектор.
 if(mode==FOLLOW||mode==VERIFY){ // Ведём дальнейшую коммутацию только по сигналу положения ротора.
  if(mode==VERIFY&&ms-searchStartedMs>=SEARCH_TIME_MS){halt(6);return;} // Сохраняем предел времени подтверждения последовательности.
  observeRotor();if(mode!=FOLLOW&&mode!=VERIFY)return; // Обновляем поиск пересечения с проверкой аварийной остановки.
  if(!crossingFound&&micros()-startedUs>sectorUs*3/2)halt(2); // При пропаже ЭДС снимаем мощность без перехода обратно на слепую коммутацию. 
 } // Завершаем обработку обратной связи. 
} // Завершаем основной цикл программы. 
