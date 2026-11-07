// Редакция с конечным сектором 20 мс и диагностикой разгона; управление двигателем классической Ардуино Нано. 
// Из предыдущих версий взята распиновка; алгоритм и начальные настройки написаны заново. 
// Сам скетч не выбирает плату в среде разработки: этот выбор выполняется ДО компиляции. 
// Файл sketch.yaml рядом со скетчем выбирает Нано по умолчанию для командной утилиты Ардуино. 
// В Ардуино ИДЕ нужно выбрать: Инструменты → Плата → Ардуино Нано → ATmega328P. 
// Обязательное соединение: виртуальная нейтраль А0 через резистор 1 кОм подключается к Д6. 
// Пуск с обратной связью — t; тест разгона без неё — o; медленные шаги — j; остановка — s. 
// Команды f и l выбирают быстрый и прежний разгон; g выводит диагностику компаратора; d выводит частоту выбега после шести секунд; q печатает состояние; e меняет полярность ЭДС; знаки + и - меняют стартовый ШИМ только при остановке. 
// После включения сначала повернуть ручку в ноль; после остановки новый пуск остаётся ручным. 
// Ручка регулирует мощность, а не заданные обороты; программного измерения тока здесь нет. 
// Это стендовый вариант: параметры запуска и достаточность паузы ключей проверяются на моторе. 
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
const uint32_t SEARCH_TIME_MS=15000,OPEN_TIME_MS=15000; // Даём пятнадцать секунд поиска ЭДС и столько же принудительного теста после разгона.
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
enum Mode:uint8_t {IDLE,POSITION,ACCELERATE,ACQUIRE,FOLLOW,COAST,TEST_RUN,JOG}; // Объявляем состояния алгоритма. 
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
uint16_t attempts=0; // Считаем завершённые сектора поиска без переполнения при увеличенной выдержке.
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
  traceMs=millis();Serial.print(mode==ACCELERATE?F("Разгон "):(mode==TEST_RUN?F("Тест "):F("Поиск ОС "))); // Указываем этап, на котором сейчас находится программа, а не измеренное движение ротора.
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
 uint8_t sink=DRIVE[vectorIndex][1],margin=pwm>28?pwm-28:0; // Сохраняем прежний отступ от фронтов примерно 1,75 микросекунды.
 uint8_t bit=sink==0?PB3:(sink==1?PB2:PB1); // Заранее определяем бит управляющего порта выбранного нижнего ключа.
 for(uint8_t attempt=0;attempt<32;attempt++){ // Не более тридцати двух проверок ждём подходящее плато вместо одиночного случайного чтения.
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
  uint32_t interval=at-lastCrossUs,reference=mode==FOLLOW?measuredUs:sectorUs; // Сравниваем реальный интервал с текущей оценкой. 
  if(interval<reference*7/10||interval>reference*13/10){senseNow.flags|=16;if(mode==FOLLOW)halt(3);return;} // При резком нарушении периода снимаем управление вместо слепой коммутации. 
  history[historyIndex]=interval;historyIndex=(historyIndex+1)%3; // Добавляем новый интервал в кольцевую историю. 
  measuredUs=medianPeriod(); // Получаем устойчивую оценку периода электрического сектора. 
 }else{for(uint8_t i=0;i<3;i++)history[i]=sectorUs;measuredUs=sectorUs;} // Первое пересечение инициализирует историю заданным периодом. 
 if(measuredUs<SHORTEST_US||measuredUs>LONGEST_US){halt(4);return;} // Останавливаем мост при выходе скорости за настроенный диапазон. 
 crossingUs=at;lastCrossUs=at;previousCrossKnown=true;crossingFound=true; // Сохраняем принятое пересечение и его связь с предыдущим. 
 if(goodTotal<65535)goodTotal++; // Наращиваем диагностический счётчик без переполнения. 
 if(mode==FOLLOW){ // При замкнутой коммутации назначаем следующее переключение по положению ротора. 
  deadlineUs=at+measuredUs/2-DEAD_TIME_US; // Отсчитываем тридцать электрических градусов и учитываем основную паузу ключей. 
  if(dueNow(micros(),deadlineUs))halt(5); // Не пытаемся выполнить уже опоздавшую коммутацию. 
 } // Завершаем назначение переключения по ЭДС. 
} // Завершаем наблюдение положения ротора. 
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
void report(){ // Объявляем печать настроек и состояния при отключённом мосте. 
 if(mode!=IDLE&&mode!=COAST){Serial.println(F("Сначала остановите двигатель командой s."));return;} // Не задерживаем работающую коммутацию длинным выводом. 
 Serial.println(F("Обратная связь: U=A3, V=A2, W=A1; нейтраль A0 через 1 кОм на D6.")); // Показываем соответствие фаз фактически проверенным входам платы.
 Serial.println(F("Опрос ШИМ: ожидание плато, до 32 проверок.")); // Показываем редакцию исправленного опроса в начале справки.
 Serial.print(F("Конечный сектор, мкс="));Serial.println(FINAL_SECTOR_US); // Позволяем проверить новую редакцию и конечную скорость поля до запуска.
 Serial.print(F("Разгон="));Serial.print(rampFactor==FAST_RAMP?F("быстрый"):F("прежний"));Serial.print(F("; поиск, с="));Serial.println(SEARCH_TIME_MS/1000); // Показываем профиль и длительность ожидания перед опытом.
 Serial.print(F("Код="));Serial.print(errorCode);Serial.print(F("; пересечений="));Serial.println(goodTotal); // Выводим причину остановки и число найденных переходов. 
 Serial.print(F("Направление="));Serial.print(direction);Serial.print(F("; инверсия="));Serial.println(invert); // Выводим выбранные направление и полярность. 
 Serial.print(F("Пусковой ШИМ="));Serial.print(startupPwm);Serial.println(F(" из 255; + и - меняют на 6.")); // Показываем регулируемую мощность до подачи тока.
 Serial.println(F("t — ОС; o — без ОС; j — шаги; s — стоп; d — выбег; g — компаратор; q — сведения; f/l — разгон; r — реверс; e — полярность.")); // Напоминаем команды управления. 
} // Завершаем печать состояния. 
void start(){ // Объявляем ручной пуск с проверкой условий. 
 if(mode!=IDLE)return; // Запрещаем повторный пуск при уже активном управлении или выбеге. 
 readHandle(); // Перед стартом получаем актуальное положение потенциометра. 
 if(!zeroSeen||handleValue<START_HANDLE)return; // Требуем предварительный ноль ручки и последующее увеличение мощности. 
 if(operated&&millis()-offMs<COAST_MS)return; // Не запускаем ротор повторно до завершения минимального выбега. 
 vectorIndex=0;pwm=startupPwm;sectorUs=FIRST_SECTOR_US; // Начинаем с фиксированного сектора и умеренной стартовой мощности. 
 goodTotal=0;errorCode=0;confidence=0;attempts=0;previousCrossKnown=false;traceMs=millis();senseCount=0;senseHead=0; // Очищаем результаты предыдущего опыта. 
 mode=POSITION;enteredMs=millis();operated=true;energize(); // Включаем позиционирование и фиксируем начало опыта. 
 Serial.println(F("Позиционирование ротора.")); // Сообщаем о старте при малом потоке диагностических данных. 
} // Завершаем ручной запуск. 
void reverseRequest(){halt(0);direction=-direction;} // Сначала отключаем мост, затем меняем направление только следующего ручного пуска. 
void controls(){ // Объявляем обработку команд и кнопки с ограниченным временем выполнения. 
 if(Serial.available()){ // За один проход принимаем не более одного символа команды. 
  char c=Serial.read(); // Получаем следующий символ последовательного порта. 
  if(c=='s')halt(0);else if(c=='r')reverseRequest();else if(c=='q')report();else if(c=='d')dumpCoast();else if(c=='g')dumpSense(); // Выполняем основные однобуквенные команды без ожидания конца строки. 
  else if((c=='t'||c=='o'||c=='j')&&mode==IDLE){launchType=c=='t'?0:(c=='o'?1:2);start();} // Выбираем обычный запуск или отдельный диагностический опыт.
  else if((c=='+'||c=='-')&&mode==IDLE){if(c=='+'&&startupPwm<90)startupPwm+=6;else if(c=='-'&&startupPwm>42)startupPwm-=6;report();} // Меняем стартовую мощность в пределах 42..90 из 255 без автоматического пуска.
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
 TCCR1A=_BV(WGM10);TCCR1B=_BV(CS10); // Настраиваем восьмиразрядный двухскатный ШИМ таймера 1 без делителя. 
 TCCR2A=_BV(WGM20);TCCR2B=_BV(CS20); // Настраиваем такой же ШИМ таймера 2 для нижнего ключа У. 
 OCR1A=0;OCR1B=0;OCR2A=0;bridgeOff();adcMode(); // Обнуляем заполнение и гарантированно снимаем управление мостом. 
 Serial.begin(115200);readHandle();report(); // Запускаем монитор порта, читаем ручку и выводим русскую справку. 
} // Завершаем однократную настройку. 
void loop(){ // Объявляем основной неблокирующий цикл управления. 
 uint32_t now=micros(); // Получаем время до выполнения менее срочных действий. 
 if(mode==FOLLOW&&crossingFound&&dueNow(now,deadlineUs)){ // В первую очередь обслуживаем назначенную коммутацию по ЭДС. 
  if(now-deadlineUs>sectorUs/6){halt(1);return;} // Отключаем мост вместо слишком позднего переключения. 
  sectorUs=measuredUs;stepForward();return; // Применяем следующий вектор с обновлённой оценкой периода. 
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
  if(sectorUs<=FINAL_SECTOR_US){sectorUs=FINAL_SECTOR_US;if(launchType==1){mode=TEST_RUN;testStartedMs=ms;}else{mode=ACQUIRE;searchStartedMs=ms;sensingMode();}} // После разгона начинаем наблюдение ЭДС на постоянной частоте. 
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
 if(mode==ACQUIRE){ // Продолжаем принудительную коммутацию, проверяя пригодность ЭДС. 
  if(ms-searchStartedMs>=SEARCH_TIME_MS){halt(6);return;} // Завершаем неудачный поиск только по новой пятнадцатисекундной выдержке.
  observeRotor();if(mode!=ACQUIRE)return; // Проверяем пересечение и учитываем возможное аварийное отключение. 
  if(dueNow(micros(),deadlineUs)){ // Анализируем полностью завершившийся сектор. 
   if(micros()-deadlineUs>sectorUs/6){halt(1);return;} // Не допускаем догоняющей серии пропущенных переключений. 
   if(crossingFound)confidence++;else{confidence=0;previousCrossKnown=false;} // Наращиваем только непрерывную серию подтверждённых секторов. 
   if(attempts<65535)attempts++; // Сохраняем число проверенных секторов без прежнего ограничения в девяносто шагов. 
   bool captured=confidence>=CLEAN_SECTORS;stepForward(); // Переходим к следующему сектору после оценки текущего. 
   if(mode!=ACQUIRE)return; // Учитываем остановку по нулевой ручке при коммутации. 
   if(captured){mode=FOLLOW;sectorUs=measuredUs;Serial.println(F("Обратная связь захвачена."));} // Передаём назначение следующих коммутаций ротору. 
   else deadlineUs=startedUs+sectorUs-DEAD_TIME_US; // До захвата сохраняем только принудительную частоту поля. 
  } // Завершаем обработку окончания сектора захвата. 
  return; // Не выполняем обработчик замкнутого режима в том же проходе. 
 } // Завершаем обработку поиска синхронизации. 
 if(mode==FOLLOW){ // Ведём дальнейшую коммутацию только по сигналу положения ротора. 
  observeRotor();if(mode!=FOLLOW)return; // Обновляем поиск пересечения с проверкой аварийной остановки. 
  if(!crossingFound&&micros()-startedUs>sectorUs*3/2)halt(2); // При пропаже ЭДС снимаем мощность без перехода обратно на слепую коммутацию. 
 } // Завершаем обработку обратной связи. 
} // Завершаем основной цикл программы. 
