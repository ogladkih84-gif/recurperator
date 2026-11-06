/*
  BMLC-175A v1.2c — аппаратный компаратор BEMF / экспериментальная ОС.
  Arduino Nano, ATmega328P, 16 MHz. Русские комментарии.

  КРИТИЧЕСКОЕ НОВОЕ СОЕДИНЕНИЕ:
    A0 (уже подключенная делённая виртуальная нейтраль N) -- 1 кОм -- D6 (AIN0).
  D6 должен оставаться ВХОДОМ БЕЗ ПОДТЯЖКИ. На обоих входах 0..5 В.
  АЦП отключается ТОЛЬКО на время компараторного наблюдения: ACME
  выбирает на отрицательный вход компаратора свободную фазу A1/A2/A3.
  На положительном входе D6 находится N. ACO=1 означает N > phase.
  Измерения проводятся при включённом нижнем MOSFET внутри ШИМ,
  без полного отключения моста, в отличие от q1 в v1.1.

  f0  обычная v0.9p по таймеру, без использования компаратора (по умолчанию).
  f1  SHADOW — диагностика компаратора, переключение по таймеру.
  f2  ARMED — после 12 ПОДРЯД корректных секторов разрешить короткий
      эксперимент TRACKING с управлением по ZC; при ошибке МОСТ OFF.
  Любой сомнительный сигнал => БЕЗ перехода в TRACKING.

  p28 выставить фиксированный ШИМ 28%; p60 = 60% (НЕ повышать без
  контроля тока). p0: считать A4 перед пуском, диапазон 20..70%,
  далее заморозить на время опыта: АЦП занят компаратором.

  t — запуск (ALIGN 250 мс, RAMP 6000 мс, HOLD 2500 мс), затем COAST.
  s — снять все ключи немедленно. d — CSV выбега, g — события компаратора.
  i12 — целевой шаг 12 мс (от 10 до 40 мс). ? — настройки.

  Запуск ТОЛЬКО с лабораторным ограничением тока и предохранителем.
  Нет аппаратного датчика тока, защита от КЗ и остановки не гарантируется.
  f2 НЕ ИСПОЛЬЗОВАТЬ до проверки сигнала D6 и журнала g из f1.
*/
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// --------------------- ВЫВОДЫ ---------------------
const uint8_t UH=7, UL=11, VH=4, VL=10, WH=2, WL=9;
const uint8_t POT=A4, REV=8;
const uint8_t BEMF_U=A3, BEMF_V=A2, BEMF_W=A1, BEMF_N=A0;
const uint8_t N_COMPARATOR_PIN=6; // AIN0: нейтраль N через 1 кОм.
const uint8_t pins[6]={UH,UL,VH,VL,WH,WL};
const uint8_t steps[6][6]={
  {1,0,0,1,0,0}, // 1: U+ V- W float
  {1,0,0,0,0,1}, // 2: U+ W- V float
  {0,0,1,0,0,1}, // 3: V+ W- U float
  {0,1,1,0,0,0}, // 4: V+ U- W float
  {0,1,0,0,1,0}, // 5: W+ U- V float
  {0,0,0,1,1,0}  // 6: W+ V- U float
};
// Канал MUX отрицательного входа компаратора: W=A1, V=A2, U=A3.
const uint8_t freeMux[6]={1,2,3,1,2,3};
// Нижний MOSFET: Timer1 для D9/D10, Timer2 для D11.
const uint8_t lowIsTimer2[6]={0,0,0,1,1,0};

const uint32_t ALIGN_MS=250, RAMP_MS=6000, HOLD_MS=2500;
const uint32_t START_STEP_US=100000UL;
const uint16_t DEAD_US=100;
const uint8_t COAST_COUNT=64; // 64*8 = 512 байт SRAM.
const uint32_t COAST_GAP_US=6000, COAST_BLANK_US=2000;
const uint8_t EVENTS=72; // 72*6 = 432 байта SRAM.
const uint32_t BLANK_US=1300; // Подавление коммутационных переходов.
const uint32_t STABLE_US=150; // Подтверждение знака в нескольких PWM циклах.
const uint32_t MIN_TRACK_STEP_US=7500, MAX_TRACK_STEP_US=26000;
const uint32_t TRACK_LIMIT_MS=1500; // Короткая работа с обратной связью.
const uint8_t LOCK_REQUIRED=12; // 12 последовательных подтверждённых ZC.
const uint8_t PWM_GATE_COUNT=12; // Отступ в счётчиках таймера от фронтов ШИМ.

struct Coast {uint16_t u,v,w,n;};
Coast coast[COAST_COUNT];
uint8_t coastLen=0;
bool coastAuto=false;
uint32_t nextCoastUs=0, coastStartUs=0;

// Сохраняем завершённые секторы: 1 номер, 1 флаг, 2 возраст ZC, 2 время сектора.
struct Event {uint16_t zcAge,stepUs; uint8_t step,flag;};
Event ev[EVENTS];
uint8_t evHead=0, evLen=0;
// 0=нет достоверного начального уровня; 1=нет ZC; 2=ZC корректен;
// 3=неверное направление; 4=пересечение вне допустимого времени;
// 5=неправильный стартовый уровень.

// Режим f2 НЕ запускается автоматически после прошивки.
uint8_t feedbackMode=0;
uint16_t targetStepMs=12;
uint8_t pwmPercent=28, pwmDuty=71;
bool pwmFromPot=false;
uint8_t zcInvert=0; // e0/e1: исключительно при подтверждении фазы на осциллографе.

// STOP/ALIGN/RAMP/HOLD/TRACK/COAST.
enum State { STOP, ALIGN, RAMP, HOLD, TRACK, COAST };
State state=STOP;
uint32_t stateStartMs=0, lastStepUs=0, stepStartUs=0;
uint32_t stepIntervalUs=START_STEP_US;
uint8_t sector=0;
bool reversed=false, compEnabled=false;
uint32_t trackingStartMs=0, nextTrackUs=0;
uint8_t streak=0;
uint16_t okZc=0, badZc=0, missingZc=0;

// Данные фильтра на один коммутационный сектор.
bool baseReady=false, eventSeen=false;
uint8_t baseLevel=0, candidateLevel=0, lastStableLevel=0;
uint8_t sampleCount=0;
uint32_t candidateSince=0, zcAtUs=0;
uint16_t zcAgeUs=0;
uint8_t sectorError=0;

char cmd[24]; uint8_t cmdLen=0;

// --------------------- СИЛОВОЙ МОСТ ---------------------
void allOff(){
  analogWrite(UL,0); analogWrite(VL,0); analogWrite(WL,0);
  digitalWrite(UH,LOW); digitalWrite(VH,LOW); digitalWrite(WH,LOW);
}
void pwmInit(){
  // 31,37 кГц, 8-битный phase correct, делитель 1.
  TCCR1A=_BV(WGM10); TCCR1B=_BV(CS10);
  TCCR2A=_BV(WGM20); TCCR2B=_BV(CS20);
  OCR1A=0; OCR1B=0; OCR2A=0;
}
void setDuty(uint8_t percent){
  pwmPercent=percent;
  pwmDuty=(uint16_t)percent*255UL/100UL;
}
void applyStep(uint8_t s){
  if(s>=6){allOff();state=STOP;return;}
  // Полное выключение перед каждым сектором: гарантированная пауза 100 мкс.
  allOff();
  delayMicroseconds(DEAD_US);
  // Ни в одном секторе не должно быть верхнего и нижнего одного плеча.
  for(uint8_t p=0;p<6;p+=2)
    if(steps[s][p] && steps[s][p+1]){allOff();state=STOP;return;}
  // Сначала подать HIGH верхнему, затем ШИМ только нужному нижнему.
  for(uint8_t p=0;p<6;p+=2) if(steps[s][p]) digitalWrite(pins[p],HIGH);
  for(uint8_t p=1;p<6;p+=2) if(steps[s][p]) analogWrite(pins[p],pwmDuty);
}
void comparatorOff(){
  ACSR |= _BV(ACD);           // Компаратор выключен.
  ADCSRB &= (uint8_t)~_BV(ACME); // Отсоединить ADC MUX от компаратора.
  ADCSRA |= _BV(ADEN);       // Вернуть АЦП для A4 и выбега.
  compEnabled=false;
}
void selectFreePhase(){
  // ADEN=0: MUX[3:0] используется как минусовой вход компаратора.
  ADMUX=(ADMUX & 0xF0) | freeMux[sector];
  // Нет нужды ждать analogRead(), выбор происходит аппаратно.
}
void comparatorOn(){
  ADCSRA &= (uint8_t)~_BV(ADEN); // Отключить ADC перед ACME.
  ADCSRB |= _BV(ACME);
  ACSR &= (uint8_t)~_BV(ACD);   // Разрешить компаратор.
  ACSR &= (uint8_t)~_BV(ACIE);  // Без прерываний: опрос в loop().
  ACSR |= _BV(ACI);             // Сбросить возможный флаг прерывания.
  compEnabled=true;
  selectFreePhase();
}
uint16_t stableAdc(uint8_t pin){analogRead(pin);return analogRead(pin);}
void startCoast(bool automatic){
  allOff(); comparatorOff();
  coastAuto=automatic; coastLen=0;
  coastStartUs=micros();nextCoastUs=coastStartUs+COAST_BLANK_US;
  state=COAST;
  Serial.println(F("AUTO COAST: ALL OFF"));
}
void stopNow(){
  allOff();comparatorOff();state=STOP;
  Serial.println(F("STOP: ALL MOSFETS OFF"));
}

// --------------------- КОМПАРАТОР + ФИЛЬТР ---------------------
void resetSectorFilter(){
  baseReady=false;eventSeen=false;sampleCount=0;
  candidateSince=0;zcAtUs=0;zcAgeUs=0;sectorError=0;
  if(compEnabled)selectFreePhase();
}
void saveEvent(uint8_t flag,uint32_t lengthUs){
  Event &r=ev[evHead];r.step=sector+1;r.flag=flag;
  r.zcAge=zcAgeUs;
  r.stepUs=(lengthUs>65535UL)?65535:(uint16_t)lengthUs;
  evHead=(evHead+1)%EVENTS;
  if(evLen<EVENTS)evLen++;
}
// При смене сектора оцениваем оконченный сектор, а не следующий.
void finishSector(uint32_t oldLength){
  if(!compEnabled)return;
  uint8_t flag=eventSeen?2:(sectorError?sectorError:(baseReady?1:0));
  saveEvent(flag,oldLength);
  if(flag==2){okZc++;if(streak<250)streak++;}
  else {badZc++;if(flag==1)missingZc++;streak=0;}
}
void nextSector(uint32_t now){
  finishSector(now-stepStartUs);
  sector=(sector+(reversed?5:1))%6;
  applyStep(sector);
  stepStartUs=micros();
  resetSectorFilter();
}
// Разрешено опрашивать ACO лишь посреди ВКЛЮЧЕННОГО участка нижнего PWM.
// Это существенно уменьшает помехи от плавающей фазы при свободном ходе.
bool pwmPlateau(){
  const uint8_t cnt=lowIsTimer2[sector]?TCNT2:TCNT1;
  // Рядом с BOTTOM и фронтами ШИМ выборку не проводим.
  return pwmDuty>(2*PWM_GATE_COUNT+8) &&
         cnt>PWM_GATE_COUNT && cnt<(uint8_t)(pwmDuty-PWM_GATE_COUNT);
}
// Для прямого обхода шагов теоретическая последовательность:
// 1 W падает (N-W: 0->1), 2 V растёт (1->0), и далее чередуется.
// Фактическая полярность уточняется режимом f1 (параметр e0/e1).
uint8_t expectedAfter(){
  const uint8_t standard=(sector%2==0)?1:0;
  return standard ^ (reversed?1:0) ^ zcInvert;
}
void observeComparator(uint32_t now){
  if(!compEnabled || eventSeen)return;
  const uint32_t age=now-stepStartUs;
  if(age<BLANK_US || age>stepIntervalUs-700UL) return;
  if(!pwmPlateau()) return;
  uint8_t level=(ACSR & _BV(ACO))?1:0;
  // Первая выборка и изменение: запускаем счетчик устойчивости.
  if(sampleCount==0 || level!=candidateLevel){
    candidateLevel=level;candidateSince=now;sampleCount=1;return;
  }
  if(sampleCount<255)sampleCount++;
  if(sampleCount<4 || now-candidateSince<STABLE_US)return;
  if(!baseReady){
    // Базовое состояние берём ДО ожидаемого пересечения.
    if(age>stepIntervalUs*35UL/100UL){sectorError=5;return;}
    baseReady=true;baseLevel=level;lastStableLevel=level;
    return;
  }
  if(level==lastStableLevel)return;
  lastStableLevel=level;
  // Рассматриваем только переход из начальной полярности.
  if(level==baseLevel)return;
  if(level!=expectedAfter()){sectorError=3;return;}
  if(age<stepIntervalUs*30UL/100UL ||
     age>stepIntervalUs*70UL/100UL){sectorError=4;return;}
  // Задержка подтверждения ~150 мкс сдвигает оценку ZC; компенсируем.
  zcAtUs=candidateSince;
  zcAgeUs=(uint16_t)(zcAtUs-stepStartUs);
  eventSeen=true;
  // Для tracking следующее переключение приблизительно через 30 эл. градусов:
  // от ZC до следующего сектора столько же, сколько от начала сектора до ZC.
  if(state==TRACK){
    nextTrackUs=zcAtUs+(uint32_t)zcAgeUs;
  }
}

// --------------------- РАЗГОН / ИЗМЕРЕНИЯ ---------------------
uint32_t rampInterval(uint32_t elapsed){
  if(elapsed>=RAMP_MS)return (uint32_t)targetStepMs*1000UL;
  const uint32_t startF=1000000000UL/START_STEP_US;
  const uint32_t endF=1000000UL/targetStepMs;
  // При i10..i40 максимум произведения < 600 млн: uint32_t достаточно.
  const uint32_t f=startF+(endF-startF)*elapsed/RAMP_MS;
  return 1000000000UL/f;
}
void startMotor(){
  if(state!=STOP){Serial.println(F("STOP FIRST"));return;}
  allOff();comparatorOff();
  reversed=(digitalRead(REV)==LOW);
  if(feedbackMode==2 && reversed){
    Serial.println(F("FAULT: f2 REVERSE NOT VALIDATED"));return;
  }
  if(pwmFromPot){
    const int adc=analogRead(POT);
    setDuty((uint8_t)map(adc,0,1023,20,70));
  }
  // Строго проверить, что новая перемычка не является выходом.
  pinMode(N_COMPARATOR_PIN,INPUT);digitalWrite(N_COMPARATOR_PIN,LOW);
  sector=0;evLen=evHead=0;coastLen=0;
  okZc=badZc=missingZc=0;streak=0;
  stepIntervalUs=START_STEP_US;
  applyStep(sector);state=ALIGN;
  stateStartMs=millis();stepStartUs=micros();lastStepUs=stepStartUs;
  Serial.print(F("ALIGN START; PWM="));Serial.print(pwmPercent);
  Serial.println(F("%; f0/f1/f2; WATCH CURRENT"));
}
void dumpEvents(){
  if(state!=STOP){Serial.println(F("STOP FIRST"));return;}
  Serial.println(F("step,flag,zc_age_us,sector_us"));
  Serial.println(F("flag: 0=no_base,1=no_ZC,2=valid,3=wrong_polarity,4=wrong_time,5=late_base"));
  uint8_t index=(evHead+EVENTS-evLen)%EVENTS;
  for(uint8_t k=0;k<evLen;k++){
    const Event &r=ev[index];
    Serial.print(r.step);Serial.print(',');Serial.print(r.flag);Serial.print(',');
    Serial.print(r.zcAge);Serial.print(',');Serial.println(r.stepUs);
    index=(index+1)%EVENTS;
  }
  Serial.print(F("valid="));Serial.print(okZc);
  Serial.print(F(" bad="));Serial.print(badZc);
  Serial.print(F(" missing="));Serial.println(missingZc);
}
void dumpCoast(){
  if(state!=STOP){Serial.println(F("STOP FIRST"));return;}
  if(coastLen==0){Serial.println(F("NO COAST DATA"));return;}
  Serial.println(F("ms,U,V,W,N,U-N,V-N,W-N"));
  for(uint8_t k=0;k<coastLen;k++){
    const Coast &r=coast[k];
    Serial.print((COAST_BLANK_US+(uint32_t)k*COAST_GAP_US)/1000UL);
    Serial.print(',');Serial.print(r.u);Serial.print(',');
    Serial.print(r.v);Serial.print(',');Serial.print(r.w);Serial.print(',');
    Serial.print(r.n);Serial.print(',');Serial.print((int)r.u-(int)r.n);
    Serial.print(',');Serial.print((int)r.v-(int)r.n);
    Serial.print(',');Serial.println((int)r.w-(int)r.n);
  }
  Serial.println(F("END LOG"));
}
void printSettings(){
  Serial.println(F("BMLC-175A v1.2c BEMF COMPARATOR"));
  Serial.print(F("f="));Serial.print(feedbackMode);
  Serial.print(F("; i="));Serial.print(targetStepMs);
  Serial.print(F("ms; field RPM="));Serial.print(5000UL/targetStepMs);
  Serial.print(F("; PWM="));Serial.print(pwmPercent);
  Serial.print(F("%; source="));Serial.println(pwmFromPot?F("A4 (start only)"):F("fixed"));
  Serial.print(F("e="));Serial.println(zcInvert);
  Serial.println(F("t start, s stop, d coast, g comparator log, ? settings"));
  Serial.println(F("f0 timer; f1 shadow; f2 armed closed-loop (EXPERIMENTAL)"));
  Serial.println(F("p20..p70 fixed PWM; p0=A4 20..70% (sampled before start)"));
  Serial.println(F("i10..i40 step; e0/e1 invert expected ZC polarity"));
  Serial.println(F("D6 must be connected to A0 through 1k for f1/f2."));
}
void processCmd(char *s){
  if(!strcmp(s,"s")){stopNow();return;}
  if(!strcmp(s,"?")){printSettings();return;}
  if(!strcmp(s,"g")){dumpEvents();return;}
  if(!strcmp(s,"d")){dumpCoast();return;}
  if(!strcmp(s,"t")){startMotor();return;}
  if(state!=STOP){Serial.println(F("STOP FIRST"));return;}
  if(s[0]=='f' && s[1]>='0' && s[1]<='2' && !s[2]){
    feedbackMode=s[1]-'0';printSettings();return;
  }
  if(s[0]=='e' && (s[1]=='0'||s[1]=='1') && !s[2]){
    zcInvert=s[1]-'0';printSettings();return;
  }
  if(s[0]=='p' || s[0]=='i'){
    char *end=NULL;long v=strtol(s+1,&end,10);
    if(end==s+1 || *end){Serial.println(F("BAD NUMBER"));return;}
    if(s[0]=='p'){
      if(v==0){pwmFromPot=true;Serial.println(F("A4 PWM AT START"));return;}
      if(v<20 || v>70){Serial.println(F("PWM 20..70"));return;}
      pwmFromPot=false;setDuty((uint8_t)v);printSettings();return;
    }
    if(v<10 || v>40){Serial.println(F("STEP 10..40 ms"));return;}
    targetStepMs=(uint16_t)v;printSettings();return;
  }
  Serial.println(F("UNKNOWN COMMAND"));
}
void pollSerial(){
  while(Serial.available()){
    const char c=(char)Serial.read();
    if(c=='\r'||c=='\n'){
      if(cmdLen){cmd[cmdLen]=0;processCmd(cmd);cmdLen=0;}
    }else if(cmdLen<sizeof(cmd)-1){cmd[cmdLen++]=c;}
    else {cmdLen=0;Serial.println(F("COMMAND TOO LONG"));}
  }
}
void setup(){
  // Поднять сигналы только после установки выхода в LOW.
  for(uint8_t i=0;i<6;i++){digitalWrite(pins[i],LOW);pinMode(pins[i],OUTPUT);}
  pinMode(POT,INPUT);pinMode(REV,INPUT_PULLUP);
  pinMode(N_COMPARATOR_PIN,INPUT);digitalWrite(N_COMPARATOR_PIN,LOW);
  allOff();pwmInit();comparatorOff();
  analogReference(DEFAULT);
  Serial.begin(115200);
  setDuty(28);printSettings();
}
void loop(){
  pollSerial();
  if(state==STOP)return;
  if(state==COAST){
    if(coastLen>=COAST_COUNT){state=STOP;Serial.println(F("CAPTURE COMPLETE: g / d"));return;}
    const uint32_t t=micros();
    if((int32_t)(t-nextCoastUs)>=0){
      Coast &r=coast[coastLen++];
      r.u=stableAdc(BEMF_U);r.v=stableAdc(BEMF_V);
      r.w=stableAdc(BEMF_W);r.n=stableAdc(BEMF_N);
      nextCoastUs+=COAST_GAP_US;
    }
    return;
  }
  const uint32_t nowMs=millis();
  if(state==ALIGN){
    if(nowMs-stateStartMs>=ALIGN_MS){
      state=RAMP;stateStartMs=millis();lastStepUs=micros();
      Serial.println(F("RAMP START"));
    }return;
  }
  if(state==RAMP){
    const uint32_t elapsed=nowMs-stateStartMs;
    if(elapsed>=RAMP_MS){
      state=HOLD;stateStartMs=millis();lastStepUs=micros();
      stepStartUs=lastStepUs;stepIntervalUs=(uint32_t)targetStepMs*1000UL;
      if(feedbackMode)comparatorOn();
      resetSectorFilter();
      Serial.println(F("HOLD START; COMPARATOR SHADOW"));
      return;
    }
    stepIntervalUs=rampInterval(elapsed);
    const uint32_t t=micros();
    if(t-lastStepUs>=stepIntervalUs){
      lastStepUs=t;
      sector=(sector+(reversed?5:1))%6;
      applyStep(sector);
    }
    return;
  }
  if(state==HOLD || state==TRACK){
    const uint32_t t=micros();
    // Проверять компаратор по возможной полной частоте loop().
    observeComparator(t);
    if(state==HOLD){
      if(nowMs-stateStartMs>=HOLD_MS){
        finishSector(micros()-stepStartUs);
        Serial.print(F("SHADOW DONE valid="));Serial.print(okZc);
        Serial.print(F(" bad="));Serial.println(badZc);
        startCoast(true);return;
      }
      if(t-lastStepUs>=stepIntervalUs){
        lastStepUs=t;
        nextSector(t);
        // ПЕРЕХОД В TRACK ТОЛЬКО ПОСЛЕ подтверждённых 12 СЕКТОРОВ.
        if(feedbackMode==2 && streak>=LOCK_REQUIRED && !reversed){
          // Планируем от ZC ПРЕДЫДУЩЕГО сектора; после nextSector он уже
          // закончился. Для нового сектора следующий ZC будет найден заново.
          state=TRACK;trackingStartMs=millis();nextTrackUs=0;
          Serial.println(F("BEMF LOCK: TRACK 1.5s MAX"));
        }
      }
      return;
    }
    // TRACK: никаких принудительных переходов без подтвержденного ZC.
    if(nowMs-trackingStartMs>=TRACK_LIMIT_MS){
      finishSector(micros()-stepStartUs);
      Serial.println(F("TRACK TIME LIMIT -> COAST"));
      startCoast(true);return;
    }
    if(t-stepStartUs>MAX_TRACK_STEP_US ||
       (eventSeen && (zcAgeUs<MIN_TRACK_STEP_US/3 || zcAgeUs>MAX_TRACK_STEP_US/2))){
      Serial.println(F("BEMF FAULT: TIMEOUT / BAD ZC"));
      stopNow();return;
    }
    if(eventSeen && nextTrackUs && (int32_t)(t-nextTrackUs)>=0){
      const uint32_t oldLen=t-stepStartUs;
      if(oldLen<MIN_TRACK_STEP_US || oldLen>MAX_TRACK_STEP_US){
        Serial.println(F("BEMF FAULT: SECTOR OUT OF RANGE"));stopNow();return;
      }
      stepIntervalUs=oldLen;
      nextSector(t);
      nextTrackUs=0;
    }
    return;
  }
}
