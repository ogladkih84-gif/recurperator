/* BMLC-175A v1.3: experimental sensorless six-step feedback, Nano classic 16 MHz.
   Pinout from user's v1.2c. REQUIRED: A0 neutral -- 1k -- D6/AIN0.
   D6 is INPUT without pullup. No current sensing. Use current-limited supply.
   Default f1 = shadow test; f2 = acquire + continuous feedback until stop/fault.
   Serial 115200/newline: t s f0/f1/f2 i2..i40 p20..p70 p0 e0/e1 v g d ?
   p0 = live potentiometer (power, NOT regulated RPM). PWM remains 31.37 kHz.
   No interrupts used for comparator. ADC is used only inside commutation blanking.
   Gate DEAD_US and expected polarity must be verified on the physical board.
*/
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#if !defined(F_CPU) || F_CPU != 16000000UL
#error This sketch requires 16 MHz
#endif
const uint8_t UH=7,UL=11,VH=4,VL=10,WH=2,WL=9,REV=8,POT=A4;
const uint8_t pins[6]={UH,UL,VH,VL,WH,WL};
const uint8_t steps[6][6]={
 {1,0,0,1,0,0},{1,0,0,0,0,1},{0,0,1,0,0,1},
 {0,1,1,0,0,0},{0,1,0,0,1,0},{0,0,0,1,1,0}};
const uint8_t freeMux[6]={1,2,3,1,2,3};
const uint8_t lowTimer2[6]={0,0,0,1,1,0};
const uint8_t lowMask[6]={_BV(PB2),_BV(PB1),_BV(PB1),_BV(PB3),_BV(PB3),_BV(PB2)};
const uint16_t DEAD_US=100; // provisional: verify MOSFET Vgs turn-off!
const uint32_t ALIGN_MS=250,RAMP_MS=6000,SHADOW_MS=3000;
const uint32_t START_US=100000UL,MIN_US=1500,MAX_US=45000;
const uint16_t CONFIRM_US=80; // confirmation delay is removed by timestamping first sample
const uint8_t LOCK_REQUIRED=12,EVENTS=64,COAST_COUNT=64;
// Allow 2 us for switch/comparator settling on each edge; BOTTOM is NOT an edge.
const uint8_t PWM_GUARD=32;
struct Event { uint16_t zc,period; uint8_t sector,flag; };
struct Coast { uint16_t time100us,u,v,w,n; };
Event events[EVENTS]; Coast coast[COAST_COUNT];
uint8_t eventHead=0,eventLen=0,coastLen=0;
enum State:uint8_t {STOP,ALIGN,RAMP,SHADOW,TRACK,COAST};
State state=STOP;
uint8_t sector=0,feedback=1,invertZc=0,powerTarget=28,pwmDuty=71;
uint16_t targetStepMs=12;
bool fromPot=false,reverseNext=false,reversed=false,hasRun=false,compEnabled=false;
uint32_t stateAtMs=0,stepAt=0,due=0,periodUs=START_US,estimateUs=12000;
uint32_t powerOffAt=0,potAtMs=0,slewAtMs=0,coastAt=0,coastDue=0;
uint32_t lastZcAt=0,previousZcAt=0,candidateAt=0,zcAt=0;
bool havePreviousZc=false,candidateValid=false,baseReady=false,eventSeen=false,scheduled=false;
uint8_t candidate=0,samples=0,sectorError=0,streak=0;
uint16_t validZc=0,badZc=0;
uint8_t lastFault=0;
bool buttonRaw=HIGH,buttonStable=HIGH;
uint32_t buttonAt=0;
char command[24];uint8_t commandLen=0;bool discardLine=false;

bool reached(uint32_t now,uint32_t deadline){return (int32_t)(now-deadline)>=0;}
void allOff(){
 analogWrite(UL,0);analogWrite(VL,0);analogWrite(WL,0);
 digitalWrite(UH,LOW);digitalWrite(VH,LOW);digitalWrite(WH,LOW);
}
void pwmInit(){
 TCCR1A=_BV(WGM10);TCCR1B=_BV(CS10);
 TCCR2A=_BV(WGM20);TCCR2B=_BV(CS20);
 OCR1A=OCR1B=OCR2A=0;
}
void comparatorOff(){
 ACSR=_BV(ACD)|_BV(ACI); // ACIE=0, ACBG=0; ACI is write-one-to-clear
 ADCSRB &= (uint8_t)~_BV(ACME);
 ADCSRA |= _BV(ADEN);compEnabled=false;
}
void comparatorOn(){
 ADCSRA &= (uint8_t)~_BV(ADEN);
 ADMUX=_BV(REFS0)|freeMux[sector];
 ADCSRB |= _BV(ACME);
 ACSR=_BV(ACI); // AIN0 positive, ADC mux negative, no interrupts
 compEnabled=true;
}
uint16_t readPot(){analogRead(POT);return analogRead(POT);}
void dutyPercent(uint8_t p){pwmDuty=(uint16_t)p*255/100;}
void resetFilter(){
 candidateValid=false;baseReady=false;eventSeen=false;scheduled=false;
 samples=0;sectorError=0;zcAt=0;
 if(compEnabled)ADMUX=_BV(REFS0)|freeMux[sector];
}
void applySector(){
 // Retain a common upper switch instead of charging/discharging its slow gate
 // unnecessarily twice per conduction interval. All lower switches go OFF first.
 analogWrite(UL,0);analogWrite(VL,0);analogWrite(WL,0);
 for(uint8_t p=0;p<6;p+=2)if(!steps[sector][p])digitalWrite(pins[p],LOW);
 delayMicroseconds(DEAD_US);
 for(uint8_t p=0;p<6;p+=2)if(steps[sector][p])digitalWrite(pins[p],HIGH);
 for(uint8_t p=1;p<6;p+=2)if(steps[sector][p])analogWrite(pins[p],pwmDuty);
 stepAt=micros();resetFilter();
}
void saveEvent(){
 if(!compEnabled)return;
 Event &e=events[eventHead];e.sector=sector+1;
 e.flag=eventSeen?2:(sectorError?sectorError:(baseReady?1:0));
 uint32_t age=eventSeen?zcAt-stepAt:0;
 e.zc=age>65535?65535:age;e.period=periodUs>65535?65535:periodUs;
 eventHead=(eventHead+1)%EVENTS;if(eventLen<EVENTS)eventLen++;
 if(e.flag==2){validZc++;if(streak<250)streak++;}
 else{badZc++;streak=0;havePreviousZc=false;}
}
void stopMotor(uint8_t fault=0){
 bool powered=(state==ALIGN||state==RAMP||state==SHADOW||state==TRACK);
 allOff();comparatorOff();
 if(powered)powerOffAt=millis();
 state=STOP;lastFault=fault;
 // Never print before removing bridge power.
 if(fault){Serial.print(F("FAULT "));Serial.println(fault);}
 else Serial.println(F("STOP"));
}
void coastStart(){
 allOff();comparatorOff();powerOffAt=millis();
 coastLen=0;coastAt=micros();coastDue=coastAt+2000;state=COAST;
 Serial.println(F("COAST"));
}
uint32_t rampPeriod(uint32_t elapsed){
 if(elapsed>=RAMP_MS)return (uint32_t)targetStepMs*1000;
 uint32_t first=1000000000UL/START_US,last=1000000UL/targetStepMs;
 return 1000000000UL/(first+(last-first)*elapsed/RAMP_MS);
}
// Only called immediately after commutation, inside the blanking interval.
// Fast ADC /32: first conversion after ADEN ~50 us, subsequent ~26 us.
void powerService(){
 uint32_t now=millis();
 if(fromPot && now-potAtMs>=50){
  potAtMs=now;bool restore=compEnabled;comparatorOff();
  uint8_t old=ADCSRA;
  ADCSRA=(ADCSRA & (uint8_t)~7)|_BV(ADPS2)|_BV(ADPS0);
  powerTarget=map(readPot(),0,1023,20,70);
  ADCSRA=old;
  if(restore)comparatorOn();
 }
 if(now-slewAtMs>=20){
  slewAtMs=now;
  uint8_t target=(uint16_t)powerTarget*255/100;
  if(pwmDuty<target)pwmDuty++;
  else if(pwmDuty>target)pwmDuty--;
  // Update the active lower output; never touch the upper switch here.
  for(uint8_t p=1;p<6;p+=2)if(steps[sector][p])analogWrite(pins[p],pwmDuty);
 }
}
void advanceSector(){
 saveEvent();sector=(sector+(reversed?5:1))%6;applySector();powerService();
}
// Read ACO between two timer snapshots. Unlike v1.2, no micros()/filter work
// between identifying an ON plateau and actually reading ACO. Check physical pin.
// Timer1 is 8-bit phase-correct; TCNT1L is sufficient and avoids 16-bit access.
inline bool comparatorSample(uint8_t &level){
 uint8_t bound=pwmDuty>PWM_GUARD?pwmDuty-PWM_GUARD:0;
 uint8_t before=lowTimer2[sector]?TCNT2:TCNT1L;
 uint8_t value=ACSR;
 uint8_t port=PINB;
 uint8_t after=lowTimer2[sector]?TCNT2:TCNT1L;
 if(before>=bound||after>=bound||!(port & lowMask[sector]))return false;
 level=(value & _BV(ACO))?1:0;return true;
}
uint8_t expectedAfter(){
 // Preserves user's v1.2 polarity convention; validate in f1, then e0/e1.
 return ((sector&1)?0:1) ^ (reversed?1:0) ^ invertZc;
}
uint32_t blankTime(){
 uint32_t b=periodUs/12;
 if(b<200)b=200;
 if(b>1300)b=1300;
 return b;
}
void observe(){
 if(!compEnabled||eventSeen)return;
 uint32_t now=micros(),age=now-stepAt;
 if(age<blankTime()||age>periodUs*4/5)return;
 uint8_t level;if(!comparatorSample(level))return;
 now=micros();age=now-stepAt;
 if(!candidateValid||candidate!=level){
  candidateValid=true;candidate=level;candidateAt=now;samples=1;return;
 }
 if(samples<255)samples++;
 if(samples<3||now-candidateAt<CONFIRM_US)return;
 uint8_t after=expectedAfter();
 if(!baseReady){
  if(age>periodUs*35/100){sectorError=5;return;}
  if(level==after){sectorError=3;return;}
  baseReady=true;return;
 }
 if(level!=after)return;
 uint32_t at=candidateAt,zcAge=at-stepAt;
 if(zcAge<periodUs/5||zcAge>periodUs*4/5){sectorError=4;return;}
 // Period from consecutive ZC timestamps, independent of previous commutation age.
 if(havePreviousZc){
  uint32_t measured=at-previousZcAt;
  uint32_t reference=(state==TRACK)?estimateUs:periodUs;
  if(measured<reference*65/100||measured>reference*135/100){
   sectorError=6;
   if(state==TRACK){saveEvent();stopMotor(3);}
   return;
  }
  estimateUs=(estimateUs*3+measured)/4;
 }else estimateUs=periodUs;
 if(estimateUs<MIN_US||estimateUs>MAX_US){sectorError=6;if(state==TRACK){saveEvent();stopMotor(4);}return;}
 previousZcAt=at;havePreviousZc=true;lastZcAt=at;zcAt=at;eventSeen=true;
 if(state==TRACK){
  uint32_t target=at+estimateUs/2;
  // applySector needs DEAD_US before the new vector is actually energized.
  uint32_t commandAt=target-DEAD_US;
  if(reached(micros(),commandAt)){saveEvent();stopMotor(5);return;}
  due=commandAt;scheduled=true;
 }
}
void startMotor(){
 if(state!=STOP){Serial.println(F("STOP FIRST"));return;}
 if(hasRun&&millis()-powerOffAt<5000){Serial.println(F("WAIT 5s; VERIFY ROTOR STOPPED"));return;}
 allOff();comparatorOff();
 if(fromPot){powerTarget=map(readPot(),0,1023,20,70);}
 dutyPercent(powerTarget);reversed=reverseNext;hasRun=true;
 eventLen=eventHead=coastLen=0;validZc=badZc=0;streak=0;lastFault=0;
 havePreviousZc=false;periodUs=START_US;estimateUs=(uint32_t)targetStepMs*1000;
 sector=0;applySector();state=ALIGN;stateAtMs=millis();
 Serial.println(F("ALIGN"));
}
void dumpEvents(){
 if(state!=STOP){Serial.println(F("STOP FIRST"));return;}
 Serial.println(F("sector,flag,zc_age_us,estimated_sector_us"));
 uint8_t index=(eventHead+EVENTS-eventLen)%EVENTS;
 for(uint8_t k=0;k<eventLen;k++){
  Event &e=events[index];Serial.print(e.sector);Serial.print(',');
  Serial.print(e.flag);Serial.print(',');Serial.print(e.zc);Serial.print(',');Serial.println(e.period);
  index=(index+1)%EVENTS;
 }
 Serial.print(F("valid="));Serial.print(validZc);Serial.print(F(" bad="));Serial.println(badZc);
 Serial.println(F("flags:0=no_base 1=no_ZC 2=valid 3=polarity 4=timing 5=late_base 6=period"));
}
void dumpCoast(){
 if(state!=STOP){Serial.println(F("STOP FIRST"));return;}
 Serial.println(F("us,U,V,W,N,U-N,V-N,W-N"));
 for(uint8_t k=0;k<coastLen;k++){
  Coast &r=coast[k];Serial.print((uint32_t)r.time100us*100);Serial.print(',');
  Serial.print(r.u);Serial.print(',');Serial.print(r.v);Serial.print(',');
  Serial.print(r.w);Serial.print(',');Serial.print(r.n);Serial.print(',');
  Serial.print((int)r.u-r.n);Serial.print(',');Serial.print((int)r.v-r.n);Serial.print(',');Serial.println((int)r.w-r.n);
 }
}
void settings(){
 if(state!=STOP){Serial.println(F("STOP FIRST"));return;}
 Serial.println(F("BMLC v1.3: f0 timer / f1 shadow / f2 continuous feedback"));
 Serial.print(F("f="));Serial.print(feedback);Serial.print(F(" i_ms="));Serial.print(targetStepMs);
 Serial.print(F(" p="));Serial.print(powerTarget);Serial.print(F(" pot="));Serial.print(fromPot);
 Serial.print(F(" e="));Serial.print(invertZc);Serial.print(F(" reverse="));Serial.println(reverseNext);
 Serial.println(F("t s f0/f1/f2 i2..40 p20..70 p0 e0/e1 v g d ?"));
 Serial.println(F("faults:1=deadline 2=missing_ZC 3=ZC_period 4=speed_range 5=late_ZC 6=no_lock"));
}
void processCommand(char *s){
 if(!strcmp(s,"s")){stopMotor();return;}
 if(!strcmp(s,"t")){startMotor();return;}
 if(!strcmp(s,"g")){dumpEvents();return;}
 if(!strcmp(s,"d")){dumpCoast();return;}
 if(!strcmp(s,"?")){settings();return;}
 // Runtime power change: queued, slew limited; all other settings require STOP.
 if(s[0]=='p'){
  char *end;errno=0;long value=strtol(s+1,&end,10);
  if(errno||end==s+1||*end|| (value!=0&&(value<20||value>70))){Serial.println(F("POWER 0 / 20..70"));return;}
  fromPot=(value==0);if(value)powerTarget=value;
  if(state==STOP)settings();
  return;
 }
 if(state!=STOP){Serial.println(F("STOP FIRST"));return;}
 if(!strcmp(s,"v")){reverseNext=!reverseNext;settings();return;}
 if(s[0]=='f'&&s[1]>='0'&&s[1]<='2'&&!s[2]){feedback=s[1]-'0';settings();return;}
 if(s[0]=='e'&&(s[1]=='0'||s[1]=='1')&&!s[2]){invertZc=s[1]-'0';settings();return;}
 if(s[0]=='i'){
  char *end;errno=0;long v=strtol(s+1,&end,10);
  if(!errno&&end!=s+1&&!*end&&v>=2&&v<=40){targetStepMs=v;settings();return;}
 }
 Serial.println(F("BAD COMMAND"));
}
void serialService(){
 // Bounded processing: serial traffic must not monopolize the commutation loop.
 for(uint8_t k=0;k<4&&Serial.available();k++){
  char c=Serial.read();
  if(c=='\r'||c=='\n'){
   if(!discardLine&&commandLen){command[commandLen]=0;processCommand(command);}
   commandLen=0;discardLine=false;
  }else if(!discardLine){
   if(commandLen<sizeof(command)-1)command[commandLen++]=c;
   else{commandLen=0;discardLine=true;}
  }
 }
}
void buttonService(){
 bool raw=digitalRead(REV);uint32_t now=millis();
 if(raw!=buttonRaw){buttonRaw=raw;buttonAt=now;}
 if(raw!=buttonStable&&now-buttonAt>=30){
  buttonStable=raw;
  if(raw==LOW){reverseNext=!reverseNext;if(state!=STOP)stopMotor();else settings();}
 }
}
void setup(){
 for(uint8_t p=0;p<6;p++){digitalWrite(pins[p],LOW);pinMode(pins[p],OUTPUT);}
 pinMode(REV,INPUT_PULLUP);pinMode(POT,INPUT);
 digitalWrite(6,LOW);pinMode(6,INPUT);DIDR1|=_BV(AIN0D);
 allOff();pwmInit();comparatorOff();analogReference(DEFAULT);
 Serial.begin(115200);settings();
}
void loop(){
 // Service an already scheduled TRACK transition BEFORE serial processing.
 uint32_t now=micros();
 if(state==TRACK&&scheduled&&reached(now,due)){
  if(now-due>periodUs/5){stopMotor(1);return;}
  periodUs=estimateUs;advanceSector();return;
 }
 serialService();buttonService();now=micros();uint32_t ms=millis();
 if(state==STOP)return;
 if(state==COAST){
  if(coastLen==COAST_COUNT){state=STOP;Serial.println(F("CAPTURE COMPLETE: g / d"));return;}
  if(reached(now,coastDue)){
   Coast &r=coast[coastLen++];r.time100us=(now-coastAt)/100;
   analogRead(A3);r.u=analogRead(A3);analogRead(A2);r.v=analogRead(A2);
   analogRead(A1);r.w=analogRead(A1);analogRead(A0);r.n=analogRead(A0);
   coastDue+=6000;
  }return;
 }
 if(state==ALIGN){
  if(ms-stateAtMs>=ALIGN_MS){
   state=RAMP;stateAtMs=ms;advanceSector();due=stepAt+periodUs-DEAD_US;
   Serial.println(F("RAMP"));
  }return;
 }
 if(state==RAMP){
  if(ms-stateAtMs>=RAMP_MS){
   state=SHADOW;stateAtMs=ms;periodUs=(uint32_t)targetStepMs*1000;
   estimateUs=periodUs;streak=0;havePreviousZc=false;
   if(feedback)comparatorOn();
   // Keep actual stepAt/due: comparator activation does not create a new sector.
   resetFilter();Serial.println(F("SHADOW"));
  }else{
   if(reached(now,due)){
    if(now-due>periodUs/5){stopMotor(1);return;}
    periodUs=rampPeriod(ms-stateAtMs);advanceSector();due=stepAt+periodUs-DEAD_US;
   }return;
  }
 }
 if(state==SHADOW){
  if(ms-stateAtMs>=SHADOW_MS){saveEvent();if(feedback==2){stopMotor(6);}else coastStart();return;}
  observe();if(state!=SHADOW)return;
  if(reached(micros(),due)){
   if(micros()-due>periodUs/5){stopMotor(1);return;}
   advanceSector();due=stepAt+periodUs-DEAD_US;
   if(feedback==2&&streak>=LOCK_REQUIRED){
    state=TRACK;periodUs=estimateUs;scheduled=false;
    Serial.println(F("LOCK: TRACK"));
   }
  }return;
 }
 if(state==TRACK){
  observe();if(state!=TRACK)return;
  // No blind fallback to forced stepping after loss of feedback.
  if(!scheduled&&micros()-stepAt>periodUs*3/2){saveEvent();stopMotor(2);return;}
 }
}
