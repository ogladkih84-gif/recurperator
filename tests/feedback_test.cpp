#include <cassert>
#include <iostream>
#include "../firmware/BMLC175A_v13_feedback/BMLC175A_v13_feedback.ino"
void commandText(const char*s){char b[100];std::strcpy(b,s);processCommand(b);}
void checkBridge(){
 for(int p=0;p<6;p+=2)assert(!(output[pins[p]]&&output[pins[p+1]]));
 if(state==STOP||state==COAST)for(auto p:pins)assert(output[p]==0);
}
void level(uint8_t value){ACSR=(ACSR & ~_BV(ACO))|(value?_BV(ACO):0);}
void runFor(uint64_t duration,bool crossings=true){
 auto end=clockUs+duration;
 while(clockUs<end){
  uint8_t sign=expectedAfter()^1;
  if(crossings && uint32_t(micros()-stepAt)>=periodUs/2)sign=expectedAfter();
  level(sign);TCNT1L=TCNT2=0;loop();checkBridge();clockUs+=20;
 }
}
void atLevel(uint32_t t,uint8_t v){clockUs=t;level(v);observe();}
int main(){
 input[REV]=HIGH;adcInput[POT]=512;setup();checkBridge();assert(state==STOP&&feedback==1);
 for(auto &row:steps){int h=0,l=0;for(int p=0;p<6;p+=2){h+=row[p];l+=row[p+1];assert(!(row[p]&&row[p+1]));}assert(h==1&&l==1);}
 commandText("p999");assert(powerTarget==28);
 commandText("p99999999999999999999999999");assert(powerTarget==28);
 commandText("i1");assert(targetStepMs==12);
 commandText("f2");commandText("t");assert(state==ALIGN);
 runFor(ALIGN_MS*1000+RAMP_MS*1000+600000);
 assert(state==TRACK&&validZc>=12); // acquisition through real filter / state machine
 assert(compEnabled && !(ADCSRA&_BV(ADEN)) && (ADCSRB&_BV(ACME)));
 runFor(2000000);assert(state==TRACK); // no 1.5s tracking cutoff
 commandText("p35");assert(powerTarget==35);
 runFor(50000);assert(state==TRACK&&pwmDuty>71);
 runFor(100000,false);assert(state==STOP&&lastFault==2);checkBridge();
 commandText("t");assert(state==STOP); // restart coast dwell
 runFor(5100000);commandText("v");commandText("t");assert(reversed&&state==ALIGN);
 commandText("s");assert(state==STOP);checkBridge();
 // Phase-error test: crossing is early (age 3000), but ZC-to-ZC period is 10000.
 // Next vector must be at ZC+5000, NOT at ZC+age.
 sector=0;state=TRACK;periodUs=estimateUs=10000;stepAt=107000;
 previousZcAt=100000;havePreviousZc=true;reversed=false;invertZc=0;
 compEnabled=true;applySector(); // then place the model at the desired timeline
 stepAt=107000;periodUs=estimateUs=10000;previousZcAt=100000;havePreviousZc=true;
 TCNT1L=TCNT2=0;
 uint8_t after=expectedAfter();
 atLevel(108000,after^1);atLevel(108050,after^1);atLevel(108100,after^1);
 assert(baseReady);
 // A short opposite-sign glitch must not become a zero crossing.
 atLevel(108500,after);atLevel(108520,after);atLevel(108540,after);
 atLevel(108560,after^1);atLevel(108660,after^1);atLevel(108760,after^1);
 assert(!eventSeen);
 atLevel(110000,after);atLevel(110050,after);atLevel(110100,after);
 assert(eventSeen&&scheduled&&zcAt==110000&&estimateUs==10000&&due==114900);
 clockUs=114900;loop();assert(state==TRACK&&sector==1&&stepAt>=115000);checkBridge();
 // Reject readings near PWM edges and when the selected lower output is not on.
 uint8_t sample=99;TCNT1L=TCNT2=pwmDuty;assert(!comparatorSample(sample));
 TCNT1L=TCNT2=0;PINB=0;assert(!comparatorSample(sample));
 assert(reached(20,0xfffffff0U));assert(!reached(0xfffffff0U,20));
 // Reversed tracking uses reversed polarity and sector order.
 allOff();state=TRACK;reversed=true;sector=0;periodUs=estimateUs=10000;
 compEnabled=true;applySector();scheduled=true;due=micros()+1000;
 clockUs=due;loop();assert(sector==5);checkBridge();
 stopMotor();
 // Exercise actual tracking across the 32-bit micros() wrap, not only a helper.
 clockUs=0xffffff00ULL;state=TRACK;sector=0;reversed=false;compEnabled=true;
 periodUs=estimateUs=10000;applySector();previousZcAt=stepAt-5000;
 havePreviousZc=true;runFor(40000);assert(state==TRACK);checkBridge();
 stopMotor();
 // Failed acquisition must not continue indefinitely in forced mode.
 hasRun=false;commandText("f2");commandText("t");
 runFor((ALIGN_MS+RAMP_MS+SHADOW_MS+100)*1000ULL,false);
 assert(state==STOP&&lastFault==6);checkBridge();
 std::cout<<"PASS: acquisition, continuous tracking, power slew, missing-ZC stop, restart dwell, period-based 30-degree scheduling, PWM gating, reverse order, wraparound, no-lock abort\n";
}
