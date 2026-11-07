#include <cassert>
#include <iostream>
#include "../firmware/NanoBldcClean/NanoBldcClean.ino"
void prepare(){
 mode=ACQUIRE;vectorIndex=0;sectorUs=FINAL_SECTOR_US;handleValue=500;
 handleMs=millis();previousCrossKnown=false;sensingMode();energize();
 TCNT1L=TCNT2=0;clockUs=startedUs+1000;
}
void readLevel(uint8_t sign,int count){
 for(int i=0;i<count;i++){
  ACSR=(ACSR&~_BV(ACO))|(sign?_BV(ACO):0);
  observeRotor();clockUs+=100;
 }
}
int main(){
 input[BUTTON]=HIGH;adcInput[HANDLE]=500;setup();
 prepare();readLevel(0,20);assert(initialSign);
 clockUs=startedUs+10000;readLevel(1,5);assert(crossingFound);saveSense();
 auto r=senseLog[(senseHead+SENSE_ROWS-1)%SENSE_ROWS];
 assert(r.samples>0&&r.ones>0&&r.ones<r.samples&&r.firstEdge>=10000&&(r.flags&3)==3);
 auto old=senseCount;saveSense();assert(senseCount==old);
 prepare();readLevel(1,30);assert(!initialSign&&!crossingFound);saveSense();
 r=senseLog[(senseHead+SENSE_ROWS-1)%SENSE_ROWS];assert(r.ones==r.samples&&r.samples>0&&(r.flags&4));
 prepare();TCNT1L=TCNT2=pwm;readLevel(0,30);saveSense();
 r=senseLog[(senseHead+SENSE_ROWS-1)%SENSE_ROWS];assert(r.tries>0&&r.samples==0);
 prepare();ADCSRA|=_BV(ADEN);readLevel(0,5);saveSense();
 r=senseLog[(senseHead+SENSE_ROWS-1)%SENSE_ROWS];assert(r.flags&32);
 for(int i=0;i<30;i++){prepare();readLevel(0,5);saveSense();}assert(senseCount==SENSE_ROWS);
 halt(6);auto saved=senseCount;halt(0);assert(senseCount==saved);
 mode=IDLE;Serial.rx="g";loop();assert(Serial.log.find("ожидаемый_знак,попытки,выборки")!=std::string::npos);
 std::cout<<"PASS: valid ZC statistics, constant comparator, missing PWM samples, wrong ADC mode, ring wrap, single sector flush, g command\n";
}
