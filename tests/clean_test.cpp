#include <cassert>
#include <iostream>
#include "../firmware/NanoBldcClean/NanoBldcClean.ino"
void safe(){
 for(int i=0;i<3;i++)assert(!(output[TOP[i]]&&output[BOTTOM[i]]));
 if(mode==IDLE||mode==COAST)for(int i=0;i<3;i++)assert(!output[TOP[i]]&&!output[BOTTOM[i]]);
}
void simulate(uint64_t length,bool signal=true){
 auto end=clockUs+length;
 while(clockUs<end){
  uint8_t after=((vectorIndex&1)?0:1)^(direction<0)^invert;
  bool late=uint32_t(micros()-startedUs)>=sectorUs/2;
  uint8_t sign=(signal&&late)?after:after^1;
  ACSR=(ACSR&~_BV(ACO))|(sign?_BV(ACO):0);
  TCNT1L=TCNT2=0;loop();safe();clockUs+=20;
 }
}
int main(){
 input[BUTTON]=HIGH;adcInput[HANDLE]=1000;setup();start();assert(mode==IDLE&&!zeroSeen);
 adcInput[HANDLE]=0;readHandle();assert(zeroSeen);
 adcInput[HANDLE]=500;start();assert(mode==POSITION);
 simulate(7000000);assert(mode==FOLLOW&&confidence>=CLEAN_SECTORS&&goodTotal>10);
 assert(comparatorActive && !(ADCSRA&_BV(ADEN)));
 simulate(1000000);assert(mode==FOLLOW&&pwm>INITIAL_PWM);
 uint16_t good=goodTotal;simulate(200000);assert(mode==FOLLOW&&goodTotal>good);
 simulate(200000,false);assert(mode==COAST&&errorCode==2);safe();
 adcInput[HANDLE]=0;simulate(COAST_MS*1000ULL+10000);assert(mode==IDLE&&zeroSeen);
 adcInput[HANDLE]=600;start();assert(mode==POSITION);
 reverseRequest();assert(mode==COAST&&direction==-1);safe();
 adcInput[HANDLE]=0;simulate(COAST_MS*1000ULL+10000);
 adcInput[HANDLE]=500;start();simulate(7000000);assert(mode==FOLLOW&&direction==-1);
 adcInput[HANDLE]=0;simulate(100000);assert(mode==COAST);safe();
 simulate(COAST_MS*1000ULL+10000);adcInput[HANDLE]=500;start();simulate(8000000,false);
 assert(mode==COAST&&errorCode==6);safe();
 assert(dueNow(20,0xfffffff0U)&&!dueNow(0xfffffff0U,20));
 history[0]=7000;history[1]=5000;history[2]=6000;assert(medianPeriod()==6000);
 std::cout<<"PASS: zero-before-start, full acquisition, continuous feedback, live power, lost signal, coast, reverse, zero stop, failed lock, median, wrap comparison\n";
}
