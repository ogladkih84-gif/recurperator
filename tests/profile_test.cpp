#include <cassert>
#include <iostream>
#include "../firmware/NanoBldcClean/NanoBldcClean.ino"
bool physicalKnown=false;uint32_t physicalEpoch=0;
void run(uint64_t duration,bool edges){
 auto end=clockUs+duration;
 while(clockUs<end){
  uint8_t after=((vectorIndex&1)?0:1)^(direction<0)^invert;
  if(mode==FOLLOW&&!physicalKnown){physicalEpoch=startedUs;physicalKnown=true;}
  if(mode!=FOLLOW)physicalKnown=false;
  bool crossed=mode==FOLLOW?uint32_t(micros()-physicalEpoch)%FINAL_SECTOR_US>=FINAL_SECTOR_US/2:uint32_t(micros()-startedUs)>=sectorUs/2;
  uint8_t value=edges&&crossed?after:after^1;
  ACSR=(ACSR&~_BV(ACO))|(value?_BV(ACO):0);TCNT1L=TCNT2=0;loop();
  for(int i=0;i<3;i++)assert(!(output[TOP[i]]&&output[BOTTOM[i]]));
  if(mode==COAST||mode==IDLE)for(int i=0;i<3;i++)assert(!output[TOP[i]]&&!output[BOTTOM[i]]);
  clockUs+=100;
 }
}
void send(const char*s){Serial.rx=s;while(Serial.available())loop();}
void rearm(){adcInput[HANDLE]=0;run(6100000,false);assert(mode==IDLE);adcInput[HANDLE]=500;}
int main(){
 input[BUTTON]=HIGH;adcInput[HANDLE]=0;setup();adcInput[HANDLE]=500;send("t");
 assert(rampFactor==FAST_RAMP);run(5000000,false);assert(mode==ACQUIRE);
 auto fastStarted=searchStartedMs;assert(fastStarted>=4000&&fastStarted<5000);
 run(7000000,false);assert(mode==ACQUIRE&&attempts>255); // no 8-bit overflow / old 90-sector cutoff
 uint64_t cutoff=uint64_t(searchStartedMs+SEARCH_TIME_MS)*1000;
 run(cutoff-clockUs-5000,false);assert(mode==ACQUIRE);
 run(10000,false);assert(mode==COAST&&errorCode==6);
 rearm();send("l");assert(rampFactor==SLOW_RAMP);send("o");
 run(5000000,false);assert(mode==ACCELERATE);
 run(5000000,false);assert(mode==TEST_RUN);
 auto openStarted=testStartedMs;
 run(10000000,false);assert(mode==TEST_RUN); // longer than the old 3s test
 cutoff=uint64_t(openStarted+OPEN_TIME_MS)*1000;
 run(cutoff-clockUs+10000,false);assert(mode==COAST&&errorCode==0);
 rearm();send("f");assert(rampFactor==FAST_RAMP);send("t");
 run(8000000,true);assert(mode==FOLLOW);
 run(25000000,true);assert(mode==FOLLOW); // real feedback has no 15s cutoff
 send("s");assert(mode==COAST&&errorCode==0);
 std::cout<<"PASS: fast/slow ramp profiles, 15s search deadline, >255 sectors, 15s open-loop test, unlimited FOLLOW, manual stop\n";
}
