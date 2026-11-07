#include <cassert>
#include <iostream>
#include "../firmware/NanoBldcClean/NanoBldcClean.ino"
void runCoast(bool decay){
 uint64_t origin=clockUs,end=origin+6010000;
 while(clockUs<end){
  adcInput[A0]=500;
  adcInput[A3]=900;adcInput[A1]=(decay&&clockUs-origin>2000000)?500:((clockUs-origin)%30000<15000?470:530);
  loop();
  for(int i=0;i<3;i++)assert(!output[TOP[i]]&&!output[BOTTOM[i]]);
  clockUs+=100;
 }
}
int main(){
 input[BUTTON]=HIGH;adcInput[HANDLE]=0;setup();mode=FOLLOW;halt(0);
 runCoast(false);assert(mode==IDLE&&coastCount==60);
 for(int i=10;i<50;i++){
  assert(coastLog[i].period100us>=280&&coastLog[i].period100us<=320);
  unsigned rpm=600000UL/(coastLog[i].period100us*ROTOR_PAIRS);
  assert(rpm>=930&&rpm<=1080);assert(coastLog[i].peak==30);
 }
 Serial.rx="d";loop();assert(Serial.log.find("электрическая_Гц,об_мин")!=std::string::npos);
 mode=FOLLOW;halt(6);uint32_t origin=coastOrigin;halt(0);
 assert(coastOrigin==origin&&errorCode==6); // repeated stop does not erase the measurement
 runCoast(true);assert(mode==IDLE&&coastCount==60);
 assert(coastLog[10].period100us>0);
 for(int i=35;i<60;i++)assert(coastLog[i].period100us==0);
 Serial.rx="d";loop();assert(Serial.log.find("нет,нет")!=std::string::npos);
 std::cout<<"PASS: 60 coast records, measured 33.3 Hz / about 1000 RPM, bridge stays off, stale signal is unavailable, repeated stop preserves log\n";
}
