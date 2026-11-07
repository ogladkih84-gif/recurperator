#include <cassert>
#include <algorithm>
#include <iostream>
#include "../firmware/NanoBldcClean/NanoBldcClean.ino"
void send(char c){Serial.rx=std::string(1,c);controls();}
int main(){
 input[BUTTON]=HIGH;adcInput[HANDLE]=0;setup();
 direction=-1;invert=true;startupPwm=84;rampFactor=SLOW_RAMP;
 goodTotal=17;errorCode=8;angleCount=3;zeroSeen=false;
 Serial.log.clear();send('p');
 assert(mode==IDLE&&slowPwm&&startupPwm==60&&direction==1&&!invert&&rampFactor==FAST_RAMP);
 assert(goodTotal==17&&errorCode==8&&angleCount==3&&!zeroSeen);
 assert(std::count(Serial.log.begin(),Serial.log.end(),'\n')==1);
 assert(Serial.log.find("ШИМ=60/255 @3921Гц")!=std::string::npos);
 for(int i=0;i<3;i++)assert(!output[TOP[i]]&&!output[BOTTOM[i]]);
 Serial.log.clear();send('p');send('q');assert(std::count(Serial.log.begin(),Serial.log.end(),'\n')==2);
 for(Mode active:{POSITION,ACCELERATE,ACQUIRE,MEASURE,WAIT_VECTOR,FIRST_VECTOR,VERIFY,FOLLOW,COAST}){
  mode=active;startupPwm=72;direction=-1;invert=true;rampFactor=SLOW_RAMP;
  send('p');assert(mode==active&&startupPwm==72&&direction==-1&&invert&&rampFactor==SLOW_RAMP);
 }
 mode=IDLE;Serial.log.clear();send('?');assert(Serial.log.find("U=A3 V=A2 W=A1")!=std::string::npos);
 std::cout<<"PASS: repeatable preset, one-line report, no automatic start, preserves diagnostics and zero interlock, rejects all active/coast modes\n";
}
