#include <cassert>
#include <iostream>
#include "../firmware/NanoBldcClean/NanoBldcClean.ino"
void send(char c){Serial.rx=std::string(1,c);controls();}
int main(){
 input[BUTTON]=HIGH;adcInput[HANDLE]=0;setup();
 assert(!slowPwm&&(TCCR1B&7)==1&&(TCCR2B&7)==1);
 send('b');assert(slowPwm&&(TCCR1B&7)==2&&(TCCR2B&7)==2);
 for(int i=0;i<3;i++)assert(!output[TOP[i]]&&!output[BOTTOM[i]]);
 mode=FOLLOW;send('h');assert(slowPwm); // no frequency switching under power
 mode=IDLE;send('h');assert(!slowPwm&&(TCCR1B&7)==1&&(TCCR2B&7)==1);
 vectorIndex=0;pwm=60;analogWrite(BOTTOM[1],pwm);TCNT1L=pwm;TCNT1L.reads=0;
 uint8_t sign;assert(!sampleSign(sign)&&TCNT1L.reads==64);
 mode=IDLE;send('b');analogWrite(BOTTOM[1],pwm);TCNT1L=pwm;TCNT1L.reads=0;
 assert(!sampleSign(sign)&&TCNT1L.reads==384); // bounded even if no valid plateau
 TCNT1L=0;assert(sampleSign(sign));
 std::cout<<"PASS: default 31kHz, explicit 3.9kHz test, paired prescalers, bridge off, runtime switch prohibited, bounded sample wait\n";
}
