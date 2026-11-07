#pragma once
#include <cassert>
#include <cmath>
#include <iostream>
#include "../firmware/NanoBldcClean/NanoBldcClean.ino"
// Независимый сигнал ротора: обратная связь не подстраивается под выбранный контроллером сектор.
double motorPeriod=20000,offsetAngle=.7,amplitude=22;
int adcBias=60;bool dynamicBias=false;
int motorSign=1,neutralError=0;
bool frozen=false,sendCrossings=true,angleNoise=false;
uint64_t origin=0;
double electricalAngle(uint64_t at){
 double noise=0;
 if(angleNoise&&mode==MEASURE&&uint32_t(micros()-quietStartedUs)>=QUIET_BLANK_US){
  unsigned index=(uint32_t(micros()-quietStartedUs)-QUIET_BLANK_US)/SNAPSHOT_GAP_US;
  if(index>0&&index<7)noise=(index%2?.6:-.6);
 }
 return offsetAngle+(frozen?0:motorSign*double(at-origin)*M_PI/(3*motorPeriod))+noise;
}
int adcMotor(int pin){
 if(pin==HANDLE)return adcInput[HANDLE];
 double theta=electricalAngle(clockUs),bias=adcBias;
 if(dynamicBias){double minimum=1;for(int i=0;i<3;i++)minimum=std::min(minimum,std::cos(theta-i*2*M_PI/3));bias=std::max(0.0,-amplitude*minimum-8);}
 if(pin==A0)return std::max(0,int(std::lround(bias))+neutralError);
 int ph=pin==A3?0:(pin==A2?1:(pin==A1?2:-1));
 return ph<0?0:std::min(1023,std::max(0,int(std::lround(bias+amplitude*std::cos(theta-ph*2*M_PI/3)))));
}
void safe(){
 for(int i=0;i<3;i++)assert(!(output[TOP[i]]&&output[BOTTOM[i]]));
 if(mode==MEASURE||mode==WAIT_VECTOR||mode==COAST||mode==IDLE)
  for(int i=0;i<3;i++)assert(!output[TOP[i]]&&!output[BOTTOM[i]]);
}
void tick(){
 int ph=DRIVE[vectorIndex][2];
 bool level=sendCrossings?std::cos(electricalAngle(clockUs)-ph*2*M_PI/3)<0:false;
 ACSR=(ACSR&~_BV(ACO))|(level?_BV(ACO):0);TCNT1L=TCNT2=0;
 loop();safe();clockUs+=40;
}
void run(uint64_t duration){uint64_t end=clockUs+duration;while(clockUs<end)tick();}
void waitFor(Mode target,uint64_t limit){auto end=clockUs+limit;while(mode!=target&&mode!=COAST&&clockUs<end)tick();assert(mode==target);}
void rearm(){adcInput[HANDLE]=0;run(6100000);assert(mode==IDLE);adcInput[HANDLE]=500;}
void send(char c){Serial.rx=std::string(1,c);tick();}
void startAt(double angle,int sign=1){offsetAngle=angle;origin=clockUs;motorSign=sign;direction=sign;send('t');assert(mode==POSITION);}
