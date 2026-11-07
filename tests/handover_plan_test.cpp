#include <cassert>
#include <cmath>
#include <iostream>
#include "../firmware/NanoBldcClean/NanoBldcClean.ino"
int main(){
 input[BUTTON]=HIGH;adcInput[HANDLE]=500;setup();
 for(int sign:{1,-1})for(int degree=0;degree<360;degree+=5){
  direction=sign;clockUs+=100000;quietStartedUs=micros();mode=MEASURE;
  angleCount=8;firstDurationUs=0;planValid=false;clippedInputs=false;
  double phase0=degree*M_PI/180,omega=sign*M_PI/(3*20000.0);
  for(int j=0;j<8;j++){
   auto &r=angleLog[j];r.at=quietStartedUs+4000+j*4000;r.neutral=200;
   double theta=phase0+omega*j*4000;
   r.angle=theta;
   for(int ph=0;ph<3;ph++)r.phase[ph]=std::lround(200+80*std::cos(theta-ph*2*M_PI/3));
  }
  clockUs=quietStartedUs+33000;planFirstVector();assert(mode==WAIT_VECTOR&&planValid);
  assert(anglePeriodUs>=19000&&anglePeriodUs<=21000&&firstDurationUs>=1500&&firstDurationUs<=anglePeriodUs);
  double phase=phase0+omega*uint32_t(plannedPowerUs-angleLog[0].at);
  double e[3];for(int ph=0;ph<3;ph++)e[ph]=std::cos(phase-ph*2*M_PI/3);
  int hi=DRIVE[plannedVector][0],lo=DRIVE[plannedVector][1];
  for(int ph=0;ph<3;ph++){assert(e[hi]>=e[ph]-.05);assert(e[lo]<=e[ph]+.05);}
  double center=plannedVector*M_PI/3-M_PI/6;
  double atBoundary=phase0+omega*uint32_t(firstBoundaryUs-angleLog[0].at);
  assert(std::abs(wrapAngle(atBoundary-center-sign*M_PI/6))<M_PI/60);
  for(int ph=0;ph<3;ph++)assert(!output[TOP[ph]]&&!output[BOTTOM[ph]]);
 }
 std::cout<<"PASS: 144 initial electrical angles, both directions, correct max/min phase vector, 30-degree boundary timing, no power during planning\n";
}
