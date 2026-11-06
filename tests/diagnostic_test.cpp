#include <cassert>
#include <iostream>
#include "../firmware/BMLC175A_v10_diag/BMLC175A_v10_diag.ino"
void command(const char *s){char buf[64];std::strcpy(buf,s);processCommand(buf);}
void checkBridge(){
  for(int p=0;p<3;p++) assert(!(output[pins[p*2]] && output[pins[p*2+1]]));
  if(motorState==STOPPED || motorState==CAPTURING)
    for(auto p:pins) assert(output[p]==0);
}
void runUntil(uint64_t target){
  while(clockUs<target){loop();checkBridge();clockUs+=100;}
}
int main(){
  input[REV]=HIGH; adcInput[POT]=512;
  adcInput[BEMF_U]=500;adcInput[BEMF_V]=510;adcInput[BEMF_W]=520;adcInput[BEMF_N]=505;
  setup();checkBridge();
  for(uint8_t s=0;s<6;s++) assert(validStep(s));
  assert(!validStep(6));
  command("i15");assert(targetStepMs==15);
  command("i9");assert(targetStepMs==15);
  command("i20garbage");assert(targetStepMs==15);
  command("i20");command("b");assert(motorState==ALIGNING);
  runUntil(stateStartMs*1000ULL+ALIGN_MS*1000ULL+1000);
  assert(motorState==RAMPING && currentStep==1);
  command("i10");assert(targetStepMs==20); // setting cannot change while running
  uint32_t rampStart=stateStartMs;
  runUntil(rampStart*1000ULL+RAMP_MS*1000ULL-500);
  assert(motorState==RAMPING);
  uint32_t oldDeadline=nextStepUs;
  runUntil(rampStart*1000ULL+RAMP_MS*1000ULL+500);
  assert(motorState==HOLDING);
  // Deadline is preserved, or advances by exactly one target sector if it was due.
  assert(nextStepUs==oldDeadline || nextStepUs==oldDeadline+20000);
  runUntil(stateStartMs*1000ULL+HOLD_MS*1000ULL+1000);
  assert(motorState==CAPTURING);checkBridge();
  runUntil(clockUs+800000);
  assert(motorState==STOPPED && coastCount==128);
  for(int i=1;i<coastCount;i++) assert(coastLog[i].time100us>coastLog[i-1].time100us);
  assert(coastLog[0].u==500 && coastLog[0].n==505);
  command("d");assert(Serial.log.find("us,U,V,W,N")!=std::string::npos);
  command("v");command("t");assert(motorState==STOPPED); // reverse dwell
  runUntil(powerOffAtMs*1000ULL+5001000);
  command("t");assert(motorState==ALIGNING && reverseDir);
  runUntil(stateStartMs*1000ULL+ALIGN_MS*1000ULL+1000);
  assert(currentStep==5); // reverse table order
  clockUs=nextStepUs+currentIntervalUs+1000;
  loop();assert(motorState==STOPPED);checkBridge();
  assert(Serial.log.find("DEADLINE MISSED")!=std::string::npos);
  command("m");assert(motorState==CAPTURING);
  command("t");assert(motorState==CAPTURING);
  command("s");assert(motorState==STOPPED);checkBridge();
  std::cout<<"PASS: table, commands, ramp/hold deadline, automatic capture, timestamps, reverse dwell/order, late deadline abort, capture stop\n";
}
