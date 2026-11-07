#include "handover_model.h"
int main(){
 input[BUTTON]=HIGH;adcInput[HANDLE]=0;adcHook=adcMotor;setup();adcInput[HANDLE]=500;send('t');
 assert(rampFactor==FAST_RAMP);run(5000000);assert(mode==ACQUIRE);
 auto fastStarted=searchStartedMs;assert(fastStarted>=4000&&fastStarted<5000);
 waitFor(FOLLOW,4000000);assert(angleCount==8&&confidence>=CLEAN_SECTORS);
 run(25000000);assert(mode==FOLLOW);send('s');rearm();
 send('l');assert(rampFactor==SLOW_RAMP);send('o');run(5000000);assert(mode==ACCELERATE);
 run(5000000);assert(mode==TEST_RUN&&angleCount==0);
 auto openStarted=testStartedMs;run(10000000);assert(mode==TEST_RUN);
 uint64_t cutoff=uint64_t(openStarted+OPEN_TIME_MS)*1000;
 run(cutoff-clockUs+10000);assert(mode==COAST&&errorCode==0);rearm();
 send('f');assert(rampFactor==FAST_RAMP);startAt(.8);waitFor(VERIFY,10000000);
 searchStartedMs=millis()-SEARCH_TIME_MS;run(1000);assert(mode==COAST&&errorCode==6);rearm();
 startAt(1.8);waitFor(WAIT_VECTOR,10000000);send('s');assert(mode==COAST);rearm();
 startAt(1.8);waitFor(MEASURE,10000000);clockUs+=QUIET_LIMIT_US+1000;tick();assert(mode==COAST&&errorCode==10);rearm();
 startAt(1.8);waitFor(WAIT_VECTOR,10000000);clockUs+=10000;tick();assert(mode==COAST&&errorCode==10);
 std::cout<<"PASS: fast/slow profiles, measured handover, 25s FOLLOW, unchanged 15s open-loop test, verification limit, stop during wait, stale sample/plan rejection\n";
}
