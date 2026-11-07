#include "handover_model.h"
int main(){
 input[BUTTON]=HIGH;adcInput[HANDLE]=0;adcHook=adcMotor;setup();adcInput[HANDLE]=500;
 // Несколько начальных углов, оба направления и переход через границу полного оборота.
 for(int sign:{1,-1})for(double angle:{.05,1.1,3.05,5.8}){
  startAt(angle,sign);waitFor(FOLLOW,10000000);
  assert(angleCount==8&&planValid&&confidence==CLEAN_SECTORS);
  assert(anglePeriodUs>18500&&anglePeriodUs<21500);
  assert(firstDurationUs>=1500&&fitAdcRms<=3);
  run(2000000);assert(mode==FOLLOW);
  send('s');assert(mode==COAST);rearm();
 }
 // Обработка отрицательных полуволн, обрезанных АЦП у земли, и меняющегося общего уровня.
 for(bool dynamic:{false,true})for(int sign:{1,-1})for(double angle:{.15,1.9,4.8}){
  adcBias=0;dynamicBias=dynamic;startAt(angle,sign);waitFor(FOLLOW,10000000);
  assert(clippedInputs&&planValid&&anglePeriodUs>=16000&&anglePeriodUs<=24000);
  send('s');rearm();
 }
 adcBias=60;dynamicBias=false;
 // Скорость снимков может отличаться от принудительной уставки; после передачи период уточняется по компаратору.
 for(double period:{18500.0,21300.0,24500.0}){
  motorPeriod=period;startAt(.8);waitFor(FOLLOW,10000000);
  assert(anglePeriodUs>=14000&&anglePeriodUs<=26000);run(300000);assert(mode==FOLLOW);
  send('s');rearm();
 }
 motorPeriod=20000;
 // Длительная работа не имеет таймерного отключения после успешной передачи.
 startAt(.4);waitFor(FOLLOW,10000000);run(25000000);assert(mode==FOLLOW);
 sendCrossings=false;run(100000);assert(mode==COAST&&errorCode==2);rearm();sendCrossings=true;
 // Слабая ЭДС, неверная нейтраль и обратное вращение запрещают подачу первого вектора.
 amplitude=0;startAt(.5);run(7500000);assert(mode==COAST&&errorCode==7&&!planValid);rearm();amplitude=22;
 neutralError=20;startAt(.5);run(7500000);assert(mode==COAST&&errorCode==11&&!planValid);rearm();neutralError=0;
 startAt(.5);motorSign=-1;run(7500000);assert(mode==COAST&&errorCode==9&&!planValid);rearm();
 frozen=true;startAt(.5);run(7500000);assert(mode==COAST&&(errorCode==8||errorCode==9)&&!planValid);rearm();frozen=false;
 angleNoise=true;startAt(.5);run(7500000);assert(mode==COAST&&errorCode==8&&!planValid);rearm();angleNoise=false;
 adcBias=1010;startAt(.5);run(7500000);assert(mode==COAST&&errorCode==12&&!planValid);rearm();adcBias=60;
 // Остановка пользователем и ноль ручки во время измерения не возобновляют мост.
 startAt(.7);waitFor(MEASURE,8000000);send('s');assert(mode==COAST&&!planValid);rearm();
 startAt(.7);waitFor(MEASURE,8000000);adcInput[HANDLE]=0;run(50000);assert(mode==COAST&&errorCode==0&&!planValid);rearm();
 // Переполнение micros() внутри серии измерений и рассчитанного ожидания.
 startAt(2.9);waitFor(MEASURE,8000000);
 clockUs=0xffffff00ULL;origin=clockUs;quietStartedUs=micros();snapshotDueUs=quietStartedUs+QUIET_BLANK_US;angleCount=0;
 waitFor(FOLLOW,1000000);assert(planValid);send('s');rearm();
 send('k');assert(Serial.log.find("снимок,мкс,U,V,W,N")!=std::string::npos);
 std::cout<<"PASS: measured-angle handover, independent physical BEMF, both directions, angle wrap, 25s FOLLOW, weak/missing/reversed/stationary signal, bad neutral, stop during quiet, zero throttle, micros wrap, k diagnostics\n";
}
