#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
using uint8_t=std::uint8_t; using uint16_t=std::uint16_t; using uint32_t=std::uint32_t;
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define DEFAULT 1
#define F(x) x
#define _BV(x) (1<<(x))
enum { WGM10=0, CS10=0, WGM20=0, CS20=0, CS11=1, CS21=1 };
inline uint8_t TCCR1A,TCCR1B,TCCR2A,TCCR2B;
inline uint16_t OCR1A,OCR1B,OCR2A;
inline uint8_t ACSR,ADCSRA,ADCSRB,ADMUX,DIDR1,PINB;
struct ModelTimer {
 uint8_t value=0,step=0;unsigned reads=0;
 ModelTimer& operator=(uint8_t v){value=v;return *this;}
 operator uint8_t(){reads++;uint8_t previous=value;value+=step;return previous;}
};
inline ModelTimer TCNT2,TCNT1L;
#define ACD 7
#define ACI 4
#define ACIE 3
#define ACO 5
#define ADEN 7
#define ACME 6
#define REFS0 6
#define ADPS2 2
#define ADPS0 0
#define AIN0D 0
#define PB1 1
#define PB2 2
#define PB3 3
inline uint64_t clockUs=0;
inline int output[32]={}, input[32]={}, adcInput[32]={};
inline uint32_t millis(){return clockUs/1000;}
inline uint32_t micros(){return clockUs;}
inline void delayMicroseconds(unsigned x){clockUs+=x;}
inline void pinMode(int,int){}
inline void digitalWrite(int p,int v){output[p]=v;}
inline void analogWrite(int p,int v){
 output[p]=v;
 int bit=p==9?1:(p==10?2:(p==11?3:-1));
 if(bit>=0){if(v)PINB|=1<<bit;else PINB&=~(1<<bit);}
}
inline int digitalRead(int p){return input[p];}
inline int (*adcHook)(int)=nullptr;
inline int analogRead(int p){clockUs+=adcHook?26:104;return adcHook?adcHook(p):adcInput[p];}
inline void analogReference(int){}
inline long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}
struct SerialMock {
  std::string log,rx;
  int availableForWrite(){return 64;}
  void begin(int){} int available(){return rx.size();} char read(){char c=rx.front();rx.erase(0,1);return c;}
  void print(const char *v){log+=v;}
  void print(char v){log+=v;}
  template<class T> void print(T v){log+=std::to_string(v);}
  void println(){log+='\n';}
  template<class T> void println(T v){print(v);println();}
};
inline SerialMock Serial;
