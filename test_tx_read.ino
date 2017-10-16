#define NUM_RC_CHANNELS 6 //You need to specify how many pins you want to use
#include "PinChangeInt.h"  //If you need pinchangeint you need to include this header
const uint8_t RC_Channel_Pin[NUM_RC_CHANNELS]={8,9,10,11,12,13,};
uint16_t RC_Channel_Value[NUM_RC_CHANNELS];
#include "RCLib.h"

void plot(int Data1, int Data2, int Data3, int Data4=0, int Data5=0, int Data6=0, int Data7=0, int Data8=0){
  Serial.print(Data1); 
  Serial.print(" ");
  Serial.print(Data2); 
  Serial.print(" ");
  Serial.print(Data3); 
  Serial.print(" ");
  Serial.print(Data4); 
  Serial.print(" ");
  Serial.print(Data5); 
  Serial.print(" ");
  Serial.println(Data6); 
}
void setup(){
  Serial.begin(115200);
  Serial.println(F("Rc serial oscilloscope demo"));
  SetRCInterrupts(); //This method will do all the config foe you.
                    //Note some problems will be reported on the serial monitor
  Serial.println(F("Interrupts Set; starting "));
}

void loop(){
  int flag;
  if(flag=getChannelsReceiveInfo()) {
    plot(
      RC_Channel_Value[0],
      RC_Channel_Value[1],
      RC_Channel_Value[2],
      RC_Channel_Value[3],
      RC_Channel_Value[4],
      RC_Channel_Value[5]
    );
  }
  delay(100);
}
