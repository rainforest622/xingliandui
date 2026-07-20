#include "asr.h"
extern "C"{ void * __dso_handle = 0 ;}
#include "setup.h"
#include "myLib/asr_event.h"

uint32_t snid;
void Serial_available();

//{speak:妞妞-中文女声,vol:10,speed:10,platform:haohaodada,version:V3}
//{playid:10001,voice:智能巡检语音助手已启动，请说小星小星。}
//{playid:10002,voice:语音助手已休眠，请说小星小星。}

void Serial_available(){
  while (1) {
    if(Serial.available() > 0){
      String _ss = Serial.readString();
      if(((uint8_t)_ss[0] == 0xF1)){
        delay(200);
        enter_wakeup(5000);
        delay(200);
        //{playid:10500,voice:收到，开始执行巡检路线。}
        play_audio(10500);
      }
      if(((uint8_t)_ss[0] == 0xF2)){
        delay(200);
        enter_wakeup(5000);
        delay(200);
        //{playid:10501,voice:巡检已暂停，等待新的指令。}
        play_audio(10501);
      }
      if(((uint8_t)_ss[0] == 0xF3)){
        delay(200);
        enter_wakeup(5000);
        delay(200);
        //{playid:10502,voice:巡检已停止，已切换为手动待命。}
        play_audio(10502);
      }
      if(((uint8_t)_ss[0] == 0xF4)){
        delay(200);
        enter_wakeup(5000);
        delay(200);
        //{playid:10503,voice:系统状态已同步到手机端。}
        play_audio(10503);
      }
      if(((uint8_t)_ss[0] == 0xF5)){
        delay(200);
        enter_wakeup(5000);
        delay(200);
        //{playid:10504,voice:当前操作需要在星闪端确认。}
        play_audio(10504);
      }
      if(((uint8_t)_ss[0] == 0xF6)){
        delay(200);
        enter_wakeup(5000);
        delay(200);
        //{playid:10505,voice:检测到紧急事件，巡检已停止，请人工接管。}
        play_audio(10505);
      }
    }
    delay(1);
  }
  vTaskDelete(NULL);
}

void ASR_CODE()
{
  //{ID:10506,keyword:"唤醒词",ASR:"小星小星",ASRTO:"我在，请说指令。"}
  if(snid == 10506){
    Serial.write(0xA0);
  }
  //{ID:10507,keyword:"唤醒词",ASR:"巡检小车",ASRTO:"我在，请说指令。"}
  if(snid == 10507){
    Serial.write(0xA0);
  }
  //{ID:10508,keyword:"命令词",ASR:"开始巡检",ASRTO:"正在开始巡检。"}
  if(snid == 10508){
    Serial.write(0xA1);
  }
  //{ID:10509,keyword:"命令词",ASR:"暂停巡检",ASRTO:"正在暂停巡检。"}
  if(snid == 10509){
    Serial.write(0xA2);
  }
  //{ID:10510,keyword:"命令词",ASR:"继续巡检",ASRTO:"正在继续巡检。"}
  if(snid == 10510){
    Serial.write(0xA3);
  }
  //{ID:10511,keyword:"命令词",ASR:"停止巡检",ASRTO:"已请求立即停车。"}
  if(snid == 10511){
    Serial.write(0xA4);
  }
  //{ID:10512,keyword:"命令词",ASR:"报告状态",ASRTO:"正在获取系统状态。"}
  if(snid == 10512){
    Serial.write(0xA5);
  }
  //{ID:10513,keyword:"命令词",ASR:"救命",ASRTO:"已收到求助，正在上报。"}
  if(snid == 10513){
    Serial.write(0xA6);
  }
  //{ID:10514,keyword:"命令词",ASR:"着火了",ASRTO:"已收到求助，正在上报。"}
  if(snid == 10514){
    Serial.write(0xA6);
  }
  //{ID:10515,keyword:"命令词",ASR:"解除报警",ASRTO:"正在请求解除报警。"}
  if(snid == 10515){
    Serial.write(0xA7);
  }
  set_state_enter_wakeup(15000);
}

void hardware_init(){
  vol_set(7);
  xTaskCreate(Serial_available,"Serial_available",512,NULL,4,NULL);
  vTaskDelete(NULL);
}

void setup()
{
  set_gpio_input(29);
  setPinFun(13,SECOND_FUNCTION);
  setPinFun(14,SECOND_FUNCTION);
  Serial.begin(9600);
  Serial.setTimeout(10);
}
