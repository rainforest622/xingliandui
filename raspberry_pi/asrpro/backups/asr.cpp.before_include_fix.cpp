#include "asr.h"
extern "C"{ void * __dso_handle = 0 ;}
#include "setup.h"

uint32_t snid;
void ASR_CODE();
void voice_reply_task();

//{speak:小鹿-甜美女声,vol:16,speed:8,platform:baidu}
//{playid:10001,voice:智能巡检语音助手已启动，请说小星小星。}
//{playid:10002,voice:语音助手已休眠，请说小星小星。}

/*语音识别成功时发送一字节控制事件。
*/
void ASR_CODE(){
  set_state_enter_wakeup(15000);
  switch (snid) {
    case 1000:
    case 1001: Serial.write(0xA0); break;
    case 1002: Serial.write(0xA1); break;
    case 1003: Serial.write(0xA2); break;
    case 1004: Serial.write(0xA3); break;
    case 1005: Serial.write(0xA4); break;
    case 1006: Serial.write(0xA5); break;
    case 1007:
    case 1008: Serial.write(0xA6); break;
    case 1009: Serial.write(0xA7); break;
  }
  //{playid:2000,voice:收到，开始执行巡检路线。}

  //{playid:2001,voice:巡检已暂停，等待新的指令。}

  //{playid:2002,voice:巡检已停止，已切换为手动待命。}

  //{playid:2003,voice:系统状态已同步到手机端。}

  //{ID:1000,keyword:"唤醒词",ASR:"小星小星",ASRTO:"我在，请说指令。"}

  //{playid:2004,voice:当前操作需要在星闪端确认。}

  //{ID:1001,keyword:"唤醒词",ASR:"巡检小车",ASRTO:"我在，请说指令。"}

  //{playid:2005,voice:检测到紧急事件，巡检已停止，请人工接管。}

  //{ID:1002,keyword:"命令词",ASR:"开始巡检",ASRTO:"已发送开始巡检请求。"}

  //{ID:1003,keyword:"命令词",ASR:"暂停巡检",ASRTO:"已发送暂停巡检请求。"}

  //{ID:1004,keyword:"命令词",ASR:"继续巡检",ASRTO:"已发送继续巡检请求。"}

  //{ID:1005,keyword:"命令词",ASR:"停止巡检",ASRTO:"已发送立即停车请求。"}

  //{ID:1006,keyword:"命令词",ASR:"报告状态",ASRTO:"正在请求系统状态。"}

  //{ID:1007,keyword:"命令词",ASR:"救命",ASRTO:"紧急事件已上报。"}

  //{ID:1008,keyword:"命令词",ASR:"着火了",ASRTO:"紧急事件已上报。"}

  //{ID:1009,keyword:"命令词",ASR:"解除报警",ASRTO:"解除报警需要星闪端确认。"}
}

void voice_reply_task(){
  while (1) {
    if (Serial.available() > 0) {
      switch ((uint8_t)Serial.read()) {
        case 0xF1: play_audio(2000); break;
        case 0xF2: play_audio(2001); break;
        case 0xF3: play_audio(2002); break;
        case 0xF4: play_audio(2003); break;
        case 0xF5: play_audio(2004); break;
        case 0xF6: play_audio(2005); break;
      }
    }
    delay(10);
  }
  vTaskDelete(NULL);
}

void hardware_init(){
  vol_set(7);
  setPinFun(13, SECOND_FUNCTION);
  setPinFun(14, SECOND_FUNCTION);
  Serial.begin(9600);
  xTaskCreate(voice_reply_task,"voice_reply_task",256,NULL,4,NULL);
  vTaskDelete(NULL);
}

void setup()
{

}
