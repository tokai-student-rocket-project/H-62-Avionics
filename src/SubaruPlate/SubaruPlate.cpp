#include <STM32_CAN.h>
#include <Arduino.h>
#include <TaskManager.h>


static CAN_message_t CAN_TX_msg;
static CAN_message_t CAN_inMsg;

// STM32_CAN Can( CAN1, DEF, RX_SIZE_64, TX_SIZE_16 );
STM32_CAN Can( CAN1, DEF);

uint8_t Counter;

namespace can{
    void SendData();
    void readCanMessage();
}

namespace led{
  void init();
  void blink();
}

void can::SendData()  // Send can messages in 50Hz phase from timer interrupt.
{
    CAN_TX_msg.id = (0x1A5);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;
    CAN_TX_msg.buf[1] =  0x41;
    CAN_TX_msg.buf[2] =  0x11;
    CAN_TX_msg.buf[3] =  0x22;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  0x00;
    CAN_TX_msg.buf[6] =  0x00;
    CAN_TX_msg.buf[7] =  0x00;

    Can.write(CAN_TX_msg);
//   Serial2.print("Sent: ");
//   Serial2.println(Counter, HEX);
}


void can::readCanMessage()  // Read data from CAN bus and print out the messages to serial bus. Note that only message ID's that pass filters are read.
{
  Serial2.flush();
  delay(100);
  Serial2.print(".");



    if (Can.read(CAN_inMsg) ) {
        Serial2.print("Channel:");
        Serial2.print(CAN_inMsg.bus);
        if (CAN_inMsg.flags.extended == false) {
          Serial2.print(" Standard ID:");
        }
        else {
          Serial2.print(" Extended ID:");
        }
        Serial2.print(CAN_inMsg.id, HEX);
        Serial2.print(" DLC: ");
        Serial2.print(CAN_inMsg.len);
        if (CAN_inMsg.flags.remote == false) {
           Serial2.print(" buf: ");
          for(int i=0; i<CAN_inMsg.len; i++) {
            Serial2.print("0x"); 
            Serial2.print(CAN_inMsg.buf[i], HEX); 
            if (i != (CAN_inMsg.len-1))  Serial2.print(" ");
          }
          Serial2.println();
        } else {
           Serial2.println(" Data: REMOTE REQUEST FRAME");
        }
      }
}


void setup(){
  Can.begin();
  Can.setBaudRate(250000);
  Serial2.begin(115200);

  led::init();

  Tasks.add(&can::SendData)->startFps(10);
  Tasks.add(&can::readCanMessage)->startFps(50);
  Tasks.add(&led::blink)->startFps(200);
}


void loop() {
    Tasks.update();
}

void led::init()
{
  pinMode(LED_BUILTIN, OUTPUT);
}

void led::blink(){
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
