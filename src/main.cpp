#include "config.h"

HardwareSerial mySerial(USART1);

eXoCAN can;
const int txMsgID = 0x069;  
uint8_t txData[8] = {0x00, 0x01, 0x23, 0x45, 0xAB, 0xCD, 0xEF, 0xFF};
const uint32_t txDly = 2000;  // ส่งข้อมูลทุก 2 วินาที
uint32_t lastSendTime = 0;

void setup() {
  mySerial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  can.begin(STD_ID_LEN, BR250K, PORTA_11_12_XCVR);
  mySerial.println("✅ CAN Bus Initialized");

  // ตั้งค่าให้รับเฉพาะ ID ที่ต้องการ
  can.filterMask16Init(0, 0x070, 0x7FF, 0, 0);
}

void loop() {
  if (millis() - lastSendTime >= txDly) {
      lastSendTime = millis();

      mySerial.print("📤 Sending CAN ID: 0x");
      mySerial.println(txMsgID, HEX);

      bool sent = can.transmit(txMsgID, txData, sizeof(txData));

      if (sent) {
          mySerial.println("✅ CAN Message Sent!");
      } else {
          mySerial.println("❌ Failed to Send CAN Message!");
      }
  }
}