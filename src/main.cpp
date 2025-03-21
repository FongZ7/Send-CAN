#include "config.h"

HardwareSerial mySerial(USART1);

eXoCAN can;
const int txMsgID = 0x069;  
uint8_t txData[8] = {0x00, 0x01, 0x23, 0x45, 0xAB, 0xCD, 0xEF, 0xFF};

const uint32_t txDly = 100;  // ส่งข้อมูลทุก 2 วินาที
uint32_t lastSendTime = 0;


// uint8_t values[8] = {12, 45, 78, 123, 200, 255, 5, 90}; 
int values[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t hexValue[8];
int emer = 0;
int start = 1;
int d_u = 128;
int l_r = 128;
int f_s = 128;

void setup() {
  mySerial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  can.begin(STD_ID_LEN, BR250K, PORTA_11_12_XCVR);
  mySerial.println("✅ CAN Bus Initialized");


  // ตั้งค่าให้รับเฉพาะ ID ที่ต้องการ
  can.filterMask16Init(0, 0x070, 0x7FF, 0, 0);

  // for (int i = 0; i < 8; i++) {
  //   hexValue[i] = (uint8_t)values[i]; // Convert and store
  // }
  
  // // Print the stored hex values
  // mySerial.print("Hex Values: ");
  // for (int i = 0; i < 8; i++) {
  //   mySerial.print("0x");
  //   mySerial.print(hexValue[i], HEX);
  //   mySerial.print(" ");
  // }
  // mySerial.print("tx Data: ");
  // for (int i = 0; i < 8; i++) {
  //   mySerial.print("0x");
  //   mySerial.print(txData[i], HEX);
  //   mySerial.print(" ");
  // }
}

void loop() {
  if (millis() - lastSendTime >= txDly) {
      lastSendTime = millis();

      values[1] = emer;
      values[2] = start;
      values[3] = d_u;
      values[4] = l_r;
      values[5] = f_s;

      convertToHex(hexValue, values, 8);

      mySerial.print("📤 Sending CAN ID: 0x");
      mySerial.println(txMsgID, HEX);

      bool sent = can.transmit(txMsgID, hexValue, sizeof(hexValue));

      if (sent) {
          mySerial.println("✅ CAN Message Sent!");
      } else {
          mySerial.println("❌ Failed to Send CAN Message!");
      }

      values[0]++;
  }
}

void convertToHex(uint8_t hexValue[], int values[], int size){
  for (int i = 0; i < size; i++) {
    hexValue[i] = (uint8_t)values[i]; // Convert and store
  }
}