
#include <Arduino.h>
#include <HardwareSerial.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"  // Ensure this is included
#include "STM32_CAN.h"

#define JOYSTICK_CAN_ID 0xCFDD633     // ID ที่ต้องการอ่านข้อมูล
#define SW_JOYSTICK_CAN_ID 0x123      // ID ที่ใช้สั่งบันทึก/เล่นซ้ำ
#define INDICATOR_CAN_ID 0x121 
#define REMOTE_CAN_ID 0x180


#define LED_Status_Can_Remote PA2
#define LED_Status_Can_Joystick PA3
#define LED_Status_Can_Indicator PA4
#define LED_Status_Can_Error PB12

// พินสำหรับการควบคุมมอเตอร์
#define MOTOR1_PWM_PIN PB1              // พิน PWM สำหรับมอเตอร์ 1 (ซ้าย-ขวา)
#define MOTOR1_DIR_PIN PA15             // พิน DIR สำหรับมอเตอร์ 1

#define MOTOR2_PWM_PIN PB0              // พิน PWM สำหรับมอเตอร์ 2 (ขึ้น-ลง)
#define MOTOR2_DIR_PIN PB3              // พิน DIR สำหรับมอเตอร์ 2

#define MOTOR3_PWM_PIN PA7              // พิน PWM สำหรับมอเตอร์ที่ 3
#define MOTOR3_DIR_PIN PB4              // พิน DIR สำหรับมอเตอร์ที่ 3

#define MOTOR4_PWM_PIN PA6              // มอเตอร์เปิดปิดวาล์ว
#define MOTOR4_DIR_PIN PB5              // ทิศทางของมอเตอร์วาล์ว

#define MAX_INDEX_A 160  // ค่า index สูงสุดสำหรับซ้าย-ขวา
#define MAX_INDEX_B 120  // ค่า index สูงสุดสำหรับขึ้น-ลง

uint8_t indexA = 0;  // index สำหรับซ้าย-ขวา (0-32)
uint8_t indexB = 0;  // index สำหรับขึ้น-ลง (0-24)

std::vector<String> joystickSequence;   // เก็บลำดับการเคลื่อนที่
bool enableJoystick = true;  // ตั้งค่าเป็น true เพื่อให้จอยสติ๊กทำงาน
bool enableRemote = false;   // ตั้งค่าเป็น false เพื่อปิดการทำงานของรีโมท
bool recording = false;                 // กำลังบันทึก?
bool replaying = false;                 // กำลังเล่นซ้ำ?
bool lastButtonState = false;           // สถานะปุ่มก่อนหน้า
bool valveState = false;                // สถานะของวาล์ว (เปิด / ปิด)
bool motorActive = false;               // สถานะการทำงานของมอเตอร์
bool motorActive2 = false;              // สถานะการทำงานของมอเตอร์
bool Status_CAN_INDICATOR = false;
bool Status_CAN_JOYSTICK = false;
bool deployed = false;  // สถานะการทำงานของรีโมท
bool stowed = true;    // สถานะการทำงานของรีโมท
bool deployjoystick = false;  // สถานะการทำงานของรีโมท
bool stowjoystick = true;    // สถานะการทำงานของรีโมท


int replayIndex = 0;                    // ตำแหน่งปัจจุบันของการเล่นซ้ำ
int status_stop = 0;                    // ตำแหน่งปัจจุบันของการเล่นซ้ำ
unsigned long lastReplayTime = 0;       // เวลาเฟรมสุดท้ายที่เล่น
unsigned long replayStartTime = 0;      // เวลาที่เริ่มเล่นซ้ำ
unsigned long firstRecordedTime = 0;    // เวลาแรกสุดตอนบันทึก
unsigned long valveMotorStartTime = 0;  // เวลาเริ่มหมุนมอเตอร์ของวาล์วs
unsigned long previousTime = 0;         // เก็บเวลาล่าสุดที่บันทึก
unsigned long previousTime2 = 0;        // เก็บเวลาล่าสุดที่บันทึก

const unsigned long motorRunTime = 1000;  // มอเตอร์หมุน 1 วินาที
unsigned long lastMovementTime = 0;     // เวลาเคลื่อนไหวล่าสุด
const unsigned long idleTime = 125;      // เวลาที่ไม่มีการเคลื่อนไหว (2 วินาที)
unsigned long A = 0;
unsigned long B = 0;

static uint8_t previous_spare1 = 0;// เพิ่มตัวแปรเก็บสถานะก่อนหน้าของ spare1
static bool remoteMode = false;// เพิ่มตัวแปรสถานะการทำงานของรีโมท

uint8_t msb;         //ซ้ายขวา
uint8_t byte2;       //ความเร็วมอเตอร์
uint8_t byte3;       //ขึ้นลง
uint8_t byte4;       //ความเร็วมอเตอร์
uint8_t byte6;       //stream/fog
uint8_t byte7;       //เปิดปิดวาล์ว

HardwareSerial mySerial(USART1);
STM32_CAN Can( CAN1, DEF );  //Use PA11/12 pins for CAN1.

static CAN_message_t CAN_RX_msg;
static CAN_message_t CAN_TX_msg;


void updateCANIndicator() {
  CAN_TX_msg.id = 0x069;
  CAN_TX_msg.len = 8;
  CAN_TX_msg.buf[0] = floor(indexA/5);  // ค่าของ indexA (0-32)
  CAN_TX_msg.buf[1] = floor(indexB/5);  // ค่าของ indexB (0-24)
  Can.write(CAN_TX_msg);
  bool send = Can.write(CAN_TX_msg);
  mySerial.println(send);
}

void stopMotors() {
  analogWrite(MOTOR1_PWM_PIN, 0);  // หยุดมอเตอร์ 1
  analogWrite(MOTOR2_PWM_PIN, 0);  // หยุดมอเตอร์ 2
  digitalWrite(MOTOR1_DIR_PIN, LOW); // ตั้งทิศทางมอเตอร์ 1 เป็น LOW
  digitalWrite(MOTOR2_DIR_PIN, LOW); // ตั้งทิศทางมอเตอร์ 2 เป็น LOW
 
  mySerial.println("Motors stopped.");
}
void setup()
{
  mySerial.begin(115200);
  // ตั้งค่าขา LED
  pinMode(LED_Status_Can_Remote, OUTPUT);
  pinMode(LED_Status_Can_Indicator, OUTPUT);
  pinMode(LED_Status_Can_Joystick, OUTPUT);
  pinMode(LED_Status_Can_Error, OUTPUT);
 
  // ตั้งค่าขา PWM เป็น Output
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);
  pinMode(MOTOR3_PWM_PIN, OUTPUT);
  pinMode(MOTOR4_PWM_PIN, OUTPUT);

  // ตั้งค่าขา Direction เป็น Output
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR3_DIR_PIN, OUTPUT);
  pinMode(MOTOR4_DIR_PIN, OUTPUT);


  Can.begin();
  Can.setBaudRate(250000);  //250KBPS
  // Can.setBaudRate(500000);  //500KBPS
  //Can.setBaudRate(1000000);  //1000KBPS

  // OFF LED
  digitalWrite(LED_Status_Can_Error, HIGH);
  digitalWrite(LED_Status_Can_Indicator, HIGH);
  digitalWrite(LED_Status_Can_Joystick, HIGH);
  digitalWrite(LED_Status_Can_Remote, HIGH);
}



void processValveMotor(bool buttonPressed) {
  unsigned long currentTime = millis();

  if (buttonPressed) {
    if (!motorActive) {
      mySerial.println("🔄 เปิดวาล์ว");
      A = millis();
      analogWrite(MOTOR4_PWM_PIN, 200);  
      digitalWrite(MOTOR4_DIR_PIN, HIGH); 
      valveMotorStartTime = currentTime;
      motorActive = true;
      valveState = true;
    } 
  } else {
    if (motorActive) {
      mySerial.println("↩ ปิดวาล์ว");
      B = millis();
      analogWrite(MOTOR4_PWM_PIN, 200);  
      digitalWrite(MOTOR4_DIR_PIN, LOW);

      valveMotorStartTime = currentTime;
      valveState = false;
      motorActive = false;
      motorActive2 = true;
    }
  }

  if (motorActive && (currentTime - valveMotorStartTime >= motorRunTime)) {
    analogWrite(MOTOR4_PWM_PIN, 0);  
    mySerial.println("⏹ มอเตอร์วาล์วหยุดทำงาน");
    B = millis();
    
  }
  if (motorActive2 && ((currentTime - valveMotorStartTime >= motorRunTime ) || (currentTime - B >=  B-A)) ) {
    analogWrite(MOTOR4_PWM_PIN, 0);
    mySerial.println("⏹ มอเตอร์วาล์วหยุดทำงาน");
    motorActive2 = false;
  }
}

void processJoystick(uint8_t msb, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte6, uint8_t byte7) {

  String horizontal = "";
  String vertical = "";
  String special1 = "";

  // กำหนดทิศทางการเคลื่อนไหว
  if (msb == 4 || msb == 132) horizontal = "L(" + String(byte2) + ")";           // ซ้าย
  else if (msb == 16 || msb == 144) horizontal = "R(" + String(byte2) + ")";  // ขวา

  if (byte3 == 16 || byte3 == 144) vertical = "U(" + String(byte4) + ")";   // ขึ้น
  else if (byte3 == 4 || byte3 == 132) vertical = "D(" + String(byte4) + ")";  // ลง

  if (byte6 == 16) special1 = "🔵";          // ปุ่มสีน้ำเงิน
  else if (byte6 == 64) special1 = "🟣";  // ปุ่มสีม่วง



  String movement = horizontal + vertical + special1;

  // บันทึกการเคลื่อนไหวและ PWM
  if (recording) {
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= 50) {
      previousTime = currentTime;  // อัปเดตเวลาล่าสุดที่บันทึก
      String pwmData = String(byte2) + "," + String(byte4);
      joystickSequence.push_back(String(currentTime) + "|" + movement + " PWM: " + pwmData);

    }
  }


  if (recording && movement.length() > 0) {
    mySerial.print("Recording: ");
    mySerial.println(movement);
  }

  // ปริ้นค่าทิศทางและ PWM แบบปกติถ้าไม่ได้บันทึก
  if (!recording && !replaying && movement.length() > 0) {
    mySerial.print("Joystick Movement: ");
    mySerial.println(movement);
  }

  // ตรวจสอบการเคลื่อนไหวและ PWM
  if (movement.length() > 0) {
      lastMovementTime = millis();  // อัปเดตเวลาการเคลื่อนไหวล่าสุด
  }


  if (byte6 == 16) {  // 🔵 ปุ่มน้ำเงิน → หมุนซ้าย
    mySerial.println("Motor 3 Direction: Left");
    digitalWrite(MOTOR3_DIR_PIN, LOW);
    analogWrite(MOTOR3_PWM_PIN, 200);  

  } else if (byte6 == 64) {  // 🟣 ปุ่มม่วง → หมุนขวา
    mySerial.println("Motor 3 Direction: Right");
    digitalWrite(MOTOR3_DIR_PIN, HIGH);
    analogWrite(MOTOR3_PWM_PIN, 200);  

  } else {  // ปล่อยปุ่ม → หยุดมอเตอร์ที่ 3
    analogWrite(MOTOR3_PWM_PIN, 0); 

  }


  // สั่งมอเตอร์ตามการเคลื่อนไหวของจอยสติ๊ก
  if (!replaying) {
    if (byte2 > 0) {
      if (msb == 4 || msb == 132) {  // ซ้าย
        mySerial.println("Motor 1 Direction: Left");
        digitalWrite(MOTOR1_DIR_PIN, LOW);
        analogWrite(MOTOR1_PWM_PIN, byte2);  
        status_stop = 0;

        if (indexA > 0) indexA--;  // ลด index (0-32)
        updateCANIndicator();

      } else if (msb == 16 || msb == 144) {  // ขวา
        mySerial.println("Motor 1 Direction: Right");
        digitalWrite(MOTOR1_DIR_PIN, HIGH);
        analogWrite(MOTOR1_PWM_PIN, byte2);  
        status_stop = 0;

        if (indexA < MAX_INDEX_A) indexA++;  // เพิ่ม index (0-32)
        updateCANIndicator();
      }
    } 




    if (byte4 > 0) {
      if (byte3 == 16 || byte3 == 144) {  // ขึ้น
        mySerial.println("Motor 2 Direction: Up");
        digitalWrite(MOTOR2_DIR_PIN, HIGH);
        analogWrite(MOTOR2_PWM_PIN, byte4);  

        if (indexB < MAX_INDEX_B) indexB++;  // เพิ่ม index (0-24)
        updateCANIndicator();

        status_stop = 0;
      } else if (byte3 == 4 || byte3 == 132) {  // ลง
        mySerial.println("Motor 2 Direction: Down");
        digitalWrite(MOTOR2_DIR_PIN, LOW);
        analogWrite(MOTOR2_PWM_PIN, byte4);  

        if (indexB > 0) indexB--;  // ลด index (0-24)
        updateCANIndicator();

        status_stop = 0;
      }
    } 
  }
}


void Deploy() {
    if (!deployed && stowed) {
      deployed = true;
      stowed = false;
      mySerial.println("Deploying...");
    } 
}

void Stow() {
    if (!stowed && deployed) {
      stowed = true;
      deployed = false;
      mySerial.println("Stowing...");
    } 
  }


void loop()
{


  //////////////////joystick///////////////////////////////////////
  if (Can.read(CAN_RX_msg)) {
    

     ///////////////////////////Teach//////////////////////////////////
    if (CAN_RX_msg.id == REMOTE_CAN_ID) {                  // CAN Recevier REMOTE
      unsigned long currentTime2 = millis();
      if (currentTime2 - previousTime2 >= 50) {
        previousTime2 = currentTime2;  // อัปเดตเวลาล่าสุดที่บันทึก
        digitalToggle(LED_Status_Can_Remote);

      }
      // ดึงข้อมูลจาก message
      uint8_t status = CAN_RX_msg.buf[1];
      uint8_t lever1_updown = CAN_RX_msg.buf[3];
      uint8_t lever1_leftright = CAN_RX_msg.buf[4];
      uint8_t lever2_updown = CAN_RX_msg.buf[5];

      // สร้างตัวแปรตามที่กำหนด
      uint8_t msb1 = 0;
      if (lever1_leftright < 128) msb1 = 4;
      else if (lever1_leftright > 128) msb1 = 16;
      else msb1 = 0;

      uint8_t byte22 = 0;
      if (lever1_leftright < 128) {
        byte22 = map(lever1_leftright, 0, 128, 255, 0);
      }

      if (lever1_leftright > 128) {
        byte22 = map(lever1_leftright, 128, 255, 0, 255);
      }

      uint8_t byte33 = 0;
      if (lever1_updown > 128) byte33 = 16;
      else if (lever1_updown < 128) byte33 = 4;
      else byte33 = 0;

      uint8_t byte44 = 0;
      if (lever1_updown < 128) {
        byte44 = map(lever1_updown, 0, 128, 255, 0);
      }

      if (lever1_updown > 128) {
        byte44 = map(lever1_updown , 128, 255, 0, 255);
      }

      uint8_t byte66 = 0;
      if (lever2_updown > 128) byte66 = 16;
      else if (lever2_updown < 128) byte66 = 64;
      else byte66= 0;

      uint8_t byte77 = (status & 4) ? 1 : 0; // On/Off button

      // ปุ่มอื่น ๆ ในรูปแบบ 0/1
      uint8_t spare1 = (status & 2) ? 1 : 0;
      uint8_t deploy_stow = (status & 8) ? 1 : 0;
      uint8_t spare2 = (status & 16) ? 1 : 0;
      uint8_t spare3 = (status & 32) ? 1 : 0;
      uint8_t teach = (status & 64) ? 1 : 0;

      // mySerial.println(deploy_stow);

      if (spare1 == 1 && previous_spare1 == 0) {
        // สลับสถานะการทำงาน
        remoteMode = !remoteMode;
        mySerial.println("Remote Mode: ");
        
        if (remoteMode) {
            // เปลี่ยนเป็นโหมดรีโมท
            mySerial.println("ON");
            enableRemote = true;
            enableJoystick = false;
            // ทำสิ่งอื่นๆ ที่ต้องการเมื่อเปลี่ยนเป็นโหมดรีโมท...
        } else {
            // เปลี่ยนเป็นโหมดจอยสติ๊ก
            mySerial.println("OFF");
            enableRemote = false;
            enableJoystick = true;
            // ทำสิ่งอื่นๆ ที่ต้องการเมื่อเปลี่ยนเป็นโหมดจอยสติ๊ก...
        }
    }
        // อัพเดตค่า spare1 ก่อนหน้า
        previous_spare1 = spare1;

      if (enableRemote) {
        // mySerial.println("Remote Mode: ON");
        if (deploy_stow == 1) {
          Deploy();  // เรียกฟังก์ชัน Deploy
          processValveMotor(byte77 == 1);
          processJoystick(msb1, byte22, byte33, byte44, byte66, byte77);
          
        } else {
          Stow();  // เรียกฟังก์ชัน Stow
        }

      }
      // mySerial.println();
      // mySerial.print("  MSB    "); mySerial.print(msb);
      // mySerial.print("  Byte2  "); mySerial.print(byte2);
      // mySerial.print("  Byte3  "); mySerial.print(byte3);
      // mySerial.print("  Byte4  "); mySerial.print(byte4);
      // mySerial.print("  Byte6  "); mySerial.print(byte6);
      // mySerial.print("  Byte7  "); mySerial.println(byte7);
  
      // mySerial.print("Spare1: "); mySerial.print(spare1);
      // mySerial.print("  Deploy/Stow: "); mySerial.print(deploy_stow);
      // // mySerial.print("  Spare2: "); mySerial.print(spare2);
      // // mySerial.print("  Spare3: "); mySerial.print(spare3);
      // mySerial.print("  Teach: "); mySerial.println(teach);

      // ตรวจจับการกดปุ่มบันทึก/หยุดเล่นซ้ำ
      if (teach == 1 && lastButtonState == false) {
        if (replaying) {
          mySerial.println("⏹ Stopping replay...");
          Status_CAN_JOYSTICK = true;
          replaying = false;
          joystickSequence.clear();  // ล้างข้อมูลที่บันทึกไว้
        } else {
          mySerial.println("📌 Start Recording...");
          // joystickSequence.clear();  // ล้างค่าที่บันทึกไว้
          recording = true;
        }
        status_stop = 0;
      }

      lastButtonState = (teach == 1);  // อัปเดตสถานะปุ่มล่าสุด

      // หยุดบันทึกและเริ่มเล่นซ้ำ
      if (recording && teach == 0) {
        mySerial.println("✅ Stop Recording. Start Replaying...");
        recording = false;
        replaying = true;
        Status_CAN_JOYSTICK = true;
        replayIndex = 0;
        lastReplayTime = millis();
        replayStartTime = millis();                                                                      // เก็บเวลาเริ่มเล่นซ้ำ
        firstRecordedTime = joystickSequence[0].substring(0, joystickSequence[0].indexOf("|")).toInt();  // เวลาแรกสุดของการบันทึก
      }

    }


      if (CAN_RX_msg.id == JOYSTICK_CAN_ID) {
        unsigned long currentTime2 = millis();
        if (currentTime2 - previousTime2 >= 50) {
          previousTime2 = currentTime2;  // อัปเดตเวลาล่าสุดที่บันทึก
          digitalToggle(LED_Status_Can_Joystick);
        }
          uint8_t msb = CAN_RX_msg.buf[0];         //ซ้ายขวา
          uint8_t byte2 = CAN_RX_msg.buf[1];       //ความเร็วมอเตอร์
          uint8_t byte3 = CAN_RX_msg.buf[2];       //ขึ้นลง
          uint8_t byte4 = CAN_RX_msg.buf[3];       //ความเร็วมอเตอร์
          uint8_t byte6 = CAN_RX_msg.buf[5];       //stream/fog
          uint8_t byte7 = CAN_RX_msg.buf[6];       //เปิดปิดวาล์ว

        if (deployjoystick && !stowjoystick) {
          Deploy();  // เรียกฟังก์ชัน Deploy
          processValveMotor(byte7 == 1);
          processJoystick(msb, byte2, byte3, byte4, byte6, byte7);
          enableRemote = false;  // ปิดการทำงานของรีโมท
          deployjoystick = false;  // อัปเดตสถานะการทำงานของรีโมท
         
        } else if (!deployjoystick && stowjoystick) {
          Stow();  // เรียกฟังก์ชัน Stow
          stowjoystick = false;  // อัปเดตสถานะการทำงานของรีโมท
        }

  }


            ///////////////////////////////////////////
    if (CAN_RX_msg.id == SW_JOYSTICK_CAN_ID) {            // ปุ่มจาก joystick 6 ปุ่ม
      uint8_t byteJT0 = CAN_RX_msg.buf[0];  // Teach
      uint8_t byteJT1 = CAN_RX_msg.buf[1];  // On/Off Valve
      uint8_t byteJT2 = CAN_RX_msg.buf[2];  // On/Off Remote
      uint8_t byteJT3 = CAN_RX_msg.buf[3];  // Spare1
      uint8_t byteJT4 = CAN_RX_msg.buf[4];  // Spare2
      uint8_t byteJT5 = CAN_RX_msg.buf[5];  // Deploy/STOW

      if (enableJoystick) {
        if (byteJT5 == 1) {
         deployjoystick = true;  // เรียกฟังก์ชัน Deploy

        } else if (byteJT5 == 0) {
        //  stowjoystick = true;  // เรียกฟังก์ชัน Stow
         stowjoystick = true;  // เรียกฟังก์ชัน Deploy
        }
      }
      // mySerial.println("SW_JOYSTICK_CAN_ID: " + String(byteJT0) + " " + String(byteJT1) + " " + String(byteJT2) + " " + String(byteJT3) + " " + String(byteJT4) + " " + String(byteJT5));


      // ตรวจจับการกดปุ่มบันทึก/หยุดเล่นซ้ำ
      if (byteJT0 == 1 && lastButtonState == false) {
        if (replaying) {
          mySerial.println("⏹ Stopping replay...");
          Status_CAN_JOYSTICK = true;
          replaying = false;
          joystickSequence.clear();  // ล้างข้อมูลที่บันทึกไว้
        } else {
          mySerial.println("📌 Start Recording...");
          // joystickSequence.clear();  // ล้างค่าที่บันทึกไว้
          recording = true;
        }
        status_stop = 0;
      }

      lastButtonState = (byteJT0 == 1);  // อัปเดตสถานะปุ่มล่าสุด

      // หยุดบันทึกและเริ่มเล่นซ้ำ
      if (recording && byteJT0 == 0) {
        mySerial.println("✅ Stop Recording. Start Replaying...");
        recording = false;
        replaying = true;
        Status_CAN_JOYSTICK = true;
        replayIndex = 0;
        lastReplayTime = millis();
        replayStartTime = millis();                                                                      // เก็บเวลาเริ่มเล่นซ้ำ
        firstRecordedTime = joystickSequence[0].substring(0, joystickSequence[0].indexOf("|")).toInt();  // เวลาแรกสุดของการบันทึก
      }
    }

    // **เล่นซ้ำแบบไม่บล็อก**
    if (replaying && !joystickSequence.empty()) {
      unsigned long currentReplayTime = millis() - replayStartTime;  // เวลาที่ผ่านไปตั้งแต่เริ่มเล่นซ้ำ
      String command = joystickSequence[replayIndex];

      // ดึง timestamp ของคำสั่งที่กำลังจะเล่น
      int delimiterIndex = command.indexOf("|");
      unsigned long recordedTime = command.substring(0, delimiterIndex).toInt();
      recordedTime -= firstRecordedTime;  // ทำให้เริ่มที่ 0

      // เล่นคำสั่งเมื่อถึงเวลาที่บันทึกไว้
      if (currentReplayTime >= recordedTime) {
        String movement = command.substring(delimiterIndex + 1);  // ข้อมูลที่เหลือ
        mySerial.println("Replaying: " + movement);

        // แยก PWM
        int pwm1 = movement.substring(movement.indexOf("PWM:") + 5, movement.indexOf(",")).toInt();
        int pwm2 = movement.substring(movement.indexOf(",") + 1).toInt();

        if (movement.indexOf("L") >= 0) {
          mySerial.println("Motor 1 Direction: Left");
          digitalWrite(MOTOR1_DIR_PIN, LOW);
          analogWrite(MOTOR1_PWM_PIN, pwm1);  
          if (indexA > 0) indexA-=1;  // ลด index (0-32)
          updateCANIndicator();
        } else if (movement.indexOf("R") >= 0) {
          mySerial.println("Motor 1 Direction: Right");
          digitalWrite(MOTOR1_DIR_PIN, HIGH);
          analogWrite(MOTOR1_PWM_PIN, pwm1);  
          if (indexA < MAX_INDEX_A) indexA+=1;  // เพิ่ม index (0-32)
          updateCANIndicator();

        } else if (movement.indexOf("U") >= 0) {
          mySerial.println("Motor 2 Direction: Up");
          digitalWrite(MOTOR2_DIR_PIN, HIGH);
          analogWrite(MOTOR2_PWM_PIN, pwm2);  
          if (indexB < MAX_INDEX_B) indexB+=1;  // เพิ่ม index (0-24)
          updateCANIndicator();
        } else if (movement.indexOf("D") >= 0) {
          mySerial.println("Motor 2 Direction: Down");
          digitalWrite(MOTOR2_DIR_PIN, LOW);
          analogWrite(MOTOR2_PWM_PIN, pwm2);  
          if (indexB > 0) indexB-=1;  // ลด index (0-24)
          updateCANIndicator();
        } else {
          stopMotors();
        }
        

        replayIndex++;
        if (replayIndex >= joystickSequence.size()) {
          replayIndex = 0;             // วนซ้ำจากต้น
          replayStartTime = millis();  // เริ่มต้นใหม่ทุกครั้งที่วน
        }
      }
    }

    // ตรวจสอบเวลาที่ไม่มีการเคลื่อนไหว
    if ((millis() - lastMovementTime > idleTime) && status_stop == 0) {
      stopMotors();  // หยุดมอเตอร์หากไม่มีการเคลื่อนไหว
      status_stop = 1;
    }
/////////////////////////////////teach////////////////////////////////////////






    if(Status_CAN_JOYSTICK && replaying){                             //  CAN Tansmitter  JOYSTICK callback
      CAN_TX_msg.id = (0x122);
      CAN_TX_msg.len = 8;
      CAN_TX_msg.buf[0] =  1;   // Teach led
      CAN_TX_msg.buf[1] =  0;   // On/Off Valve led
      CAN_TX_msg.buf[2] =  0;   // On/Off Remote led
      CAN_TX_msg.buf[3] =  0;   // Spare1 led
      CAN_TX_msg.buf[4] =  0;   // Spare2 led
      CAN_TX_msg.buf[5] =  0;   // Deploy/STOW led
      Can.write(CAN_TX_msg);
      Status_CAN_JOYSTICK = false;
    }else if(Status_CAN_JOYSTICK && !replaying){                             //  CAN Tansmitter  JOYSTICK callback
      CAN_TX_msg.id = (0x122);
      CAN_TX_msg.len = 8;
      CAN_TX_msg.buf[0] =  0;   // Teach led
      CAN_TX_msg.buf[1] =  0;   // On/Off Valve led
      CAN_TX_msg.buf[2] =  0;   // On/Off Remote led
      CAN_TX_msg.buf[3] =  0;   // Spare1 led
      CAN_TX_msg.buf[4] =  0;   // Spare2 led
      CAN_TX_msg.buf[5] =  0;   // Deploy/STOW led
      Can.write(CAN_TX_msg);
      Status_CAN_JOYSTICK = false;
    }


    if (CAN_RX_msg.id == INDICATOR_CAN_ID) {              //  CAN Recevier INDICATOR 
      unsigned long currentTime2 = millis();
      if (currentTime2 - previousTime2 >= 50) {
        previousTime2 = currentTime2;  //  biuk
        digitalToggle(LED_Status_Can_Indicator);
      }
    }
    if(Status_CAN_INDICATOR){                             //  CAN Tansmitter  INDICATOR callback
      CAN_TX_msg.id = (0x069);
      CAN_TX_msg.len = 8;
      CAN_TX_msg.buf[0] =  0;   // index gruopA 32 led
      CAN_TX_msg.buf[1] =  0;   // index gruopB 24 led
      Can.write(CAN_TX_msg);
    }

  }

  delay(5);

}


