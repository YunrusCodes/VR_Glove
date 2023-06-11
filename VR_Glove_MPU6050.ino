#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define ARRAY_SIZE(array) ((sizeof(array))/(sizeof(array[0])))

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
void setPwmStrength();
// pwm parameter
String getPwmNum = "";
const int cPwmNum = 5;
int pwmStrength[cPwmNum] = {0};
int pwmNum = cPwmNum;
// pwm parameter end

typedef enum{
  WRIST = 7,
  INDEX = 6,
  MIDDLE = 10,
  RING = 9,
  PINKY = 8,
  THUMB = 5
} HandPart;

typedef enum{
  AD0_LOW = 0x68,
  AD0_HIGH
} AD0State;

typedef enum{
  ACCEL_XOUT_H = 0x3B,
  ACCEL_XOUT_L,
  ACCEL_YOUT_H,
  ACCEL_YOUT_L,
  ACCEL_ZOUT_H,
  ACCEL_ZOUT_L,  
  GYRO_XOUT_H = 0x43,
  GYRO_XOUT_L,
  GYRO_YOUT_H,
  GYRO_YOUT_L,
  GYRO_ZOUT_H,
  GYRO_ZOUT_L,
  PWR_MGMT_1 = 0x6B,
  PWR_MGMT_2
} RegistMap;

typedef struct Data{
  String dataName;
  HandPart chip;
  RegistMap regAddress;
  int raw;
  double corr = 0;
  double realNum;
 };
 Data Finger[18];

double now, lastTime = 0;
double dt;
const int corrNum = 100; // 誤差筆數

void setup(){
   Serial.begin(115200);
   Wire.begin(); // Initiate the Wire library
   pinMode( WRIST, OUTPUT);
   pinMode( INDEX, OUTPUT);
   pinMode( MIDDLE, OUTPUT);
   pinMode( RING, OUTPUT);
   pinMode( PINKY, OUTPUT);
   pinMode( THUMB, OUTPUT);
   WriteRegister( 0x68, PWR_MGMT_1, 0x00 );
   WriteRegister( 0x69, PWR_MGMT_1, 0x00 );

   // 以下程式校正所有部位的三軸陀螺儀數據
   HandPart hand[6] = {WRIST,INDEX,MIDDLE,RING,PINKY,THUMB};
   String handPartName[6] = {"Wrist","Index","Middle","Ring","Pinky","Thumb"};
   RegistMap gyro[3] = {GYRO_XOUT_H,GYRO_YOUT_H,GYRO_ZOUT_H};
   char gyroDirction[3] = {'x','y','z'};
   
   for(int i = 0; i < ARRAY_SIZE(Finger) ; i++){
     Finger[i].dataName = handPartName[i / 3] + "_" + gyroDirction[i % 3];
     Finger[i].chip = hand[i / 3];
     Finger[i].regAddress = gyro[i % 3];
   }//for(int i = 0; i < ARRAY_SIZE(Finger) ; i++)
   
   for(int i = 0; i < corrNum ; i++){
     for(int j = 0; j < ARRAY_SIZE(Finger); j++){
      Finger[j].raw = Read16BitsOnChip( Finger[j].chip, Finger[j].regAddress);
      Finger[j].corr += Finger[j].raw/corrNum;
     } // for(int j = 0; j < ARRAY_SIZE(Finger); j++)
   } // for(int i = 0; i < corrNum ; i++)
} // void setup()

void loop(){
  now = millis();             //当前时间(ms)
  dt = (now - lastTime);           //微分时间(s)
  lastTime = now;
  // 以下程式列印出所有的陀螺儀數據
  for(int i = 0; i <  ARRAY_SIZE(Finger) ; i++){
   Finger[i].raw = Read16BitsOnChip( Finger[i].chip, Finger[i].regAddress);
   Finger[i].realNum = ((Finger[i].raw - Finger[i].corr)/131)*dt/900;
   Serial.print(Finger[i].dataName); Serial.print(",");
   Serial.print(Finger[i].realNum);
   (i < ARRAY_SIZE(Finger)-1)? Serial.print(",") : Serial.println();
  }//for(int i = 0; i < 18 ; i++)
 
  // 以下程式更新pwm強度
  setPwmStrength();
  // for(uint8_t pin=0; pin<cPwmNum; pin++)
  //   pwm.setPWM(pin, pwmStrength[pin], 4096 - pwmStrength[pin]);
   analogWrite(11,pwmStrength[0]);
} // void loop()

void WriteRegister( int i2cAddress, int registAddress, int uWriteNum ){
   //I2C傳輸地址位於"i2cAddress"的mpu6050，將"uWriteNum"寫入其"registAddress"的暫存器位置
   Wire.beginTransmission(i2cAddress);
   Wire.write(registAddress);
   Wire.write(uWriteNum);
   Wire.endTransmission(true);
} // void WriteRegister( AD0State ad0, RegistMap target, int writed )

int Read16BitsRegister(int i2cAddress, int registAddress){
  // 讀取I2C傳輸地址位於"i2cAddress"之"registADDRESS"位置
  int readInMessage;
  Wire.beginTransmission(i2cAddress);
  Wire.write(registAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 2, true);
  readInMessage = (Wire.read() << 8 | Wire.read());
  return readInMessage;
} // Read16BitsRegister(HandPart targetChip, AD0State ad0, RegistMap targetReg)

int Read16BitsOnChip(int targetChip, int targetReg){
  // 請從上個任務複製這個函式
  int message = 0;
  digitalWrite(targetChip, HIGH);
  message = Read16BitsRegister(0x69, targetReg);
  digitalWrite(targetChip, LOW);
  return message;
} // int Read16BitsOnChip(int targetChip, int targetReg)

void setPwmStrength() {
  // 讀取serial並設定pwm強度

  if(Serial.available() > 0){
    getPwmNum = "";
    pwmNum = 0;
  } // if(Serial.available() > 0)
 
  while(Serial.available() > 0) {
    char incomingByte = Serial.read();
    if(!isDigit(incomingByte)){
      if( pwmNum < cPwmNum)
        pwmStrength[pwmNum] =  getPwmNum.toInt();
      getPwmNum = "";
      pwmNum++;
    } // if
    else
      getPwmNum += (char)incomingByte;
    delayMicroseconds(100);
  } // while(Serial.available() > 0)
 
} // void setPwmStrength()