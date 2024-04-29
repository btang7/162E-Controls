/*
 * @Author: Maclab
 * @Date: 2024-02-06 11:59:09
 * @LastEditTime: 2020-12-18 14:14:35
 * @LastEditors: AJ<3
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */
#include <avr/wdt.h>
//#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "DeviceDriverSet_xxx0.h"
#include <Arduino.h>
#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

extern "C" { 
#include "DemoWeek6.h"
#include "DemoWeek6_private.h"
#include "DemoWeek6_types.h"
}
/*Hardware device object list*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;

DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;

/*f(x) int */
static boolean
function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}
static void
delay_xxx(uint16_t _ms)
{
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}

void ApplicationFunctions_Init(void)
{
    bool res_error = true;
    AppVoltage.DeviceDriverSet_Voltage_Init(); 
    AppMotor.DeviceDriverSet_Motor_Init(); 
    AppServo.DeviceDriverSet_Servo_Init(90); 
    AppKey.DeviceDriverSet_Key_Init();
    AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
    AppIRrecv.DeviceDriverSet_IRrecv_Init();
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
    AppITR20001.DeviceDriverSet_ITR20001_Init();
    res_error = AppMPU6050getdata.MPU6050_dveInit();
    AppMPU6050getdata.MPU6050_calibration();

    // Intialize DemoWeek 5 Parameters 
    // DemoWeek6_P.controlEN = true; 
    // DemoWeek6_P.dir_MA    = true; 
    // DemoWeek6_P.dir_MB    = true; 
    // DemoWeek6_P.speed_MA  = 128; 
    // DemoWeek6_P.speed_MB  = 64; 
}

// Initialize some variables 
float Yaw; // yaw angle from the IMU
int IRSensL; // Left IR sensor 
int IRSensM; // Middle IR sensor 
int IRSensR; // Right IR sensor
uint8_t keyValue; // key value
float device_voltage; // pin voltage
uint16_t ultrasonic_fb; // ultrasonic reading 
bool IRerror; // IR receive error 
uint8_t IRrecv_code; // IR receive code 
unsigned long previous_time = millis(); 

/* Motor Inputs */ 
bool dirMA; 
bool dirMB; 
bool motEN; 
uint8_t PWMA; 
uint8_t PWMB; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  ApplicationFunctions_Init(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw); // Get vehicle orientation
  IRSensL = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L(); 
  IRSensM = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M(); 
  IRSensR = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R(); 
  AppKey.DeviceDriverSet_key_Get(&keyValue); 
  device_voltage = AppVoltage.DeviceDriverSet_Voltage_getAnalogue(); 
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&ultrasonic_fb); 
  AppIRrecv.DeviceDriverSet_IRrecv_Get(&IRrecv_code); 

  /* Send fb data to Simulink Module */ 
  DemoWeek6_U.IRSensorL_in     = IRSensL; 
  DemoWeek6_U.IRSensorM_in     = IRSensM; 
  DemoWeek6_U.IRSensorR_in     = IRSensR; 
  DemoWeek6_U.VoltageDetect_in = device_voltage; 
  DemoWeek6_U.UltraSensor_in   = ultrasonic_fb; 
  DemoWeek6_U.Key_in           = keyValue; 
  DemoWeek6_U.MPU6050IMU_yaw_in= Yaw; 
  DemoWeek6_U.IRSensorCode_in  = IRrecv_code; 

  /* Step Simulink Module*/
  DemoWeek6_step(); 

  /* Extact outputs from Simulink Module */
  PWMA  = DemoWeek6_Y.MotorASpeed; 
  PWMB  = DemoWeek6_Y.MotorBSpeed; 
  motEN = DemoWeek6_Y.MotorEN; 
  dirMA = DemoWeek6_Y.direction_MA; 
  dirMB = DemoWeek6_Y.direction_MB; 

  /* Send commands to actuators */ 
  AppMotor.DeviceDriverSet_Motor_control(dirMA, PWMA, dirMB, PWMB , motEN); 
  
  /* Verify remaining outputs */ 
  if (millis()- previous_time >= 1000){
    Serial.print(Yaw); Serial.print(","); 
    /*Serial.print("deg"); 
    /* Serial.print("  IR_L= "); */ Serial.print(IRSensL); Serial.print(","); 
    /* Serial.print("  IR_M= "); */ Serial.print(IRSensM); Serial.print(",");
    /* Serial.print("  IR_R= "); */ Serial.print(IRSensR); Serial.print(",");

    /*Serial.print("  Key= ");*/ Serial.print(keyValue); Serial.print(",");
    /*Serial.print("  Device Voltage= ");*/ Serial.print(device_voltage);Serial.print(",");
    /*Serial.print("  IRrecvCode= "); */ Serial.print(IRrecv_code); Serial.print(","); 
    /*Serial.print("  Ultrasonic fb: "); */ Serial.print(ultrasonic_fb);Serial.print(","); 

    /*Serial.print("Yawo= ");*/ 
    Serial.print(DemoWeek6_Y.IMU_yaw_out); Serial.print(",");
    /*Serial.print("deg"); */
    /*Serial.print(" IRLo= ");*/ Serial.print(DemoWeek6_Y.IRSensorL_out);Serial.print(",");
    /*Serial.print(" IRMo= ");*/ Serial.print(DemoWeek6_Y.IRSensorM_out);Serial.print(",");
    /*Serial.print(" IRRo= ");*/ Serial.print(DemoWeek6_Y.IRSensorR_out);Serial.print(",");

    /*Serial.print(" Keyo= ");*/ Serial.print(DemoWeek6_Y.key_out);Serial.print(",");
    /*Serial.print(" DeviceVoltageo= ");*/ Serial.print(DemoWeek6_Y.VoltageDetect_out);Serial.print(",");
    /*Serial.print(" IRrecvCodeo= ");*/ Serial.print(DemoWeek6_Y.IRSensorCode_out);Serial.print(",");  
    /*Serial.print(" Ultrasonicfbo: ");*/ Serial.println(DemoWeek6_Y.UltraSensor_out); 
    previous_time=millis(); 
  }
}
