// avc I/O
// serial in/out main loop
//

#include <Wire.h>
#include <Metro.h>
#include <Servo.h>
#include <ADC.h>
#include <Time.h>
#include <AccelStepper.h>

#define IR_DEBUG 0

ADC *adc = new ADC();
ADC::Sync_result result;
ADC::Sync_result result1;

Servo MySpeed;
Servo MyDirection;
Servo Camera;

#define TELEMETRY "192.168.4.2" // Default telemetry server (first client) port 2223
#define TELEMETRYX "192.168.4.4"

long timer_old;

float valued1;
float valued2;
float valued3;
float valued4;

uint16_t rf_Distance;
uint16_t rr_Distance;
uint16_t lf_Distance;
uint16_t lr_Distance;
uint16_t LR_distance;
uint16_t SR_distance;

bool serialin;

union In_Data
{
  uint16_t Cmd_data[4];
  uint8_t Cmd_Bytes[8];
};

union In_Data commands;

union TmData
{
  uint16_t Write[24];
  uint8_t Read[48];
};

union TmData Telemetry;

uint16_t cmdAcceptCount = 0;
uint16_t currMode = 0;

elapsedMillis currTime;

const int motor_Battery = A8;
const int elect_Battery = A9;
const int distanceRF = A15;
const int distanceRR = A12;
const int distanceLF = A14;
const int distanceLR = A13;
const int longFront = A0;
const int shortFront = A1;

const int lrsr_Sel = 10;
const int Run = 6;
const int l_Wheel = 11;
const int r_Wheel = 12;

char Cmd;

// AUX definitions

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

String MAC;  // MAC address of Wifi module

AccelStepper ir_Scan(AccelStepper::HALF4WIRE, 24, 26, 25, 27);

void setup()
{
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0);
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_1);
  adc->setResolution(12, ADC_0);
  adc->setResolution(12, ADC_1);
  adc->setAveraging(16, ADC_0);
  adc->setAveraging(16, ADC_1);

  pinMode(lrsr_Sel, OUTPUT);
  pinMode(Run, INPUT);
  pinMode(l_Wheel, INPUT);
  pinMode(r_Wheel, INPUT);

  digitalWrite(lrsr_Sel, 0);
  cmdAcceptCount = 0;
  

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

  for (int i = 0; i < 24; i++)
  {
    Telemetry.Write[i] = i;
  }

  // Initialize I2C bus (MPU6050 is connected via I2C)
  Wire.setClock(400000);
  Wire.begin();

  delay(200);
  /*  Serial.println("Don't move for 10 sec...");
    Serial.println("MPU6050 setup");
    //  MPU6050_setup();  // setup MPU6050 IMU
    delay(500);

    // With a ESP8266 WIFI MODULE WE NEED an INITIALIZATION PROCESS
    Serial.println("WIFI init");
    Serial1.flush();
    Serial1.print("+++");  // To ensure we exit the transparent transmision mode
    delay(250);
    ESPsendCommand("AT", "OK", 1);
    ESPsendCommand("AT+RST", "OK", 20); // ESP Wifi module RESET
    delay(250);
    //  ESPwait("ready", 20);
    ESPsendCommand("AT+GMR", "OK", 10);
    // generate a wifi access point
    Serial1.println("AT+CIPSTAMAC?");
    ESPgetMac();
    Serial.print("MAC:");
    Serial.println(MAC);
    delay(200);
    ESPsendCommand("AT+CWSAP?", "OK", 20);
    ESPsendCommand("AT+CWMODE=2", "OK", 10); // Soft AP mode
    // Generate Soft AP. SSID=TMavX1, PASS=password
    char cmd[] = "AT+CWSAP=\"TMavcX1_XX\",\"password\",6,3";
    // Update XX characters with MAC address (last 2 characters)
    cmd[18] = MAC[10];
    cmd[19] = MAC[11];
    Serial.println(cmd);
    ESPsendCommand(cmd, "OK", 6);
    ESPsendCommand("AT+CIPAP?", "OK", 10);

    // Start UDP SERVER on port 2222, telemetry port 2223
    Serial.println("Start UDP server");
    ESPsendCommand("AT+CIPMUX=0", "OK", 5);  // Single connection mode
    ESPsendCommand("AT+CIPMODE=1", "OK", 5); // Transparent mode
    char Telemetry[80];
    strcpy(Telemetry, "AT+CIPSTART=\"UDP\",\"");
    strcat(Telemetry, TELEMETRY);
    strcat(Telemetry, "\",2223,2222,0");
    ESPsendCommand(Telemetry, "OK", 15);if (Serial.available() >0)

    // Calibrate gyros
    //  MPU6050_calibrate();

    ESPsendCommand("AT+CIPSEND", ">", 5); // Start transmission (transparent mode)

    // Init servos
    Serial.println(">>>>>>>>>> Servo init <<<<<<<<<<");
    //  TMavc1_initServo();



    #if TELEMETRY_BATTERY==1
    BatteryValue = BROBOT_readBattery(true);
    Serial.print("BATT:");
    Serial.println(BatteryValue);
    //  Serial.println("50");
    #endif
    al.print("Current Time: ");
    uint32
    Serial.println("TMavcX1 by TinkerMill v0.2");
    Serial.println("Start...");
    timer_old = micros();if (Serial.available() >0)
  */

} // end setup

void loop() {
if (serialin);
{
  cmdDecode(Cmd);
  serialin = false;
}
  ReadSensors();
  delay(500);
  TmData();
} // end loop

void  cmdDecode(char cmd)
{
  switch (cmd)
  {
    //avc commands

    case 'M': // Move command
      Serial2.println("Move");
      cmdAcceptCount++;
      break;

    case 'T': // Turn
      Serial2.println("Turn Command");
      cmdAcceptCount++;
      break;

    case 'E': // Estop command
      Serial2.println("E Stopcommand");
      cmdAcceptCount++;
      break;

    case 'H': // Hartbeat
      Serial2.println("Hartbeat Command");
//      cmdAcceptCount++;
      break;

    case 'L': // lighting command
      Serial2.println("Lighting command");
      cmdAcceptCount++;
      break;

    case 'P': // Speed parms
      Serial2.println("speed parms Command");
      cmdAcceptCount++;
      break;

    case 'Q': // Steering parms 
      Serial2.println("speed parms command");
      cmdAcceptCount++;
      break; 

    case 'D': // Goto mode
      Serial2.println("goto mode Command");
      cmdAcceptCount++;
      break;

    case 'N': // NOP command
      Serial2.println("NOP command");
      cmdAcceptCount++;
      break;

    case 'S': // Scan speed
      Serial2.println("Scan speed Command");
      cmdAcceptCount++;
      break;

    case 'A': // Scan angle
      Serial2.println("Scan angle command");
      cmdAcceptCount++;
      break;

    case 'V': // Camera angle
      Serial2.println("Camera angle Command");
      cmdAcceptCount++;
      break;

    case 'C': // Set scan sensor
      Serial2.println("Set scan sensor command");
      cmdAcceptCount++;
      break;

    default:
      // do nothing
      break;

  } //end switch

  Serial2.print("Cmd Accept Ctr: ");
  Serial2.println(cmdAcceptCount);

} // end cmd decode


void serialEvent()
{
  int i = 0;
  while (Serial.available())
  {
    commands.Cmd_Bytes[i] = Serial.read(); 
      i++;
  }

  for ( i = 0; i < 4; i++)
  {
    Cmd = (char)commands.Cmd_data[0];
  }
  serialin = true;
} // end serial event


void TmData()
{
  Telemetry.Write[0] = currTime & 0x0000FFFF;
  Telemetry.Write[1] = (currTime >> 16);
  Telemetry.Write[2] = currMode;
  Telemetry.Write[3] = cmdAcceptCount;

  Serial.write(Telemetry.Read, sizeof(Telemetry.Read));

}
// Serial.println(bytes_sent);
/*  Serial.println(Telemetry[0]);
  Serial.println(Telemetry[1]);
  Serial.print("Current Time: ");Serial.available() >0)
  uint32_t x = (uint32_t)Telemetry[1] << 16 | (uint32_t)Telemetry[0];
  Serial.println(x);
  Serial.print("Mode: ");
  Serial.println(Telemetry[2]);
  Serial.print("Command Count: ");
  Serial.println(Telemetry[3]);
  Serial.print("Left Front Distance: ");
  Serial.println(Telemetry[7]);al.print("Current Time: ");
  uint32
  Serial.print("Left Rear Distance: ");
  Serial.println(Telemetry[8]);
  Serial.print("Right Front Distance: ");
  Serial.println(Telemetry[9]);
  Serial.print("Right Rear Distance: ");
  Serial.println(Telemetry[10]);
  Serial.println(currTime);
*/



