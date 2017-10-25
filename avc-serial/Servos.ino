// TMavc1
// License: GPL v2
// Servo and aux functions

// Default servo definitions
#define SPEED_NEUTRAL 1500  // Servo neutral position
#define SPEED_MIN_PULSEWIDTH 1000
#define SPEED_MAX_PULSEWIDTH 2000
#define DIRECTION_NEUTRAL 1500  // Servo neutral position
#define DIRECTION_MIN_PULSEWIDTH 1000
#define DIRECTION_MAX_PULSEWIDTH 2000


#define BATT_VOLT_FACTOR 1

int battery;


void TMavc1_initServo()
{
  MySpeed.writeMicroseconds(SPEED_NEUTRAL);
  MyDirection.writeMicroseconds(DIRECTION_NEUTRAL);

}

void TMavc1_Speed(int mySpeed)
{
  MySpeed.writeMicroseconds(mySpeed);
}

void TMavc1_Direction(int mydirection)
{
  MyDirection.writeMicroseconds(mydirection);
}

// output : Battery voltage*10 (aprox) and noise filtered
int BROBOT_readBattery(bool first_time)
{
  if (first_time)
  {
    int bat = 0;
    battery = adc->analogRead(motor_Battery) / BATT_VOLT_FACTOR;
  }  else {
    battery = (battery * 9 + (adc->analogRead(motor_Battery) / BATT_VOLT_FACTOR)) / 10;
    return battery;
  }
  return battery;
}

int tmSdistance()
{
  int sDistance;
  //  sDistance = adc->analogRead(distanceRF);
  return sDistance;
}

int tmLdistance()
{
  int lDistance;
  //  lDistance = adc->analogRead(distanceRR);
  return lDistance;
}

void irScan(int degrees)
{
  Serial.println("irScan");
  ir_Scan.runSpeed();
}


