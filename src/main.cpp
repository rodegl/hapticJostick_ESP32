#include <Arduino.h>
#include <Encoder.h>
#include <HBrigde.h>
#include <AnalogSensor.h>
#include <Equations.h>
#include <Filter.h>
#include <PID.h>
#include <math.h>
#include <iostream>
#include <AS5600.h>
#include <Wire.h>

String str = "";
const char separator = ',';
const int dataLength = 14;
double data[dataLength];

PID control_pitch;
float w1 = 0;
PID control_roll;
float w2 = 0;
HBridge pitch_motor;
HBridge roll_motor;
Filter movAvg_A;
Filter movAvg_B;
Filter movAvg_pitch;
Filter movAvg_roll;
Filter movAvg_Aamp;
Filter movAvg_Bamp;

float bArray[] = {0.0674552738890719, 0.134910547778144, 0.0674552738890719};
float aArray[] = {1, -1.14298050253990, 0.412801598096189};

//AS5600
AMS_5600 pitch_sensor;
uint8_t DIR_A = 23;


// MOTOR Pitch
uint8_t IN1_A = 26;
uint8_t IN2_A = 33;
uint8_t PWM_A = 14;

uint8_t EN_A = 19;
uint8_t CS_A = 27;
uint8_t HE_Pitch = 32;
float pitch_angle;
float pitch_pos;

// MOTOR Roll
uint8_t IN1_B = 18;
uint8_t IN2_B = 5;
uint8_t PWM_B = 17;
uint8_t EN_B = 16;
uint8_t CS_B = 13;
uint8_t HE_Roll = 25;
float roll_angle;
float roll_pos;
float pwm_pitch;
float pwm_roll;

// POSITION <------------------------------------------- NUEVO!!! (calculo de posición la punta del joyctick según vista superior)
float pi = 3.14159;
float height = 82.32; // altura del vástago del joystick (mm)
// -------------------------------------------------------------------------------------------------------------------------------------


// Polynomial coefficients <----------- NUEVO!!! (polinomio grado 4 para posición angular a partir de voltage de sensor efecto Hall)
// Pitch
float a1 = 13.7373756808925;
float b1 = -136.9899232967516;
float c1 = 487.8935919079339;
float d1 = -692.1059715498230;
float e1 = 290.1461279023154;
// Roll
float a2 = -22.819635464583;
float b2 = 224.432302710678;
float c2 = -809.268002179282;
float d2 = 1226.850939556437;
float e2 = -631.919838255068;
// -------------------------------------------------------------------------------------------------------------------------------------

float threshold;
float auto_on;
float sumInt_time;
float sample_time;

const uint8_t NUM_SENSORS = 4;
AnalogSensor sensor[NUM_SENSORS];
const uint8_t SENSOR_PIN[NUM_SENSORS] = {HE_Pitch, CS_A, HE_Roll, CS_B};
float coefficients[] = {0.0f, 0.000806f};

ulong prev_Time, prev_Time2, prev_Time3, current_time, current_time2, current_time3;
float cycle_time;

float sign (float value) {

  float signo;
  if (value > 0) {
     signo = 1;
  } else if (value < 0) {
    signo = -1;
  } else {
    signo = 0;
  }

  return signo;
}


void setup()
{
  pinMode(DIR_A, OUTPUT);
  Serial.begin(115200);
  Wire.begin();

  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    sensor[i].setup(SENSOR_PIN[i], polynomial, coefficients, sizeof(coefficients) / sizeof(coefficients[0]));
  }
  pitch_motor.setup(IN1_A, IN2_A, PWM_A, EN_A, 0);
  roll_motor.setup(IN1_B, IN2_B, PWM_B, EN_B, 1);

  delay(1000); // give me time to bring up serial monitor

  movAvg_A.setup(aArray, bArray, 3, 3);
  movAvg_B.setup(aArray, bArray, 3, 3);
  movAvg_pitch.setup(aArray, bArray, 3, 3);
  movAvg_roll.setup(aArray, bArray, 3, 3);
  movAvg_Aamp.setup(aArray, bArray, 3, 3);
  movAvg_Bamp.setup(aArray, bArray, 3, 3);
  control_pitch.setup(0, 0, 0, 0);
  control_pitch.setupSP(0, 0);
  control_roll.setup(0, 0, 0, 0);
  control_pitch.setupSP(0, 0);

  pitch_angle = 0;
  roll_angle = 0;
  auto_on = 0;

  prev_Time = millis();
  prev_Time2 = millis();
  cycle_time = 0;
  sample_time = 0.01;
}

float Angle() {
  digitalWrite(DIR_A, HIGH);
  float in;
  in = map(pitch_sensor.getRawAngle(),3642,2627,-2500,2500);
  return in;
}

void loop()
{
  prev_Time = millis();
  

  // Read data from Serial Port
  if (Serial.available())
  {
    str = Serial.readStringUntil('\n');
    for (int i = 0; i < dataLength; i++)
    {
      int index = str.indexOf(separator);
      data[i] = str.substring(0, index).toFloat();
      str = str.substring(index + 1);
    }
    //PITCH Data
    control_pitch.SetPoint = data[0];
    control_pitch.gainP = data[1];
    control_pitch.gainI = data[2];
    control_pitch.gainD = data[3];
    //ROLL Data
    control_roll.SetPoint = data[4];
    control_roll.gainP = data[5];
    control_roll.gainI = data[6];
    control_roll.gainD = data[7];
    auto_on = data[8];
    threshold = data[9];
    sumInt_time = data[10];
    sample_time = data[11];
    control_pitch.delta2 = data[12];
    control_roll.delta2 = data[13];
  }
  
  current_time3 = millis();

// Ventana tiempo Sumoatorio del Error <----------- NUEVO!!! (resetea la sumatorio del error al tiempo establecido por 'sumInt_time')
  if (current_time3 - prev_Time3 > (sumInt_time * 1000)) // verifica que hayan pasado 500ms
  {
    prev_Time3 = current_time3;
    control_pitch.sumIntegral = 0;
    control_roll.sumIntegral = 0;
  }
// -------------------------------------------------------------------------------------------------------------------------------------
  
  

  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    sensor[i].read();
    sensor[i].getMeasurament(SENSOR_MEASURAMENT);
  }

  //float pitch_amp = movAvg_Aamp.filtering(sensor[1].getMeasurament(SENSOR_MEASURAMENT));

  //float roll_amp = movAvg_Bamp.filtering(sensor[3].getMeasurament(SENSOR_MEASURAMENT));

  //float pitch_volt = movAvg_A.filtering(sensor[0].getMeasurament(SENSOR_MEASURAMENT));
  //float roll_volt = movAvg_B.filtering(sensor[2].getMeasurament(SENSOR_MEASURAMENT));

  //float pitch_volt = sensor[0].getMeasurament(SENSOR_MEASURAMENT);
  float roll_volt = sensor[2].getMeasurament(SENSOR_MEASURAMENT);
  
  
  if (pitch_sensor.detectMagnet() == 1 ) { //----------------------------- Lectura sensor AS5600
      pitch_angle = Angle()/100;
  }


  // PITCH - conversión de lectura de voltaje a posición angular
  //pitch_angle = a1 * (pitch_volt * pitch_volt * pitch_volt * pitch_volt) + b1 * (pitch_volt * pitch_volt * pitch_volt) + c1 * (pitch_volt * pitch_volt) + d1 * pitch_volt + e1;
  // Posición pitch <----------- NUEVO!!! (conversión de ángulo a posición lineal)
  pitch_pos = height * sin((pitch_angle * pi) / 180);

  // ROLL - conversión de lectura de voltaje a posición angular
  roll_angle = a2 * (roll_volt * roll_volt * roll_volt * roll_volt) + b2 * (roll_volt * roll_volt * roll_volt) + c2 * (roll_volt * roll_volt) + d2 * roll_volt + e2;
  // Posición roll <----------- NUEVO!!! (conversión de ángulo a posición lineal)
  roll_pos = height * sin((roll_angle * pi) / 180);

 
  // PITCH CONTROL Output ----------------------------------------------------------------------------------------------------
  //float pitch_output = control_pitch.getOutput(pitch_angle, sample_time);
  //float pitch_output = control_pitch.superTwistingLV(pitch_angle, sample_time, control_pitch.delta2); //Control con Super Twinting
  
  float u1 = control_pitch.gainP;

  float sigma1 =  control_pitch.delta2 + control_pitch.gainD*(pitch_angle - control_pitch.SetPoint); 
  w1 = w1 - sample_time*(1.1*u1)*sign(sigma1);
  float pitch_output = - sqrt(u1)*sqrt(abs(sigma1))*sign(sigma1) + w1;        //Super Twisting

  //float pitch_output_f = movAvg_pitch.filtering(pitch_output);
  pitch_motor.setSpeed(pitch_output);
  
  //ROLL CONTROL Output ----------------------------------------------------------------------------------------------------
  //float roll_output = control_roll.getOutput(roll_angle, sample_time);
  //float roll_output = control_roll.superTwistingLV(roll_angle, sample_time, control_roll.delta2); //Control con Super Twinting
  
  float u2 = control_roll.gainP;

  float sigma2 =  control_roll.delta2 + control_roll.gainD*(roll_angle - control_roll.SetPoint); 
  w2 = w2 - sample_time*(1.1*u2)*sign(sigma2);
  float roll_output = - sqrt(u2)*sqrt(abs(sigma2))*sign(sigma2) + w2;

  //float roll_output_f = movAvg_roll.filtering(roll_output);
  roll_motor.setSpeed(roll_output);

  

  //HAPTIC FEEDBACK

  /* if (auto_on == 1)
  {
    if (abs(pitch_angle) > threshold)
    {
      control_pitch.gainP = 0.7f;
      control_pitch.gainD = 0.01f;
      control_pitch.SetPoint = 0;
    }

    if (abs(roll_angle) > threshold)
    {
      control_roll.gainP = 0.7f;
      control_roll.gainD = 0.01f;
      control_roll.SetPoint = 0;
    }

    // if (sqrtf((pitch_angle * pitch_angle) * (roll_angle * roll_angle)) > threshold)
    // {
    //   control_pitch.gainP = 0.7f;
    //   control_pitch.gainD = 0.01f;
    //   control_pitch.SetPoint =  0;

    //   control_roll.gainP = 0.7f;
    //   control_roll.gainD = 0.01f;
    //   control_roll.SetPoint =  0;
    // }
  } */

  current_time2 = micros();
 
  if (current_time2 - prev_Time2 > (sample_time * 1000)) // verifica que hayan pasado 10ms
  {
    
    
    prev_Time2 = current_time2;
    // SERIAL PORT WRITE
  
    Serial.printf("%.2f, %.2f, %.2f, %.2f, ", control_pitch.SetPoint, control_roll.SetPoint, pitch_output, roll_output);
    Serial.printf("%.2f, %.2f,", pitch_pos, roll_pos);
    Serial.printf("%.2f, %.2f, %.2f, %.2f,", pitch_angle, roll_angle, control_pitch.error, control_roll.error);
    Serial.printf("%.2f \n", cycle_time);
    

  }
  
  while ((millis()-prev_Time) < (sample_time*1000)){
  }

  cycle_time = millis() - prev_Time;

  // -----------------------------------------------------------------------------------------------------
}