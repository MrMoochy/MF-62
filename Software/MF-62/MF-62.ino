#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

int PIN_POTENCIOMETRO = A2;
int PIN_JOYSTICK_X = A1;
int PIN_JOYSTICK_Y = A0;
#define PIN_SERVO_IZQUIERDA 6
#define PIN_SERVO_DERECHA 7
#define PIN_SERVO_ATRAS 8
#define PIN_MOTOR 9
#define DEBUG 0
#define PITCH_OFFSET 4.40
#define ROLL_OFFSET 6.0

float Kp = 1.50;
float Ki = 0.25;
float Kd = 1;

float pitch_setpoint = 0;
float roll_setpoint = 0;

float integral_pitch = 0;
float integral_roll = 0;

float error_anterior_pitch = 0; 
float error_anterior_roll = 0;

float p = 0;
float r = 0;

unsigned long tiempo_anterior = 0;
unsigned long last_time = 0;

struct JoystickPad 
{ 
  int x; 
  int y; 
};

JoystickPad joystick;

Servo servoIzquierda;
Servo servoDerecha;
Servo servoAtras;
Servo motorBrushless;

Adafruit_MPU6050 mpu;

float acc_pitch_input, acc_roll_input;
float gyro_pitch_input, gyro_roll_input;

bool esc_armado = false;
unsigned long tiempo_inicio_esc;


void setup()
{
  Serial.begin(115200);

  servoIzquierda.attach(PIN_SERVO_IZQUIERDA);
  servoDerecha.attach(PIN_SERVO_DERECHA);
  servoAtras.attach(PIN_SERVO_ATRAS);
  motorBrushless.attach(PIN_MOTOR);

  if(!mpu.begin())
  {
    Serial.println("MPU no responde :( ");
    while(1);
  }

  motorBrushless.writeMicroseconds(1000);

  mpu.setMotionDetectionThreshold(20);
  mpu.setMotionDetectionDuration(40);
  mpu.setMotionInterrupt(true);
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);


  if(!DEBUG)   // Centramos servos
  {
    servoIzquierda.write(90);
    servoDerecha.write(90);
    servoAtras.write(90);
  }

  tiempo_inicio_esc = millis();
  tiempo_anterior = millis();
  last_time = micros();
}

void loop()
{
  leerBotones();
  controlBrushless();
 
  if(comprobarPuntoMuerto(joystick.x, joystick.y)) //verificamos que hayamos efectuado un control manual
  {
    if(mpu.getMotionInterruptStatus()) //verificamos que el MPU se haya movido
    {
      float pitch, roll;
      leerMPU(&pitch, &roll); //tomamos valores del MPU
      controlPID(pitch, roll); //efectuamos la correcion con PID
    }
  }
  else
  {
    controlManualServos(); //efectuamos los movimientos manualmente
  }
}

void leerBotones()
{
  joystick.x = analogRead(PIN_JOYSTICK_X);
  joystick.y = analogRead(PIN_JOYSTICK_Y);
}

void controlBrushless()
{
  if(!esc_armado)
  {
    if(millis() - tiempo_inicio_esc > 3000)
    {
      esc_armado = true;
    }
    motorBrushless.writeMicroseconds(1000);
    return;
  }

  int lectura = analogRead(PIN_POTENCIOMETRO);
  int potencia = map(lectura, 0, 1023, 1000, 2000);

  potencia = constrain(potencia, 1000, 2000);
  motorBrushless.writeMicroseconds(potencia);
}

bool comprobarPuntoMuerto(int x, int y)
{
  return(x > 510 && x < 530 && y > 510 && y < 530);
}

void controlManualServos()
{
  int roll  = map(joystick.x, 0, 1023, -30, 30);
  int pitch = map(joystick.y, 0, 1023, -20, 20);

  int centro = 90;

  int angulo_izquierdo = centro + roll;
  int angulo_derecho = centro - roll;

  angulo_izquierdo = constrain(angulo_izquierdo, 0, 180);
  angulo_derecho = constrain(angulo_derecho, 0, 180);

  servoIzquierda.write(angulo_izquierdo);
  servoDerecha.write(angulo_derecho);

  servoAtras.write(90 + pitch);
}

void leerMPU(float *pitch_out, float *roll_out) 
{
  sensors_event_t acelerometro, giroscopio, temperatura;
  mpu.getEvent(&acelerometro, &giroscopio, &temperatura);

  unsigned long now = micros();
  float dt = (now - last_time) / 1e6;
  last_time = now;

  float acc_roll  = atan2(acelerometro.acceleration.y, acelerometro.acceleration.z) * 180 / PI;
  float acc_pitch = atan(-acelerometro.acceleration.x /
                        sqrt(acelerometro.acceleration.y * acelerometro.acceleration.y +
                             acelerometro.acceleration.z * acelerometro.acceleration.z)) * 180 / PI;

  p += giroscopio.gyro.y * dt * 180 / PI;
  r  += giroscopio.gyro.x * dt * 180 / PI;

  const float alpha = 0.98;

  p = alpha * p + (1 - alpha) * acc_pitch;
  r  = alpha * r  + (1 - alpha) * acc_roll;

  *pitch_out = p + PITCH_OFFSET;
  *roll_out  = r + ROLL_OFFSET;
}

void controlPID(float pitch_input, float roll_input)
{
  /* 
  error = setpoint – input;
  integral += error * dt;
  derivative = (error – error_anterior) / dt;
  output = Kp*error + Ki*integral + Kd*derivative;
  */
  unsigned long ahora = millis();
  float dt = (ahora - tiempo_anterior) / 1000.0;
  tiempo_anterior = ahora;

  float error_pitch = pitch_setpoint - pitch_input;
  float error_roll  = roll_setpoint  - roll_input;

  integral_pitch += error_pitch * dt;
  integral_roll  += error_roll * dt;

  integral_pitch = constrain(integral_pitch, -20, 20);
  integral_roll = constrain(integral_roll, -20, 20);

  float derivada_pitch = (error_pitch - error_anterior_pitch) / dt;
  float derivada_roll  = (error_roll  - error_anterior_roll) / dt;

  float pitch_output = Kp * error_pitch + Ki * integral_pitch + Kd * derivada_pitch;
  float roll_output  = Kp * error_roll  + Ki * integral_roll  + Kd * derivada_roll;

  error_anterior_pitch = error_pitch;
  error_anterior_roll  = error_roll;

  int servo_pitch = map(pitch_output, -45, 45, 0, 90);
  int servo_roll  = map(roll_output , -45, 45, 0, 90);
  
  servo_pitch = constrain(servo_pitch, 0, 90);
  servo_roll  = constrain(servo_roll , 0, 90);

  if(DEBUG)
  {
    Serial.print("Servo pitch: "); Serial.println(servo_pitch);
    Serial.print("Servo roll: "); Serial.println(servo_roll);
  }
  else
  {
    servoIzquierda.write(servo_roll);
    servoDerecha.write(180 - servo_roll);
    servoAtras.write(servo_pitch);
  }
}

