#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;  //SDA a4, SCL a5

// Servos
Servo servoBasex, servoBasey, servoCodo, servoPinza;

const int pinBasex = 2;
const int pinBasey = 3;
const int pinCodo  = 4;
const int pinPinza = 5;

// Variables para el Filtro Complementario
float angleX = 0, angleY = 0;
unsigned long prevTime = 0;

// Ajuste de sensibilidad del filtro (0.98 a 0.99 es ideal)
// Da mucha importancia al giroscopio para evitar vibraciones
const float filterAlpha = 0.96; 

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Inicializar MPU6050
  if (!mpu.begin()) {
    Serial.println(F("No se encontro MPU6050"));

    while (1) yield();
  }

  // Configurar rangos para mayor precisión
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Servos
  servoBasex.attach(pinBasex, 500, 2400);
  servoBasey.attach(pinBasey, 500, 2400);
  servoCodo.attach(pinCodo, 500, 2400);
  servoPinza.attach(pinPinza, 500, 2400);

  prevTime = millis();
  Serial.println(F("MPU6050 Listo..."));
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calcular tiempo transcurrido
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // 1. Calcular ángulo desde el Acelerómetro (Inestable por sí solo)
  float accAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float accAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  // 2. Filtro Complementario:
  // Combina la rapidez del Giroscopio con la estabilidad del Acelerómetro
  // Angulo = 96% (Angulo_anterior + Giro * dt) + 4% (Angulo_Acelerometro)
  angleX = filterAlpha * (angleX + (g.gyro.x * 180 / PI) * dt) + (1 - filterAlpha) * accAngleX;
  angleY = filterAlpha * (angleY + (g.gyro.y * 180 / PI) * dt) + (1 - filterAlpha) * accAngleY;

  // 3. Mapeo a grados de Servo (0 a 180)
  // Ajustamos el rango de inclinación de -60/60 a 0/180 para que sea más natural
  int servoXPos = map(angleX, -60, 60, 0, 180);
  int servoYPos = map(angleY, -60, 60, 0, 180);
  
  // Seguridad y suavizado final
  servoXPos = constrain(servoXPos, 10, 170);
  servoYPos = constrain(servoYPos, 10, 170);

  // Mover servos
  servoBasex.write(servoXPos);
  servoBasey.write(servoYPos);
  servoCodo.write(servoYPos); // El codo imita a la base Y en este ejemplo

  // Debugging
  Serial.print(F("AngX: ")); Serial.print(angleX);
  Serial.print(F(" | AngY: ")); Serial.print(angleY);
  Serial.print(F(" | ServoX: ")); Serial.println(servoXPos);

  delay(500); // Mucho más rápido que antes (100Hz aprox) gracias al Giroscopio
}
