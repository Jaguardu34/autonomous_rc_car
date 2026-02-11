#include <Servo.h>
#include <NewPing.h>
#include <Adafruit_MPU6050.h>

#define NB_SERVOS 2

Servo servo[NB_SERVOS];

int posservo = 90;
int posservo2 = 90;

bool radioState = false;

const float R1 = 10200.0; // 10.2 kΩ
const float R2 = 2000.0;  // 2 kΩ
const float Vref = 5.0;   // tension de référence Arduino
float battery_voltage = 0.0;
float distance = 0.0;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float tempC;


#define TRIGGER_PIN  6
#define ECHO_PIN     7
#define MAX_DISTANCE 200
#define BATTERY_PIN A1
#define MPU6050_ADDR 0x68


Adafruit_MPU6050 mpu;


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


void setup()
{
    Serial.begin(115200);
    servo[0].attach(10);
    servo[1].attach(11);
    servo[0].write(90);
    servo[1].write(90);
    if (!mpu.begin(MPU6050_ADDR)) {
        Serial.println("MPU6050 not found!");
        while (1);
    }

}

void loop()
{
    ReceiveData();
    updateCaptorDistance();
    updateBatteryJetson();
    updateMPU();
    sendInfo();
    
}


void ReceiveData() {
    static char buffer[32];
    static byte index = 0;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {                 // fin du message
            buffer[index] = '\0';
            parseAndMove(buffer);
            index = 0;
        } 
        else if (index < sizeof(buffer) - 1) {
            buffer[index++] = c;
        }
    }
}


void parseAndMove(char *data) {
    char *token = strtok(data, " ,");
    byte i = 0;

    while (token != NULL && i < NB_SERVOS) {
        int angle = atoi(token);
        servo[i].write(constrain(angle, 0, 180));
        token = strtok(NULL, " ,");
        i++;
    }
}

void updateCaptorDistance() {
  static unsigned long previousMillis = 0;
  const unsigned long interval = 100; // 1 seconde

  
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    int uS = sonar.ping();
    distance = uS / US_ROUNDTRIP_CM;

  }
}

void updateBatteryJetson() {
    int raw = analogRead(BATTERY_PIN);          // lecture ADC (0–1023)
    float V_A0 = raw * (Vref / 1023);  // convertir en volts
    float V_bat_raw = V_A0 * ((R1 + R2) / R2);
    float V_bat = V_bat_raw; // ajustez selon votre diviseur de tension
    battery_voltage = V_bat;

}

void sendInfo(){
    static unsigned long previousMillis = 0;
    const unsigned long interval = 100; // 1 seconde
  
    unsigned long currentMillis = millis();
    
  
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Serial.print(battery_voltage);    
        Serial.print(",");
        Serial.print(accelX);
        Serial.print(",");
        Serial.print(accelY);
        Serial.print(",");
        Serial.print(accelZ);
        Serial.print(",");
        Serial.print(gyroX);
        Serial.print(",");
        Serial.print(gyroY);
        Serial.print(",");
        Serial.print(gyroZ);
        Serial.print(",");
        Serial.print(tempC);
        Serial.print(",");
        Serial.println(distance);
    }

}

void updateMPU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;
    gyroX = g.gyro.x;
    gyroY = g.gyro.y;
    gyroZ = g.gyro.z;
    tempC = temp.temperature;
    // Traitez les données de l'accéléromètre (a), du gyroscope (g) et de la température (temp) selon vos besoins
}







