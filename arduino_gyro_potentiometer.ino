#include <Wire.h> 
#include <MPU6050.h> 
MPU6050 mpu; 
int16_t ax, ay, az, gx, gy, gz; 
float roll, pitch; 
int throttleValue, rollValue, pitchValue; 
 
void setup() { 
    Serial.begin(115200); 
    Wire.begin(); 
    mpu.initialize(); 
 
    if (!mpu.testConnection()) { 
        Serial.println("MPU6050 connection failed"); 
        while (1); // halt 
    } 
} 
 
void loop() { 
 
    // === Read throttle from potentiometer === 
    int potValue = analogRead(A0); 
    throttleValue = map(potValue, 0, 1023, 1000, 2000); 
 
    // === Read IMU (MPU6050) data === 
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
 
    // === Convert accelerometer data to float === 
    float axf = (float)ax; 
    float ayf = (float)ay; 
    float azf = (float)az; 
 
    // === Calculate roll and pitch angles (in degrees) === 
    roll = atan2(ayf, azf) * 180.0 / PI; 
 
    float denominator = sqrt(ayf * ayf + azf * azf); 
     
    pitch = (denominator == 0) ? 0 : atan2(-axf, denominator) * 180.0 / PI; 
 
    // === Map roll/pitch angles (-90 to +90) to RC range (1000 to 2000) === 
    rollValue = map(constrain((int)roll, -90, 90), -90, 90, 1000, 2000); 
    pitchValue = map(constrain((int)pitch, -90, 90), -90, 90, 1000, 2000); 
 
    // === Send data as CSV: throttle,roll,pitch === 
    Serial.print(throttleValue); Serial.print(","); 
    Serial.print(rollValue); Serial.print(","); 
    Serial.println(pitchValue); 
 
    delay(50); // ~20Hz 
} 