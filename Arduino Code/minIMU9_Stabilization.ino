#include <Wire.h>
#include <LSM6.h>
#include <BasicLinearAlgebra.h>
#include <Servo.h>

using namespace BLA;

LSM6 imu1; //camera IMU
LSM6 imu2; // base IMU
Servo pitchServo;
Servo yawServo;

// Bias for calibration
int16_t accelBias1[3] = {0, 0, 0};
int16_t gyroBias1[3] = {0, 0, 0};
int16_t accelBias2[3] = {0, 0, 0};
int16_t gyroBias2[3] = {0, 0, 0};

// EKF state and matrices
BLA::Matrix<4> state = {0, 0, 0, 0};   // [pitch, yaw, pitch_rate, yaw_rate]
BLA::Matrix<4, 4> P = {0};            // State covariance
BLA::Matrix<4, 4> F = {0};            // State transition
BLA::Matrix<4, 4> Q = {0.001, 0, 0, 0,
                        0, 0.001, 0, 0,
                        0, 0, 0.001, 0,
                        0, 0, 0, 0.001}; // Process noise
BLA::Matrix<2, 4> H = {1, 0, 0, 0,
                        0, 1, 0, 0};    // Measurement matrix
BLA::Matrix<2, 2> R = {0.1, 0,
                        0, 1};          // Measurement noise

unsigned long lastTime = 0;
float dt = 0.01;  // Time step (assumes 100 Hz)

// Complementary filter constant
float compFilterAlpha = 0.98;
float lastPitch = 0;
float lastYaw = 0;

void calibrateIMU(LSM6 &imu, int16_t accelBias[3], int16_t gyroBias[3], int samples = 1000) {
  long sumAccel[3] = {0, 0, 0};
  long sumGyro[3] = {0, 0, 0};

  Serial.println("Calibrating... Keep the sensor still!");

  for (int i = 0; i < samples; i++) {
    imu.read();

    sumAccel[0] += imu.a.x;
    sumAccel[1] += imu.a.y;
    sumAccel[2] += imu.a.z;

    sumGyro[0] += imu.g.x;
    sumGyro[1] += imu.g.y;
    sumGyro[2] += imu.g.z;

    delay(10);
  }

  for (int i = 0; i < 3; i++) {
    accelBias[i] = sumAccel[i] / samples;
    gyroBias[i] = sumGyro[i] / samples;
  }

  Serial.println("Calibration complete.");
}

void predictEKF(float gyroPitchRate, float gyroYawRate) {
  F = {1, 0, dt, 0,
       0, 1, 0, dt,
       0, 0, 1, 0,
       0, 0, 0, 1};

  state = F * state;
  state(2) = gyroPitchRate;
  state(3) = gyroYawRate;

  P = F * P * ~F + Q;
}

void updateEKF(float pitchAccel, float yawAccel) {
  BLA::Matrix<2> z = {pitchAccel, yawAccel};
  BLA::Matrix<2> y = z - H * state;
  BLA::Matrix<2, 2> S = H * P * ~H + R;
  BLA::Matrix<2, 2> S_inv;
  Invert(S, S_inv);

  BLA::Matrix<4, 4> I = {1, 0, 0, 0,
                         0, 1, 0, 0,
                         0, 0, 1, 0,
                         0, 0, 0, 1};

  BLA::Matrix<4, 2> K = P * ~H * S_inv;
  state = state + K * y;
  P = (I - K * H) * P;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!imu1.init() || !imu2.init()) {
    Serial.println("Failed to detect and initialize IMUs!");
    while (1);
  }

  imu1.enableDefault();
  imu2.enableDefault();

  pitchServo.attach(6);
  yawServo.attach(10);
  pitchServo.write(90);
  yawServo.write(90);

  calibrateIMU(imu1, accelBias1, gyroBias1);
  calibrateIMU(imu2, accelBias2, gyroBias2);
}

void loop() {
  imu1.read();
  imu2.read();

  if (imu1.a.z != 0 && imu2.a.z != 0) {
    unsigned long currentTime = millis();
    dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Remove biases from accelerometer readings
    float pitchAccel1 = atan2(imu1.a.y - accelBias1[1], imu1.a.z - accelBias1[2]) * 180 / PI;
    float yawAccel1 = atan2(imu1.a.x - accelBias1[0], imu1.a.z - accelBias1[2]) * 180 / PI;

    float pitchAccel2 = atan2(imu2.a.y - accelBias2[1], imu2.a.z - accelBias2[2]) * 180 / PI;
    float yawAccel2 = atan2(imu2.a.x - accelBias2[0], imu2.a.z - accelBias2[2]) * 180 / PI;

    // Remove biases from gyroscope readings
    float gyroPitchRate1 = (imu1.g.y - gyroBias1[0]) * 0.07;
    float gyroYawRate1 = (imu1.g.z - gyroBias1[2]) * 0.07;

    float gyroPitchRate2 = (imu2.g.y - gyroBias2[0]) * 0.07;
    float gyroYawRate2 = (imu2.g.z - gyroBias2[2]) * 0.07;

    //combining the two IMUs
    float pitchAccel = pitchAccel - pitchAccel1;
    float yawAccel = (yawAccel1 + yawAccel2) / 2;
    
    float gyroPitchRate = compFilterAlpha * gyroPitchRate1 + (1 - compFilterAlpha) * gyroPitchRate2;
    float gyroYawRate = compFilterAlpha * gyroYawRate1 + (1 - compFilterAlpha) * gyroYawRate2;
    //EKF
    predictEKF(gyroPitchRate, gyroYawRate);
    updateEKF(pitchAccel, yawAccel);
    //Low pass filter
    float filteredPitch = compFilterAlpha * state(0) + (1 - compFilterAlpha) * lastPitch;
    float filteredYaw = compFilterAlpha * state(1) + (1 - compFilterAlpha) * lastYaw;

    lastPitch = filteredPitch;
    lastYaw = filteredYaw;

    int pitchServoAngle = map(filteredPitch * 4.0, -90, 90, 30, 130);
    int yawServoAngle = map(-filteredYaw * 4.0, -90, 90, 20, 140); 

    pitchServoAngle = constrain(pitchServoAngle, 30, 130);
    yawServoAngle = constrain(yawServoAngle, 30, 130);

    pitchServo.write(pitchServoAngle);
    yawServo.write(yawServoAngle);
  } else {
    Serial.println("Invalid accelerometer data");
  }

  delay(10);
}
