#include <Wire.h>
#include <Arduino.h>

constexpr int MPU = 0x68;
constexpr int RDX = 0x6b;
constexpr int ACCEL_CONFIG = 0x1c;
constexpr int GYRO_CONFIG = 0x1b;


double accX {};
double accY {};
double accZ {};

double gyroX {};
double gyroY {};
double gyroZ {};

double accAngleX {};
double accAngleY {};


double gyroAngleX {};
double gyroAngleY {};

double roll {};
double yaw {};
double pitch {};

double accErrorX {};
double accErrorY {};


double gyroErrorX {};
double gyroErrorY {};
double gyroErrorZ {};

double elapsedTime {};
std::uint64_t currentTime {};
std::uint64_t previousTime {};


void calibrate_imu_err();

void read_all_imu_accel();

void read_all_imu_gyro();

void setup() {
    Serial.begin(115200);
    if (!Wire.begin()) {
        Serial.println("I2C bus cannot be initialised. Check the IMU connection");
        return;
    }
    Wire.beginTransmission(MPU);
    Wire.write(RDX);
    Wire.write(0);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x10);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x10);
    Wire.endTransmission(true);
    delay(20);

    calibrate_imu_err();
    delay(20);

}

void loop() {
    read_all_imu_accel();

    accAngleX = std::atan(accY / std::sqrt(std::pow(accX, 2) + std::pow(accZ, 2))) * 180 / PI - 0.58;
    accAngleY = std::atan(-1 * accX / std::sqrt(std::pow(accY, 2) + std::pow(accZ, 2))) * 180 / PI + 1.58;

    previousTime = currentTime;
    currentTime = millis();

    elapsedTime = static_cast<double>(currentTime - previousTime) / 1000;

    read_all_imu_gyro();

    gyroX += 0.56;
    gyroY -= 2;
    gyroZ += 0.79;

    gyroAngleX += gyroX * elapsedTime;
    gyroAngleY += gyroY * elapsedTime;

    yaw += gyroZ * elapsedTime;
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    Serial.print("Acceleration: ");
    Serial.print(accX);
    Serial.print(", ");
    Serial.print(accY);
    Serial.print(", ");
    Serial.print(accZ);
    Serial.println();

    Serial.print("Rotation: ");
    Serial.print(roll);
    Serial.print("°, ");
    Serial.print(pitch);
    Serial.print("°, ");
    Serial.print(yaw);
    Serial.println();

}

void read_all_imu_accel() {
    Wire.beginTransmission(MPU);
    Wire.write(0x3b);
    Wire.endTransmission(true);
    constexpr std::size_t len {6};
    Wire.requestFrom(static_cast<std::uint8_t>(MPU), len, true);
    accX = static_cast<double>(Wire.read() << 8 | Wire.read()) / (2 << 14);
    accY = static_cast<double>(Wire.read() << 8 | Wire.read()) / (2 << 14);
    accZ = static_cast<double>(Wire.read() << 8 | Wire.read()) / (2 << 14);
}

void read_all_imu_gyro() {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(true);
    constexpr std::size_t len {6};
    Wire.requestFrom(static_cast<std::uint8_t>(MPU), len, true);
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
}

void calibrate_imu_err() {
    for (int i {}; i < 200; ++i) {
        read_all_imu_accel();

        accErrorX += ((std::atan(accY / std::sqrt(std::pow(accX, 2) + std::pow(accZ, 2))) * 180 / PI));
        accErrorY += ((std::atan(-1 * accY / std::sqrt(std::pow(accY, 2) + std::pow(accZ, 2))) * 180 / PI));

    }
    accErrorX /= 200;
    accErrorY /= 200;

    for (int i {}; i < 200; ++i) {

        read_all_imu_gyro();

        gyroErrorX += gyroX / 131;
        gyroErrorY += gyroY / 131;
        gyroErrorZ += gyroZ / 131;

    }
    gyroErrorX /= 200;
    gyroErrorY /= 200;
    gyroErrorZ /= 200;

}

