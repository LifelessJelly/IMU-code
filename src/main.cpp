#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

std::array<std::uint8_t, 6> broadcastAddress = {0xfc, 0xe8, 0xc0, 0xa2, 0x4f, 0x90};
Adafruit_MPU6050 mpu6050;

class ImuData {
    float accelerationX;
    float accelerationY;
    float accelerationZ;
    char foot;

    // to calculate velocity over time -> (t1-t0) * (a1+a0) / 2
public:
    ImuData(float accelerationX, float accelerationY, float accelerationZ) {
        this->accelerationX = accelerationX;
        this->accelerationY = accelerationY;
        this->accelerationZ = accelerationZ;
        this->foot = 'L';
    }
};

void callbackFunc(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Data send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
}

void setup(){
    Serial.begin(115200);

    delay(100);

    if (!mpu6050.begin(0x68)) {
        Serial.println("Failed to discover MPU6050 sensor");
        return;
    }
    Serial.println("MPU6050 Found!");

    mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(10000);

    Serial.print("Starting WiFi...");

    WiFiClass::mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initialising ESP-NOW");
        return;
    }

    esp_now_register_send_cb(callbackFunc);

    esp_now_peer_info_t peerInfo;

    std::move(broadcastAddress.begin(), broadcastAddress.end(), peerInfo.peer_addr);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

}

void loop(){
    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;
    mpu6050.getEvent(&acceleration, &gyro, &temperature);
    Serial.print(acceleration.acceleration.x);
    Serial.print("X, ");
    Serial.print(acceleration.acceleration.y);
    Serial.print("Y, ");
    Serial.print(acceleration.acceleration.z);
    Serial.println("Z");


    ImuData data {acceleration.acceleration.x, acceleration.acceleration.y, acceleration.acceleration.z};

    esp_now_send(broadcastAddress.data(), reinterpret_cast<std::uint8_t*>(&data), sizeof(data));

}