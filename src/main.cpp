#include <WiFi.h>
#include <esp_now.h>
#include <queue>
#include <sstream>

#define FLASH_GPIO_NUM 4



class ImuData {
    float accelerationX;
    float accelerationY;
    float accelerationZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    char foot;
    // to calculate velocity over time -> (t1-t0) * (a1+a0) / 2
public:
    ImuData() = default;

    void printData() const {

        std::stringstream ss;
        ss << "X Acceleration: " << accelerationX << ", Y Acceleration: " << accelerationY << ", Z Acceleration: " << accelerationZ << ", Time elapsed: "
        << millis() << ", On " << (foot == 'L' ? "Left" : "Right") << "foot";
        Serial.println(ss.str().c_str());
    }

    bool isLeftLeg() const {
        return foot == 'L';
    }

};

static std::queue<ImuData> left_gait_queue_stack {};
static std::queue<ImuData> right_gait_queue_stack {};

void receiveCallback(const uint8_t * mac, const uint8_t *incomingData, int len) {

    ImuData data {};
    memcpy(&data, incomingData, sizeof(data));
    if (data.isLeftLeg()) {
        left_gait_queue_stack.push(data);

    }
    else {
        right_gait_queue_stack.push(data);
    }
    digitalWrite(FLASH_GPIO_NUM, HIGH);
}

void setup(){

    Serial.begin(115200);
    Serial.println("Gay");
    pinMode (FLASH_GPIO_NUM, OUTPUT);
    WiFiClass::mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initialising ESP-NOW");
        return;
    }
    else {
        Serial.println("ESP-NOW Initialised");
    }
    // who the fuck made the part about receiving data a callback with no user context (billions must use global variables)
    esp_now_register_recv_cb(receiveCallback);

}

void loop(){
    if (!left_gait_queue_stack.empty()) {
        left_gait_queue_stack.front().printData();
        left_gait_queue_stack.pop();
    }
    else {
        Serial.println("No Left Leg Data");
    }
    if (!right_gait_queue_stack.empty()) {
        right_gait_queue_stack.front().printData();
        right_gait_queue_stack.pop();
    }
    else {
        Serial.println("No Right Leg Data");
    }
}