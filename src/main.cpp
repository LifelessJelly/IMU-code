#include <WiFi.h>
#include <esp_now.h>
#include <sstream>
#include "camera_init.h"
#include <Adafruit_MLX90614.h>
#include <ArduinoJson.h>
#include <azure_funcs.h>
#include <esp_wifi.h>
#include <base64.h>

#define FLASH_GPIO_NUM 4

class ImuData {
public:
    float accelerationY;
    std::uint64_t cadence;
    std::double_t strideLength;
    char foot;
    // to calculate velocity over time -> (t1-t0) * (a1+a0) / 2

    ImuData() = default;

    bool isLeftLeg() const {
        return foot == 'L';
    }

};

ImuData leftLeg;
ImuData rightLeg;
String encoded_str;
std::double_t body_temp;

auto mlx = Adafruit_MLX90614();

void receiveCallback(const uint8_t * mac, const uint8_t *incomingData, int len) {

    ImuData data {};
    memcpy(&data, incomingData, sizeof(data));
    if (data.isLeftLeg()) {
        leftLeg = data;
    }
    else {
        rightLeg = data;
    }

    camera_fb_t *frameBuffer = esp_camera_fb_get();
    if (frameBuffer == nullptr) {
        ESP_LOGE("fail", "Failed to obtain a frame, did you call init_camera() ?");
        return;
    }

    encoded_str = base64::encode(frameBuffer->buf, frameBuffer->len);

    Serial.println(encoded_str.c_str());
    Serial.println(encoded_str.length());

    esp_camera_fb_return(frameBuffer);

    body_temp = mlx.readObjectTempC();
}

void setup(){

    Serial.begin(115200);
    Serial.println("Gay");
    init_camera();


    if (mlx.begin()) {
        Serial.println("Error connecting to MLX sensor");
    }
    Serial.print("Emissivity setting: ");
    Serial.println(mlx.readEmissivity());

    pinMode (FLASH_GPIO_NUM, OUTPUT);
    WiFiClass::mode(WIFI_STA);
    establishConnection();
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initialising ESP-NOW");
        return;
    }
    Serial.println("ESP-NOW Initialised");
    // who the fuck made the part about receiving data a callback with no user context (billions must use global variables)
    esp_now_register_recv_cb(receiveCallback);

}

void loop(){

    if (WiFiClass::status() != WL_CONNECTED)
    {
        connectToWiFi();
    }
#ifndef IOT_CONFIG_USE_X509_CERT
    else if (sasToken.IsExpired())
    {
        Logger.Info("SAS token expired; reconnecting with a new one.");
        (void)esp_mqtt_client_destroy(mqtt_client);
        initializeMqttClient();
    }
#endif
    else if (millis() > next_telemetry_send_time_ms)
    {
        JsonDocument jsonStr {};

        const auto leg_data_ref = jsonStr["leg_data"];

        leg_data_ref["upward_accelerationL"] = leftLeg.accelerationY;
        leg_data_ref["cadenceL"] = leftLeg.cadence;
        leg_data_ref["strideLengthL"] = leftLeg.strideLength;

        leg_data_ref["upward_accelerationR"] = rightLeg.accelerationY;
        leg_data_ref["cadenceR"] = rightLeg.cadence;
        leg_data_ref["strideLengthR"] = rightLeg.strideLength;

        jsonStr["temperature"] = body_temp;
        jsonStr["camera_base64"] = encoded_str;

        jsonStr["timestamp"] = millis();

        std::string payload = "{}";

        serializeJson(jsonStr, payload);
        //
        // if (esp_mqtt_client_publish(mqtt_client, telemetry_topic, payload.c_str(), 0, MQTT_QOS1, DO_NOT_RETAIN_MSG) == 0) {
        //     Serial.println("Failed to send :(");
        // }
        // else {
        //     Serial.println("Send successful!");
        // }
        next_telemetry_send_time_ms = millis() + TELEMETRY_FREQUENCY_MILLISECS;
    }
    //takes the latest frame of the IMU readings (because the camera is the slowest among all of them)


}