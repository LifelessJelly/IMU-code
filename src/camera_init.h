//
// Created by josep on 9/14/2024.
//

#ifndef TEST_CAMERA_INIT_H
#define TEST_CAMERA_INIT_H

#include <esp32-hal.h>
#include "esp_camera.h"

#define CAM_Y0_PIN (-1)
#define CAM_Y1_PIN (-1)
#define CAM_Y2_PIN 5
#define CAM_Y3_PIN 18
#define CAM_Y4_PIN 19
#define CAM_Y5_PIN 21
#define CAM_Y6_PIN 36
#define CAM_Y7_PIN 39
#define CAM_Y8_PIN 34
#define CAM_Y9_PIN 35

#define CAMERA_PWR_PIN 32
#define CAMERA_RESET_PIN (-1)
#define CAMERA_CLK_PIN 0
#define SERIAL_IO_CLK_PIN 27
#define SERIAL_IO_DATA_PIN 26
#define VSYNC_PIN 25
#define HREF_IO_PIN 23
#define PIXEL_CLK_PIN 22

#define FLASH_PIN 4

static constexpr camera_config_t camera_config {CAMERA_PWR_PIN,
                                                CAMERA_RESET_PIN,
                                                CAMERA_CLK_PIN,
                                                SERIAL_IO_DATA_PIN,
                                                SERIAL_IO_CLK_PIN,
                                                CAM_Y9_PIN,
                                                CAM_Y8_PIN,
                                                CAM_Y7_PIN,
                                                CAM_Y6_PIN,
                                                CAM_Y5_PIN,
                                                CAM_Y4_PIN,
                                                CAM_Y3_PIN,
                                                CAM_Y2_PIN,
                                                VSYNC_PIN,
                                                HREF_IO_PIN,
                                                PIXEL_CLK_PIN,
                                                20000000,
                                                LEDC_TIMER_0,
                                                LEDC_CHANNEL_0,
                                                PIXFORMAT_JPEG,
                                                FRAMESIZE_UXGA,
                                                8,
                                                2,
                                                CAMERA_FB_IN_PSRAM,
                                                CAMERA_GRAB_LATEST
};

inline esp_err_t init_camera() {
    pinMode(CAMERA_PWR_PIN, OUTPUT);
    digitalWrite(CAMERA_PWR_PIN, LOW);

    const esp_err_t error_code = esp_camera_init(&camera_config);
    if (error_code != ESP_OK) {
        //log some failure stuff

        ESP_LOGE("fail", "Failed Camera Initialisation, check if parameters are correct");
    }
    return error_code;
}

inline esp_err_t camera_capture(camera_fb_t& retriever, unsigned int& length) {
    camera_fb_t* frameBuffer = esp_camera_fb_get(); //JPEG frames start with FF D8 and end with FF D9, anything after FF D9 is just trash
    Serial.println("buffer transfer complete");
    if (frameBuffer == nullptr) {
        ESP_LOGE("fail", "Failed to obtain a frame, did you call init_camera() ?");
        return ESP_FAIL;
    }
    length = frameBuffer->len;
    retriever = *frameBuffer;

    Serial.println("Buffer copy complete");
    esp_camera_fb_return(frameBuffer);
    return ESP_OK;
}

#endif //TEST_CAMERA_INIT_H