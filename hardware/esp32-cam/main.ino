#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <base64.h>

// Wi-Fi credentials
const char* ssid = "Realme GT Neo 3";
const char* password = "qwertyuiop1";

// REST API endpoint
const char* serverUrl = "http://192.168.240.119:5000/upload";

// Camera pins for AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// FPS control
unsigned long previousMillis = 0;
const long frameInterval = 200; // 200ms = 5fps
boolean readyForNextFrame = true;

// FPS calculation
unsigned long fpsCounterTime = 0;
unsigned int frameCount = 0;
float fps = 0;

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  // WiFi setup
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to WiFi, IP address: ");
  Serial.println(WiFi.localIP());
  
  // Camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA; // 800x600 - Use SVGA
    config.jpeg_quality = 15;  // 0-63, lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA; // 640x480 if no PSRAM
    config.jpeg_quality = 20;
    config.fb_count = 1;
  }
  
  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  // Set camera parameters
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_SVGA);
  s->set_quality(s, 15);
  s->set_brightness(s, 1);
  s->set_contrast(s, 1);
  
  // Initialize FPS counter
  previousMillis = millis();
  fpsCounterTime = millis();
}

void sendImageToServer(camera_fb_t *fb) {
  // Convert frame to base64
  String base64Image = base64::encode(fb->buf, fb->len);
  
  // Create HTTP client
  HTTPClient http;
  
  // Begin HTTP connection
  http.begin(serverUrl);
  
  // Set content type
  http.addHeader("Content-Type", "application/json");
  
  // Create JSON payload
  String payload = "{\"image\":\"" + base64Image + "\"}";
  
  Serial.printf("Image size: %d bytes, Base64 size: %d bytes\n", fb->len, base64Image.length());
  
  // Send the POST request
  int httpResponseCode = http.POST(payload);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.printf("HTTP Response code: %d\n", httpResponseCode);
    Serial.println(response);
  } else {
    Serial.printf("HTTP POST Error: %d\n", httpResponseCode);
  }
  
  // End HTTP connection
  http.end();
  
  // Set ready flag for next capture after brief delay
  delay(50);
  readyForNextFrame = true;
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Capture frame at our target interval (5fps) if ready for next frame
  if (currentMillis - previousMillis >= frameInterval && readyForNextFrame) {
    previousMillis = currentMillis;
    readyForNextFrame = false; // Block until this frame is processed
    
    // Calculate FPS every second
    frameCount++;
    if (currentMillis - fpsCounterTime >= 1000) {
      fps = frameCount / ((currentMillis - fpsCounterTime) / 1000.0);
      Serial.printf("Current FPS: %.1f\n", fps);
      frameCount = 0;
      fpsCounterTime = currentMillis;
    }
    
    // Capture frame
    camera_fb_t *fb = esp_camera_fb_get();
    
    if (!fb) {
      Serial.println("Camera capture failed");
      readyForNextFrame = true; // Reset flag to try again
      return;
    }
    
    // Send the image to the REST API server
    sendImageToServer(fb);
    
    // Release the frame buffer
    esp_camera_fb_return(fb);
  }
}
