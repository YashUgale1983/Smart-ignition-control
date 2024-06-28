#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"

const char* ssid = "xxxx";
const char* password = "xxxx";

String serverName = "helmet-vision-api.onrender.com";   // Updated serverName
String serverPath = "/detectLabels";     // Updated serverPath

const int serverPort = 443; //server port for HTTPS

const int relayPin = 2;
int helmetDetectedCount = 0;
int helmetNotDetectedCount = 0;
bool engineFlag = false;
#define FLASH_GPIO_NUM 4

WiFiClientSecure client;

// CAMERA_MODEL_AI_THINKER
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

const int timerInterval = 5000;    // time between each HTTP POST image
unsigned long previousMillis = 0;   // last time image was sent
int consecutiveNotDetectedCount = 0; // Counter for consecutive "Helmet not detected" messages

const char* twilioSID = "xxxxxx";
const char* twilioAuthToken = "xxxxxx";
const char* twilioPhoneNumber = "+xxxxxxxxxxx";
const char* recipientPhoneNumber = "+91xxxxxxxxxx";

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);

  pinMode(relayPin, OUTPUT);
  pinMode(FLASH_GPIO_NUM, OUTPUT);

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  sendPhoto(); 
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval) {
    sendPhoto();
    previousMillis = currentMillis;
  }
}

String sendPhoto() {
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    flashLED(2);
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("Connecting to server: " + serverName);
  
  client.setInsecure(); //skip certificate validation
  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");    
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"image\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
  
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println();
    client.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }   
    client.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }  
    }
    Serial.println();
    client.stop();
    Serial.println(getBody);






    if (getBody.indexOf("Helmet detected") != -1) {
      if (!engineFlag) {
        helmetDetectedCount++;
        if (helmetDetectedCount == 1) {
          digitalWrite(relayPin, HIGH);
          engineFlag = true;
          Serial.print("relay pin set high and engine flag set true");
          flashLED(3);
          delay(10000);
          digitalWrite(relayPin, LOW);
        } else {
          // helmetDetectedCount++;
          // Serial.print("helmet detected count incremented");
        }
      }else{
        digitalWrite(relayPin, HIGH);
        delay(10000);
        digitalWrite(relayPin, LOW);
        // first turned on then off then again try to turn on
      }
    } else if (getBody.indexOf("Helmet not detected") != -1) {
      if (engineFlag) {
        if (helmetNotDetectedCount == 5) {
          sendSMS();
          helmetNotDetectedCount = 0;
          Serial.print("sms sent and reset helmetNotDetectedCount");
          
        } else {
          helmetNotDetectedCount++;
          Serial.print("helmet not detected count incremented");
        }
      } else {
        digitalWrite(relayPin, LOW);
        engineFlag = false;
        helmetDetectedCount=0;
        Serial.print("relay pin set low and engine flag is false");
      }
    }
















    

    // // Check if "Helmet detected" message is received
    // if (getBody.indexOf("Helmet detected") != -1) {
    //   // Reset consecutiveNotDetectedCount
    //   consecutiveNotDetectedCount = 0;

    //   // digitalWrite(relayPin, HIGH); // if helmet detection, relay pin turned high
    //   // delay(10000);
    //   // digitalWrite(relayPin, LOW);
    //   // delay(10000);
    // }
    // // Check if "Helmet not detected" message is received
    // else if (getBody.indexOf("Helmet not detected") != -1) {
    //   // Increment counter
    //   consecutiveNotDetectedCount++; 
    // }

    // // Check if "Helmet not detected" has occurred consecutively for 5 times
    // if (consecutiveNotDetectedCount == 3) {
    //   Serial.println("Sending alert message");
    //   sendSMS(); // Call the sendSMS() function when consecutiveNotDetectedCount reaches 5
    //   // Reset the counter
    //   consecutiveNotDetectedCount = 0;
    // }
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    flashLED(1);
    delay(1000);
    Serial.println(getBody);
  }
  return getBody;
}

void sendSMS() {
  HTTPClient http;

  // Prepare the Twilio API URL
  String url = "https://api.twilio.com/2010-04-01/Accounts/";
  url += twilioSID;
  url += "/Messages.json";

  // Prepare the message data
  String data = "To=";
  data += recipientPhoneNumber;
  data += "&From=";
  data += twilioPhoneNumber;
  data += "&Body=Helmet%20not%20detected%20for%205%20consecutive%20times";

  http.begin(url);
  http.setAuthorization(twilioSID, twilioAuthToken);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  int httpCode = http.POST(data);
  String payload = http.getString();

  Serial.print("HTTP Code: ");
  Serial.println(httpCode);
  Serial.println("Response:");
  Serial.println(payload);

  http.end();
}


void flashLED(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(FLASH_GPIO_NUM, HIGH);
    delay(500);
    digitalWrite(FLASH_GPIO_NUM, LOW);
    delay(500);
  }
}
