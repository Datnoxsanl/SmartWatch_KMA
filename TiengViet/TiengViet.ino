#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <RTClib.h>
#include <U8g2lib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <ArduinoJson.h>
#include <Audio.h>
#include <esp_heap_caps.h>
#include <time.h>
// WiFi & APIs
const char* ssid = "Fire And Blood";
const char* password = "12345678";
const char* viettel_api = "https://viettelai.vn/asr/recognize";
const char* viettel_token = "6622b17dc7d8528f5b6d31ee27d40b5c";
const char* weather_api = "http://api.openweathermap.org/data/2.5/weather?lat=21.0285&lon=105.8542&appid=f5044cf6ac87465b7e240e4cbbf7d3f2&units=metric";

// Thêm vào đầu file, sau các #include
const char* forecast_api = "http://api.openweathermap.org/data/2.5/forecast?lat=21.0285&lon=105.8542&appid=f5044cf6ac87465b7e240e4cbbf7d3f2&units=metric";

struct DailyForecast {
  float temp;
  String desc;
  uint32_t timestamp;
  float pop;  // Xác suất mưa (0.0 đến 1.0)
};

DailyForecast forecastData[3];
SemaphoreHandle_t forecastSemaphore;
#define SUN 0
#define SUN_CLOUD 1
#define CLOUD 2
#define RAIN 3
#define THUNDER 4
const char* daysOfWeek[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
// I2S Microphone (INMP441 with new I2S driver)
#define I2S_WS 41
#define I2S_SCK 42
#define I2S_SD 40
#define RATE 16000
#define CHUNK 1024
#define RECORD_SECONDS 3

// I2S Speaker (MAX98357)
#define I2S_SPK_BCLK 5
#define I2S_SPK_LRC 4
#define I2S_SPK_DOUT 15

// WS2812B
#define LED_PIN 48
#define NUM_LEDS 1
Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// OLED
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 13, 12);

// RTC
RTC_DS1307 rtc;

// MAX7219 pins
int DataOUT = 21;
int CLK = 47;
int Load = 20;

// Buzzer
#define BUZZER_PIN 14
// Shared variables with semaphores

// Biến chia sẻ với semaphore
String currentCommand = "Chua co lenh";
String weatherInfo = "Weather: N/A";
SemaphoreHandle_t commandSemaphore;
SemaphoreHandle_t weatherSemaphore;

// Biến cho báo thức, hẹn giờ, và hẹn giờ 5 phút
bool alarmEnabled = false;
int alarmHour = -1;
int alarmMinute = 0;
bool timerEnabled = false;
uint32_t timerEndTime = 0;
bool timer5MinEnabled = false;
uint32_t timer5MinEnd = 0;
SemaphoreHandle_t displaySemaphore;

// 4 chữ số để hiển thị
int Digit3 = 0;
int Digit2 = 0;
int Digit1 = 0;
int Digit0 = 0;

struct WeatherIcon {
  uint16_t code;
  const char* font;
};

// Task handles
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t uartTaskHandle = NULL;

// UART1 configuration
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 17
#define UART_RX_PIN 18
#define UART_BAUD_RATE 115200

bool isOnline = true;                                     // Trạng thái Online/Offline
bool lastWiFiState = false;                               // Lưu trạng thái WiFi trước đó
bool timer2MinEnabled = false;                            // 2-minute timer
uint32_t timer2MinEnd = 0;                                // 2-minute timer end time
bool displayMode2 = false;                                // Track OLED display mode
TickType_t lastWiFiCheck = 0;                             // Thời điểm kiểm tra WiFi cuối
const TickType_t wifiDebounceTime = pdMS_TO_TICKS(5000);  // Debounce 5 giây

// I2S channel handle for microphone
i2s_chan_handle_t mic_rx_chan = NULL;

// Global Audio object for speaker
Audio* audio = NULL;

// Flag to track I2S status
bool micI2SInitialized = false;

// Bảng mã cho số 0-9 cho seg0 (LED 1)
const byte digits_seg0[10] = {
  0b11111100,  // 0
  0b00110000,  // 1
  0b11101001,  // 2
  0b01111001,  // 3
  0b00110101,  // 4
  0b01011101,  // 5
  0b11011101,  // 6
  0b01110000,  // 7
  0b11111101,  // 8
  0b01111101   // 9
};

// Bảng mã cho số 0-9 cho seg1 (LED 2)
const byte digits_seg1[10] = {
  0b11111101,  // 0
  0b00110001,  // 1
  0b11101011,  // 2
  0b01111011,  // 3
  0b00110111,  // 4
  0b01011111,  // 5
  0b11011111,  // 6
  0b01110001,  // 7
  0b11111111,  // 8
  0b01111111   // 9
};

// Bảng mã cho số 0-9 cho seg2 (LED 3)
const byte digits_seg2[10] = {
  0b11111101,  // 0
  0b10000101,  // 1
  0b11101011,  // 2
  0b11001111,  // 3
  0b10010111,  // 4
  0b01011111,  // 5
  0b01111111,  // 6
  0b10001101,  // 7
  0b11111111,  // 8
  0b11011111   // 9
};

// Bảng mã cho số 0-9 cho seg3 (LED 4)
const byte digits_seg3[10] = {
  0b11111100,  // 0
  0b10000100,  // 1
  0b11101010,  // 2
  0b11001110,  // テキスト
  0b10010110,  // 4
  0b01011110,  // 5
  0b01111110,  // 6
  0b10001100,  // 7
  0b11111110,  // 8
  0b11011110   // 9
};
void enableRTCBatteryMode() {
  Wire.beginTransmission(0x68);  // Địa chỉ DS1307
  Wire.write(0x00);              // Thanh ghi giây
  Wire.endTransmission();

  Wire.requestFrom(0x68, 1);
  uint8_t seconds = Wire.read();

  // Bật bộ dao động (CH = 0)
  if (seconds & 0x80) {  // Nếu bit CH = 1, đồng hồ bị dừng
    Wire.beginTransmission(0x68);
    Wire.write(0x00);
    Wire.write(seconds & 0x7F);  // Xóa bit CH để bật đồng hồ
    Wire.endTransmission();
    Serial.println("Đã bật bộ dao động DS1307");
  }
}

void setupI2S_new_driver() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &mic_rx_chan);
  if (err != ESP_OK) {
    mic_rx_chan = NULL;
    return;
  }
  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_SCK,
      .ws = (gpio_num_t)I2S_WS,
      .dout = I2S_GPIO_UNUSED,
      .din = (gpio_num_t)I2S_SD,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;  // INMP441 outputs on left channel
  err = i2s_channel_init_std_mode(mic_rx_chan, &std_cfg);
  if (err != ESP_OK) {
    i2s_del_channel(mic_rx_chan);
    mic_rx_chan = NULL;
    return;
  }

  err = i2s_channel_enable(mic_rx_chan);
  if (err != ESP_OK) {
    i2s_del_channel(mic_rx_chan);
    mic_rx_chan = NULL;
    return;
  }
  micI2SInitialized = true;
}

void cleanupI2S_new_driver() {
  if (mic_rx_chan != NULL) {
    esp_err_t disable_err = i2s_channel_disable(mic_rx_chan);
    if (disable_err != ESP_OK) {
    }
    esp_err_t del_err = i2s_del_channel(mic_rx_chan);
    if (del_err != ESP_OK) {
    }
    mic_rx_chan = NULL;
  }
  micI2SInitialized = false;
}

bool recordAudio_new_driver(uint8_t** outBuffer, size_t* outLength) {
  if (!micI2SInitialized || mic_rx_chan == NULL) {
    return false;
  }

  Serial.printf("Free heap trước khi cấp phát PCM: %d bytes\n", ESP.getFreeHeap());
  size_t pcmSize = RATE * RECORD_SECONDS * 2;
  int numChunks = (pcmSize + CHUNK - 1) / CHUNK;
  pcmSize = numChunks * CHUNK;

  uint8_t* pcmData = (uint8_t*)heap_caps_malloc(pcmSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!pcmData) {
    Serial.println("Lỗi: Không đủ bộ nhớ PSRAM cho dữ liệu PCM!");
    Serial1.write('c');
    return false;
  }

  size_t offset = 0;
  int readErrorCount = 0;
  const int maxReadErrors = 3;  // Ngưỡng lỗi liên tiếp
  Serial.println("Bắt đầu thu âm với driver mới...");
  for (int i = 0; i < numChunks; i++) {
    size_t bytesRead = 0;
    esp_err_t result = i2s_channel_read(mic_rx_chan, pcmData + offset, CHUNK, &bytesRead, pdMS_TO_TICKS(750));
    if (result != ESP_OK) {
      heap_caps_free(pcmData);
      return false;
    }
    if (bytesRead != CHUNK) {
      readErrorCount++;
      if (readErrorCount >= maxReadErrors) {
        heap_caps_free(pcmData);
        return false;
      }
    } else {
      readErrorCount = 0;  // Reset lỗi nếu đọc thành công
    }
    offset += bytesRead;
  }
  Serial.println("Thu âm hoàn tất.");

  *outBuffer = pcmData;
  *outLength = offset;
  Serial.printf("Free heap sau khi thu âm: %d bytes\n", ESP.getFreeHeap());
  return true;
}

void playTTS(const char* text, const char* lang) {
  if (!audio) {
    return;
  }

  Serial.printf("Bắt đầu phát TTS: %s, lang: %s\n", text, lang);
  Serial.printf("Free heap trước TTS: %d bytes\n", ESP.getFreeHeap());

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Lỗi: WiFi không kết nối, không thể phát TTS");
    return;
  }

  audio->setPinout(I2S_SPK_BCLK, I2S_SPK_LRC, I2S_SPK_DOUT, I2S_NUM_1);
  audio->setVolume(80);
  if (!audio->connecttospeech(text, lang)) {
    Serial.println("Lỗi: Không thể kết nối đến dịch vụ TTS");
    return;
  }

  Serial.printf("Free heap sau connecttospeech: %d bytes\n", ESP.getFreeHeap());
  while (audio->isRunning()) {
    audio->loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  audio->stopSong();  // Dừng để tránh xung đột
  Serial.println("TTS hoàn tất.");
  Serial.printf("Free heap sau TTS: %d bytes\n", ESP.getFreeHeap());
}
void fetchForecast() {
  HTTPClient http;
  http.begin(forecast_api);
  int httpResponseCode = http.GET();

  if (httpResponseCode == 200) {
    String payload = http.getString();
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.println("Lỗi phân tích JSON dự báo: " + String(error.c_str()));
      if (xSemaphoreTake(forecastSemaphore, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 3; i++) {
          forecastData[i].desc = "Error";
          forecastData[i].pop = 0.0;
          forecastData[i].temp = 0.0;
          forecastData[i].timestamp = 0;
        }
        xSemaphoreGive(forecastSemaphore);
      }
      http.end();
      return;
    }

    JsonArray list = doc["list"];
    int dayCount = 0;
    DateTime now = rtc.now();
    uint32_t todayMidnight = now.unixtime() - (now.hour() * 3600) - (now.minute() * 60) - now.second();

    // Lưu trữ mục gần nhất với 12:00 cho mỗi ngày
    struct ForecastCandidate {
      uint32_t timestamp;
      float temp;
      String desc;
      float pop;
      int64_t timeDiff;
    };
    ForecastCandidate candidates[3] = {
      { 0, 0.0, "N/A", 0.0, INT64_MAX },
      { 0, 0.0, "N/A", 0.0, INT64_MAX },
      { 0, 0.0, "N/A", 0.0, INT64_MAX }
    };

    for (JsonObject item : list) {
      uint32_t dt = item["dt"];
      struct tm timeinfo;
      time_t rawtime = dt;
      localtime_r(&rawtime, &timeinfo);

      // Xác định ngày dự báo (1, 2, 3 ngày tiếp theo)
      for (int i = 0; i < 3; i++) {
        uint32_t targetMidnight = todayMidnight + (i + 1) * 86400;
        uint32_t targetNoon = targetMidnight + 12 * 3600;  // 12:00 ngày tiếp theo
        if (dt >= targetMidnight && dt < targetMidnight + 86400) {
          // Tính khoảng cách thời gian đến 12:00
          int64_t timeDiff = (int64_t)dt - (int64_t)targetNoon;
          if (timeDiff < 0) timeDiff = -timeDiff;  // Lấy giá trị tuyệt đối

          // Nếu gần 12:00 hơn mục hiện tại
          if (timeDiff < candidates[i].timeDiff) {
            candidates[i].timestamp = dt;
            candidates[i].temp = item["main"]["temp"];
            candidates[i].desc = item["weather"][0]["description"].as<String>();
            candidates[i].pop = item["pop"];
            candidates[i].timeDiff = timeDiff;
          }
        }
      }
    }

    // Cập nhật forecastData
    if (xSemaphoreTake(forecastSemaphore, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 3; i++) {
        forecastData[i].timestamp = candidates[i].timestamp;
        forecastData[i].temp = candidates[i].temp;
        forecastData[i].desc = candidates[i].desc;
        forecastData[i].pop = candidates[i].pop;

        // Log dữ liệu
        char timeStr[20];
        time_t rawtime = forecastData[i].timestamp;
        struct tm* timeinfo = localtime(&rawtime);
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M", timeinfo);
        Serial.printf("Dự báo ngày %d: %s, %.1f°C, %s, xác suất mưa %.0f%%\n",
                      i + 1, timeStr, forecastData[i].temp, forecastData[i].desc.c_str(), forecastData[i].pop * 100);
      }
      xSemaphoreGive(forecastSemaphore);
    }
  } else {
    Serial.println("Lỗi HTTP dự báo: " + String(httpResponseCode));
    if (xSemaphoreTake(forecastSemaphore, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 3; i++) {
        forecastData[i].desc = "Error";
        forecastData[i].pop = 0.0;
        forecastData[i].temp = 0.0;
        forecastData[i].timestamp = 0;
      }
      xSemaphoreGive(forecastSemaphore);
    }
  }
  http.end();
}
void createWavHeader(uint8_t* wavHeader, int dataSize) {
  memcpy(wavHeader, "RIFF", 4);
  *(uint32_t*)(wavHeader + 4) = dataSize + 36;
  memcpy(wavHeader + 8, "WAVE", 4);
  memcpy(wavHeader + 12, "fmt ", 4);
  *(uint32_t*)(wavHeader + 16) = 16;
  *(uint16_t*)(wavHeader + 20) = 1;
  *(uint16_t*)(wavHeader + 22) = 1;
  *(uint32_t*)(wavHeader + 24) = RATE;
  *(uint32_t*)(wavHeader + 28) = RATE * 2;
  *(uint16_t*)(wavHeader + 32) = 2;
  *(uint16_t*)(wavHeader + 34) = 16;
  memcpy(wavHeader + 36, "data", 4);
  *(uint32_t*)(wavHeader + 40) = dataSize;
}

String parseTranscript(const String& response) {
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    Serial.println("Lỗi phân tích JSON: " + String(error.c_str()));
    return "";
  }

  JsonArray results = doc["response"]["result"];
  if (results.size() > 0) {
    const char* transcript = results[0]["transcript"];
    return transcript ? String(transcript) : "";
  }
  return "";
}

bool extractTimerTime(const String& transcript, int& hour, int& minute) {
  String lowerTranscript = removeVietnameseDiacritics(transcript);
  lowerTranscript.toLowerCase();
  Serial.println("Lower transcript (timer): [" + lowerTranscript + "]");

  int foundMinute = -1;

  // Mảng số tiếng Việt từ 0-99
  const char* vietnameseNumbers[] = {
    "khong", "mot", "hai", "ba", "bon", "nam", "sau", "bay", "tam", "chin",
    "muoi", "muoi mot", "muoi hai", "muoi ba", "muoi bon", "muoi nam", "muoi sau",
    "muoi bay", "muoi tam", "muoi chin", "hai muoi", "hai muoi mot", "hai muoi hai",
    "hai muoi ba", "hai muoi bon", "hai muoi nam", "hai muoi sau", "hai muoi bay",
    "hai muoi tam", "hai muoi chin", "ba muoi", "ba muoi mot", "ba muoi hai",
    "ba muoi ba", "ba muoi bon", "ba muoi nam", "ba muoi sau", "ba muoi bay",
    "ba muoi tam", "ba muoi chin", "bon muoi", "bon muoi mot", "bon muoi hai",
    "bon muoi ba", "bon muoi bon", "bon muoi nam", "bon muoi sau", "bon muoi bay",
    "bon muoi tam", "bon muoi chin", "nam muoi", "nam muoi mot", "nam muoi hai",
    "nam muoi ba", "nam muoi bon", "nam muoi nam", "nam muoi sau", "nam muoi bay",
    "nam muoi tam", "nam muoi chin", "sau muoi", "sau muoi mot", "sau muoi hai",
    "sau muoi ba", "sau muoi bon", "sau muoi nam", "sau muoi sau", "sau muoi bay",
    "sau muoi tam", "sau muoi chin", "bay muoi", "bay muoi mot", "bay muoi hai",
    "bay muoi ba", "bay muoi bon", "bay muoi nam", "bay muoi sau", "bay muoi bay",
    "bay muoi tam", "bay muoi chin", "tam muoi", "tam muoi mot", "tam muoi hai",
    "tam muoi ba", "tam muoi bon", "tam muoi nam", "tam muoi sau", "tam muoi bay",
    "tam muoi tam", "tam muoi chin", "chin muoi", "chin muoi mot", "chin muoi hai",
    "chin muoi ba", "chin muoi bon", "chin muoi nam", "chin muoi sau", "chin muoi bay",
    "chin muoi tam", "chin muoi chin"
  };

  // Tìm vị trí "phut"
  int minuteIndex = lowerTranscript.indexOf("phut");
  if (minuteIndex == -1) {
    Serial.println("Không tìm thấy 'phút' trong transcript (timer)");
    return false;
  }

  // Lấy chuỗi trước "phut"
  String beforeMinute = lowerTranscript.substring(0, minuteIndex);
  beforeMinute.trim();
  Serial.println("Chuỗi trước phút: [" + beforeMinute + "]");

  // Tách chuỗi thành các từ
  String words[10];
  int wordCount = 0;
  int start = 0;
  for (int i = 0; i <= beforeMinute.length(); i++) {
    if (i == beforeMinute.length() || beforeMinute[i] == ' ' || beforeMinute[i] == '-') {
      if (start < i) {
        words[wordCount] = beforeMinute.substring(start, i);
        words[wordCount].trim();
        Serial.printf("Từ %d: [%s]\n", wordCount, words[wordCount].c_str());
        wordCount++;
      }
      start = i + 1;
    }
  }

  // Tìm số hợp lệ gần "phut" nhất
  int lastNumber = -1;
  int lastNumberPos = -1;

  // Kiểm tra trường hợp hai số nối bằng "-" (như "2 - 3" thành 23)
  for (int i = 0; i < wordCount - 2; i++) {
    Serial.printf("Kiểm tra ghép số tại i=%d: [%s] [%s] [%s]\n", i, words[i].c_str(),
                  (i + 1 < wordCount) ? words[i + 1].c_str() : "",
                  (i + 2 < wordCount) ? words[i + 2].c_str() : "");
    if (i + 2 < wordCount && words[i + 1] == "-" && words[i].toInt() > 0 && words[i + 2].toInt() > 0) {
      int firstNum = words[i].toInt();
      int secondNum = words[i + 2].toInt();
      int combinedNum = firstNum * 10 + secondNum;
      Serial.printf("Tìm thấy số ghép: %d - %d = %d\n", firstNum, secondNum, combinedNum);
      if (combinedNum <= 99) {
        lastNumber = combinedNum;
        lastNumberPos = beforeMinute.lastIndexOf(words[i + 2]);
        break;
      } else {
        Serial.println("Số ghép không hợp lệ (> 99): " + String(combinedNum));
      }
    }
  }

  // Nếu không tìm thấy số ghép, kiểm tra số ghép tiếng Việt hoặc số đơn
  if (lastNumber == -1) {
    Serial.println("Không tìm thấy số ghép với '-', kiểm tra số khác");
    for (int i = wordCount - 1; i >= 0; i--) {
      String currentWord = words[i];
      String nextWord = (i + 1 < wordCount) ? words[i + 1] : "";

      // Tạo số ghép (ví dụ: "muoi" + "mot" = "muoi mot")
      String combined = currentWord;
      if (nextWord != "" && (currentWord == "muoi" || currentWord == "hai" || currentWord == "ba" || currentWord == "bon" || currentWord == "nam" || currentWord == "sau" || currentWord == "bay" || currentWord == "tam" || currentWord == "chin")) {
        combined = currentWord + " " + nextWord;
        Serial.println("Kiểm tra số ghép tiếng Việt: [" + combined + "]");
      }

      // Kiểm tra số ghép trong mảng vietnameseNumbers
      for (int j = 0; j <= 99; j++) {
        if (combined == vietnameseNumbers[j]) {
          lastNumber = j;
          lastNumberPos = beforeMinute.lastIndexOf(combined);
          Serial.printf("Tìm thấy số ghép tiếng Việt: %d tại vị trí %d\n", lastNumber, lastNumberPos);
          break;
        }
      }

      // Kiểm tra số đơn
      if (lastNumber == -1) {
        for (int j = 0; j <= 99; j++) {
          if (currentWord == vietnameseNumbers[j]) {
            lastNumber = j;
            lastNumberPos = beforeMinute.lastIndexOf(currentWord);
            Serial.printf("Tìm thấy số đơn: %d tại vị trí %d\n", lastNumber, lastNumberPos);
            break;
          }
        }
      }

      // Kiểm tra số dạng chuỗi (như "11")
      if (lastNumber == -1 && currentWord.toInt() > 0 && currentWord.toInt() <= 99) {
        lastNumber = currentWord.toInt();
        lastNumberPos = beforeMinute.lastIndexOf(currentWord);
        Serial.printf("Tìm thấy số dạng chuỗi: %d tại vị trí %d\n", lastNumber, lastNumberPos);
      }

      if (lastNumber != -1) {
        break;  // Lấy số gần "phut" nhất
      }
    }
  }

  // Xử lý lỗi nhận diện "moi" thay vì "muoi"
  if (lastNumber == -1 && beforeMinute.indexOf("moi") != -1) {
    lastNumber = 10;
    lastNumberPos = beforeMinute.lastIndexOf("moi");
    Serial.println("Tìm thấy số 'moi' (mười): 10");
  }

  if (lastNumber == -1) {
    Serial.println("Không tìm thấy số phút hợp lệ trong transcript");
    return false;
  }

  foundMinute = lastNumber;
  if (foundMinute < 0 || foundMinute > 99) {
    Serial.printf("Lỗi: Số phút (%d) không hợp lệ, phải từ 0 đến 99\n", foundMinute);
    return false;
  }

  hour = 0;  // Hẹn giờ không dùng giờ
  minute = foundMinute;
  Serial.printf("Trích xuất thời gian (timer): %d giờ %d phút\n", hour, minute);
  return true;
}
bool extractAlarmTime(const String& transcript, int& hour, int& minute) {
  String lowerTranscript = removeVietnameseDiacritics(transcript);
  lowerTranscript.toLowerCase();
  Serial.println("Lower transcript (alarm): [" + lowerTranscript + "]");

  int foundHour = 0;
  int foundMinute = 0;

  // Danh sách số dạng chữ từ 0 đến 59
  const char* vietnameseNumbers[] = {
    "khong", "mot", "hai", "ba", "bon", "nam", "sau", "bay", "tam", "chin", "muoi",
    "muoi mot", "muoi hai", "muoi ba", "muoi bon", "muoi lam", "muoi sau",
    "muoi bay", "muoi tam", "muoi chin", "hai muoi", "hai muoi mot",
    "hai muoi hai", "hai muoi ba", "hai muoi tu", "hai muoi lam", "hai muoi sau",
    "hai muoi bay", "hai muoi tam", "hai muoi chin", "ba muoi", "ba muoi mot",
    "ba muoi hai", "ba muoi ba", "ba muoi bon", "ba muoi lam", "ba muoi sau",
    "ba muoi bay", "ba muoi tam", "ba muoi chin", "bon muoi", "bon muoi mot",
    "bon muoi hai", "bon muoi ba", "bon muoi bon", "bon muoi lam", "bon muoi sau",
    "bon muoi bay", "bon muoi tam", "bon muoi chin", "nam muoi", "nam muoi mot",
    "nam muoi hai", "nam muoi ba", "nam muoi bon", "nam muoi lam", "nam muoi sau",
    "nam muoi bay", "nam muoi tam", "nam muoi chin"
  };

  // Tách transcript thành các từ
  char transcriptCopy[128];
  strncpy(transcriptCopy, lowerTranscript.c_str(), sizeof(transcriptCopy) - 1);
  transcriptCopy[sizeof(transcriptCopy) - 1] = '\0';
  char* token = strtok(transcriptCopy, " ");
  int tokenIndex = 0;
  int gioIndex = -1;
  int hIndex = -1;
  int numIndices[16];
  int numValues[16];
  int numCount = 0;

  // Duyệt qua các từ để tìm "gio", "h" và các số
  while (token != NULL && numCount < 16) {
    String tokenStr = String(token);
    tokenStr.trim();

    // Xử lý trường hợp "XhY" (ví dụ: "2h15")
    if (tokenStr.indexOf("h") != -1 && tokenStr != "phut") {
      hIndex = tokenIndex;
      int hPos = tokenStr.indexOf("h");
      String hourPart = tokenStr.substring(0, hPos);
      String minutePart = tokenStr.substring(hPos + 1);

      // Xử lý giờ
      int hourNum = hourPart.toInt();
      if (hourNum > 0 || hourPart == "0") {
        numIndices[numCount] = tokenIndex - 1;
        numValues[numCount] = hourNum;
        numCount++;
        foundHour = hourNum;
      } else {
        for (int i = 0; i <= 59; i++) {
          if (hourPart == vietnameseNumbers[i]) {
            numIndices[numCount] = tokenIndex - 1;
            numValues[numCount] = i;
            numCount++;
            foundHour = i;
            break;
          }
        }
      }

      // Xử lý phút
      if (minutePart.length() > 0) {
        int minuteNum = minutePart.toInt();
        if (minuteNum > 0 || minutePart == "0") {
          numIndices[numCount] = tokenIndex;
          numValues[numCount] = minuteNum;
          numCount++;
          foundMinute = minuteNum;
        } else {
          for (int i = 0; i <= 59; i++) {
            if (minutePart == vietnameseNumbers[i]) {
              numIndices[numCount] = tokenIndex;
              numValues[numCount] = i;
              numCount++;
              foundMinute = i;
              break;
            }
          }
        }
      }
    } else if (tokenStr == "gio") {
      gioIndex = tokenIndex;
    } else {
      // Kiểm tra số dạng số
      int num = tokenStr.toInt();
      if (num > 0 || tokenStr == "0") {
        numIndices[numCount] = tokenIndex;
        numValues[numCount] = num;
        numCount++;
      } else {
        // Kiểm tra số dạng chữ
        for (int i = 0; i <= 59; i++) {
          if (tokenStr == vietnameseNumbers[i]) {
            numIndices[numCount] = tokenIndex;
            numValues[numCount] = i;
            numCount++;
            break;
          }
        }
      }
    }
    token = strtok(NULL, " ");
    tokenIndex++;
  }

  // Log debug
  Serial.printf("gioIndex: %d, hIndex: %d, numCount: %d\n", gioIndex, hIndex, numCount);
  for (int i = 0; i < numCount; i++) {
    Serial.printf("Number %d at index %d: %d\n", i, numIndices[i], numValues[i]);
  }

  // Xử lý báo thức nếu không tìm thấy "XhY"
  if (foundHour == 0 && (gioIndex != -1 || hIndex != -1)) {
    int targetIndex = gioIndex != -1 ? gioIndex : hIndex;
    int minDistance = 100;
    int closestNumIndex = -1;
    for (int i = 0; i < numCount; i++) {
      if (numIndices[i] <= targetIndex) {
        int distance = targetIndex - numIndices[i];
        if (distance < minDistance) {
          minDistance = distance;
          closestNumIndex = i;
        }
      }
    }
    if (closestNumIndex != -1) {
      foundHour = numValues[closestNumIndex];
      foundMinute = 0;  // Báo thức không có phút → mặc định 0
    }
  }

  if (foundHour == 0 && foundMinute == 0) {
    Serial.println("Không tìm thấy giờ hoặc phút trong transcript (alarm).");
    return false;
  }

  hour = foundHour;
  minute = foundMinute;
  Serial.printf("Trích xuất thời gian (alarm): %d giờ %d phút\n", hour, minute);
  return true;
}

String removeVietnameseDiacritics(String input) {
  String output = input;
  const char* vietnamese[] = {
    "à", "á", "ạ", "ả", "ã", "â", "ầ", "ấ", "ậ", "ẩ", "ẫ", "ă", "ằ", "ắ", "ặ", "ẳ", "ẵ",
    "è", "é", "ẹ", "ẻ", "ẽ", "ê", "ề", "ế", "ệ", "ể", "ễ",
    "ì", "í", "ị", "ỉ", "ĩ",
    "ò", "ó", "ọ", "ỏ", "õ", "ô", "ồ", "ố", "ộ", "ổ", "ỗ", "ơ", "ờ", "ớ", "ợ", "ở", "ỡ",
    "ù", "ú", "ụ", "ủ", "ũ", "ư", "ừ", "ứ", "ự", "ử", "ữ",
    "ỳ", "ý", "ỵ", "ỷ", "ỹ",
    "đ", "Đ"
  };
  const char* noDiacritics[] = {
    "a", "a", "a", "a", "a", "a", "a", "a", "a", "a", "a", "a", "a", "a", "a", "a", "a",
    "e", "e", "e", "e", "e", "e", "e", "e", "e", "e", "e",
    "i", "i", "i", "i", "i",
    "o", "o", "o", "o", "o", "o", "o", "o", "o", "o", "o", "o", "o", "o", "o", "o", "o",
    "u", "u", "u", "u", "u", "u", "u", "u", "u", "u", "u",
    "y", "y", "y", "y", "y",
    "d", "D"
  };
  for (size_t i = 0; i < sizeof(vietnamese) / sizeof(vietnamese[0]); i++) {
    output.replace(vietnamese[i], noDiacritics[i]);
  }
  return output;
}

void timeToVietnameseText(int hour, int minute, int day, int month, int year, char* output, size_t maxLen) {
  const char* numbers[] = {
    "không", "một", "hai", "ba", "bốn", "năm", "sáu", "bảy", "tám", "chín", "mười",
    "mười một", "mười hai", "mười ba", "mười bốn", "mười lăm", "mười sáu", "mười bảy",
    "mười tám", "mười chín", "hai mươi", "hai mươi mốt", "hai mươi hai", "hai mươi ba",
    "hai mươi tư", "hai mươi lăm", "hai mươi sáu", "hai mươi bảy", "hai mươi tám", "hai mươi chín",
    "ba mươi", "ba mươi mốt", "ba mươi hai", "ba mươi ba", "ba mươi bốn", "ba mươi lăm",
    "ba mươi sáu", "ba mươi bảy", "ba mươi tám", "ba mươi chín", "bốn mươi", "bốn mươi mốt",
    "bốn mươi hai", "bốn mươi ba", "bốn mươi bốn", "bốn mươi lăm", "bốn mươi sáu", "bốn mươi bảy",
    "bốn mươi tám", "bốn mươi chín", "năm mươi", "năm mươi mốt", "năm mươi hai", "năm mươi ba",
    "năm mươi bốn", "năm mươi lăm", "năm mươi sáu", "năm mươi bảy", "năm mươi tám", "năm mươi chín",
    "sáu mươi", "sáu mươi mốt", "sáu mươi hai", "sáu mươi ba", "sáu mươi bốn", "sáu mươi lăm",
    "sáu mươi sáu", "sáu mươi bảy", "sáu mươi tám", "sáu mươi chín", "bảy mươi", "bảy mươi mốt",
    "bảy mươi hai", "bảy mươi ba", "bảy mươi bốn", "bảy mươi lăm", "bảy mươi sáu", "bảy mươi bảy",
    "bảy mươi tám", "bảy mươi chín", "tám mươi", "tám mươi mốt", "tám mươi hai", "tám mươi ba",
    "tám mươi bốn", "tám mươi lăm", "tám mươi sáu", "tám mươi bảy", "tám mươi tám", "tám mươi chín",
    "chín mươi", "chín mươi mốt", "chín mươi hai", "chín mươi ba", "chín mươi bốn", "chín mươi lăm",
    "chín mươi sáu", "chín mươi bảy", "chín mươi tám", "chín mươi chín"
  };

  const char* single_digits[] = {
    "không", "một", "hai", "ba", "bốn", "năm", "sáu", "bảy", "tám", "chín"
  };

  // Khởi tạo chuỗi đầu ra
  output[0] = '\0';

  // Thêm giờ
  snprintf(output, maxLen, "Bây giờ là %s giờ", numbers[hour]);

  // Thêm phút nếu có
  if (minute > 0) {
    strncat(output, " ", maxLen - strlen(output) - 1);
    strncat(output, numbers[minute], maxLen - strlen(output) - 1);
    strncat(output, " phút", maxLen - strlen(output) - 1);
  }

  // Thêm ngày, tháng, năm
  strncat(output, ", ngày ", maxLen - strlen(output) - 1);
  strncat(output, numbers[day], maxLen - strlen(output) - 1);
  strncat(output, " tháng ", maxLen - strlen(output) - 1);
  strncat(output, numbers[month], maxLen - strlen(output) - 1);
  strncat(output, " năm ", maxLen - strlen(output) - 1);

  // Chuyển đổi năm thành chữ (ví dụ: 2025 -> "hai nghìn không trăm hai mươi lăm")
  char year_text[64] = "";
  int thousands = year / 1000;
  int hundreds = (year % 1000) / 100;
  int tens_units = year % 100;

  if (thousands > 0) {
    strncat(year_text, single_digits[thousands], sizeof(year_text) - strlen(year_text) - 1);
    strncat(year_text, " nghìn", sizeof(year_text) - strlen(year_text) - 1);
  }

  if (hundreds > 0) {
    if (strlen(year_text) > 0) strncat(year_text, " ", sizeof(year_text) - strlen(year_text) - 1);
    strncat(year_text, single_digits[hundreds], sizeof(year_text) - strlen(year_text) - 1);
    strncat(year_text, " trăm", sizeof(year_text) - strlen(year_text) - 1);
  }

  if (tens_units > 0) {
    if (strlen(year_text) > 0) strncat(year_text, " ", sizeof(year_text) - strlen(year_text) - 1);
    strncat(year_text, numbers[tens_units], sizeof(year_text) - strlen(year_text) - 1);
  } else if (year == 0) {
    strncat(year_text, "không", sizeof(year_text) - strlen(year_text) - 1);
  }

  strncat(output, year_text, maxLen - strlen(output) - 1);
}

void sendToViettelAPI(uint8_t* pcmData, size_t pcmSize) {
  Serial.printf("pcmSize: %d bytes\n", pcmSize);
  if (pcmSize == 0) {
    Serial.println("Lỗi: Không có dữ liệu âm thanh từ mic!");
    heap_caps_free(pcmData);
    playTTS("Vui lòng thử lại", "vi");
    return;
  }

  Serial.printf("Free heap trước khi cấp phát WAV: %d bytes\n", ESP.getFreeHeap());
  size_t wavSize = 44 + pcmSize;
  uint8_t* wavData = (uint8_t*)heap_caps_malloc(wavSize, MALLOC_CAP_8BIT);
  if (!wavData) {
    Serial.println("Lỗi: Không đủ bộ nhớ cho WAV!");
    heap_caps_free(pcmData);
    return;
  }

  uint8_t wavHeader[44];
  createWavHeader(wavHeader, pcmSize);
  memcpy(wavData, wavHeader, 44);
  memcpy(wavData + 44, pcmData, pcmSize);
  heap_caps_free(pcmData);

  const char* boundary = "----ESP32Boundary";
  String part1 = String("--") + boundary + "\r\n" + "Content-Disposition: form-data; name=\"file\"; filename=\"recorded.wav\"\r\n" + "Content-Type: audio/wav\r\n\r\n";
  String part2 = "\r\n--" + String(boundary) + "--\r\n";

  size_t part1Len = part1.length();
  size_t part2Len = part2.length();
  size_t payloadSize = part1Len + wavSize + part2Len;

  Serial.printf("wavSize: %d bytes, payloadSize: %d bytes\n", wavSize, payloadSize);
  Serial.printf("Free heap trước khi cấp phát payload: %d bytes\n", ESP.getFreeHeap());
  uint8_t* payload = (uint8_t*)heap_caps_malloc(payloadSize, MALLOC_CAP_8BIT);
  if (!payload) {
    Serial.println("Lỗi: Không đủ bộ nhớ cho payload!");
    heap_caps_free(wavData);
    return;
  }

  memcpy(payload, part1.c_str(), part1Len);
  memcpy(payload + part1Len, wavData, wavSize);
  memcpy(payload + part1Len + wavSize, part2.c_str(), part2Len);
  heap_caps_free(wavData);

  HTTPClient http;
  Serial.printf("WiFi status: %d, RSSI: %d dBm\n", WiFi.status(), WiFi.RSSI());
  http.setTimeout(15000);
  http.begin(viettel_api);
  String contentType = "multipart/form-data; boundary=" + String(boundary);
  http.addHeader("Content-Type", contentType);
  http.addHeader("Authorization", "Bearer " + String(viettel_token));

  unsigned long start = millis();
  int httpResponseCode = http.POST(payload, payloadSize);
  Serial.printf("Thời gian gửi yêu cầu: %lu ms\n", millis() - start);
  heap_caps_free(payload);

  String responseText = "";
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Phản hồi API: " + response);

    String transcript = parseTranscript(response);
    if (transcript == "") {
      Serial.println("Lỗi: Transcript rỗng, không nhận diện được giọng nói");
      Serial1.write('c');
      responseText = "Không nhận diện được giọng nói, vui lòng thử lại";
    } else {
      Serial.print("Transcript: ");
      Serial.println(transcript);

      transcript.replace(".", "");
      String noDiacriticsTranscript = removeVietnameseDiacritics(transcript);
      String lowerTranscript = noDiacriticsTranscript;
      lowerTranscript.toLowerCase();

      if (xSemaphoreTake(commandSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
        currentCommand = transcript;
        xSemaphoreGive(commandSemaphore);
      } else {
        Serial.println("Lỗi: Không lấy được commandSemaphore");
      }

      // Hàm chuyển đổi mô tả thời tiết sang tiếng Việt
      auto translateWeatherDesc = [](String desc) {
        desc.replace("clear sky", "trời quang đãng");
        desc.replace("few clouds", "trời quang đãng");
        desc.replace("scattered clouds", "trời nhiều mây");
        desc.replace("broken clouds", "trời nhiều mây");
        desc.replace("overcast clouds", "trời nhiều mây");
        desc.replace("light rain", "mưa nhẹ");
        desc.replace("moderate rain", "mưa vừa");
        desc.replace("heavy intensity rain", "mưa rất to");
        desc.replace("very heavy rain", "mưa to kèm sấm chớp");
        desc.replace("extreme rain", "mưa to kèm sấm chớp");
        desc.replace("shower rain", "mưa rào");
        desc.replace("light intensity drizzle", "mưa phùn nhẹ");
        desc.replace("drizzle", "mưa phùn");
        desc.replace("heavy intensity drizzle", "mưa phùn dày đặc");
        desc.replace("thunderstorm", "giông bão");
        desc.replace("thunderstorm with rain", "giông kèm mưa");
        desc.replace("thunderstorm with heavy rain", "giông kèm mưa rất to");
        return desc;
      };

      // Xử lý các lệnh
      if (lowerTranscript.indexOf("do") != -1) {
        Serial.println("Bật đèn đỏ");
        responseText = "Đã bật đèn đỏ";
        Serial1.write('c');
      } else if (lowerTranscript.indexOf("vang") != -1) {
        Serial.println("Bật đèn vàng");
        responseText = "Đã bật đèn vàng";
        Serial1.write('d');
      } else if (lowerTranscript.indexOf("xanh") != -1) {
        Serial.println("Bật đèn xanh");
        responseText = "Đã bật đèn xanh";
        Serial1.write('e');
      } else if (lowerTranscript.indexOf("trang") != -1) {
        Serial.println("Bật đèn trắng");
        responseText = "Đã bật đèn trắng";
        Serial1.write('f');
      } else if (lowerTranscript.indexOf("tat") != -1) {
        Serial.println("Tắt đèn");
        responseText = "Đã tắt đèn";
        Serial1.write('b');
      } else if (lowerTranscript.indexOf("bay gio") != -1 || lowerTranscript.indexOf("may gio") != -1 || lowerTranscript.indexOf("may") != -1) {
        Serial.println("Hỏi giờ");
        if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
          DateTime now = rtc.now();
          char timeText[150];
          timeToVietnameseText(now.hour(), now.minute(), now.day(), now.month(), now.year(), timeText, sizeof(timeText));
          responseText = timeText;
          xSemaphoreGive(displaySemaphore);
          Serial1.write('m');
        }
      } else if ((lowerTranscript.indexOf("thoi") != -1 || lowerTranscript.indexOf("tiet") != -1 || lowerTranscript.indexOf("hom") != -1 || lowerTranscript.indexOf("nay") != -1) && (lowerTranscript.indexOf("du bao") == -1 && lowerTranscript.indexOf("ngay mai") == -1 && lowerTranscript.indexOf("ngay kia") == -1 && lowerTranscript.indexOf("ba ngay") == -1)) {
        Serial.println("Hỏi thời tiết");
        if (xSemaphoreTake(weatherSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
          String vietnameseWeather = weatherInfo;
          float temp = 0;
          String desc = "";
          int colonIndex = vietnameseWeather.indexOf(": ");
          int commaIndex = vietnameseWeather.indexOf(", ");
          if (colonIndex != -1 && commaIndex != -1) {
            String tempStr = vietnameseWeather.substring(colonIndex + 2, commaIndex);
            tempStr.replace("C", "");
            temp = tempStr.toFloat();
            desc = vietnameseWeather.substring(commaIndex + 2);
          }
          desc = translateWeatherDesc(desc);
          char weatherTTS[64];
          snprintf(weatherTTS, sizeof(weatherTTS), "Hà Nội, %d độ C, %s", (int)temp, desc.c_str());
          responseText = weatherTTS;
          xSemaphoreGive(weatherSemaphore);
          Serial1.write('n');
        }
      } else if (lowerTranscript.indexOf("bao thuc") != -1 || lowerTranscript.indexOf("thuc") != -1 || lowerTranscript.indexOf("luc") != -1) {
        Serial.println("Báo thức");
        int hour, minute;
        if (extractAlarmTime(transcript, hour, minute)) {
          if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            alarmEnabled = true;
            alarmHour = hour;
            alarmMinute = minute;
            currentCommand = "Đã đặt báo thức vào lúc " + String(hour) + ":" + (minute < 10 ? "0" : "") + String(minute);
            responseText = currentCommand;
            Serial.printf("Đã đặt báo thức: %d:%02d\n", hour, minute);
            xSemaphoreGive(displaySemaphore);
            Serial1.write('p');
          }
        } else {
          responseText = "Không rõ thời gian báo thức, vui lòng thử lại";
        }
      } else if (lowerTranscript.indexOf("hen") != -1 || lowerTranscript.indexOf("phut") != -1) {
        Serial.println("Hẹn giờ");
        int hour, minute;
        if (lowerTranscript.indexOf("nam phut") != -1) {
          if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            timer5MinEnabled = true;
            timer5MinEnd = rtc.now().unixtime() + (5 * 60);
            timerEnabled = false;
            currentCommand = "Đã đặt hẹn giờ 5 phút";
            responseText = currentCommand;
            Serial.println("Đã đặt hẹn giờ: 5 phút");
            xSemaphoreGive(displaySemaphore);
            Serial1.write('o');
          }
        } else if (extractTimerTime(transcript, hour, minute)) {
          if (minute > 0) {
            if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
              timerEnabled = true;
              timerEndTime = rtc.now().unixtime() + (minute * 60);
              timer5MinEnabled = false;
              timer2MinEnabled = false;
              currentCommand = "Hẹn giờ kết thúc sau " + String(minute) + " phút";
              responseText = currentCommand;
              Serial.printf("Đã đặt hẹn giờ: %d phút\n", minute);
              xSemaphoreGive(displaySemaphore);
              Serial1.write('o');
            }
          } else {
            responseText = "Vui lòng cung cấp thời gian hẹn giờ hợp lệ";
          }
        } else {
          responseText = "Vui lòng cung cấp thời gian hẹn giờ";
        }
      } else if (lowerTranscript.indexOf("du bao") != -1) {
        Serial.println("Hỏi dự báo thời tiết");
        // Lấy thời gian hiện tại để tính ngày
        DateTime now = rtc.now();
        DateTime tomorrow = DateTime(now.unixtime() + 86400);
        DateTime dayAfterTomorrow = DateTime(now.unixtime() + 2 * 86400);
        DateTime thirdDay = DateTime(now.unixtime() + 3 * 86400);

        if (lowerTranscript.indexOf("ngay mai") != -1 || lowerTranscript.indexOf("mot ngay") != -1) {
          Serial.println("Dự báo thời tiết ngày mai");
          if (xSemaphoreTake(forecastSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (forecastData[0].timestamp > 0) {
              String desc = translateWeatherDesc(forecastData[0].desc);
              char forecastTTS[256];
              snprintf(forecastTTS, sizeof(forecastTTS),
                       "Dự báo ngày mai, %d tháng %d, Hà Nội, %d độ C, %s, xác suất mưa %d phần trăm",
                       tomorrow.day(), tomorrow.month(),
                       (int)forecastData[0].temp, desc.c_str(), (int)(forecastData[0].pop * 100));
              responseText = forecastTTS;
              Serial1.write('n');
            } else {
              responseText = "Không có dữ liệu dự báo ngày mai, vui lòng thử lại sau";
            }
            xSemaphoreGive(forecastSemaphore);
          }
        } else if (lowerTranscript.indexOf("ngay kia") != -1 || lowerTranscript.indexOf("hai ngay") != -1) {
          Serial.println("Dự báo thời tiết ngày kia");
          if (xSemaphoreTake(forecastSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (forecastData[1].timestamp > 0) {
              String desc = translateWeatherDesc(forecastData[1].desc);
              char forecastTTS[256];
              snprintf(forecastTTS, sizeof(forecastTTS),
                       "Dự báo ngày kia, %d tháng %d, Hà Nội, %d độ C, %s, xác suất mưa %d phần trăm",
                       dayAfterTomorrow.day(), dayAfterTomorrow.month(),
                       (int)forecastData[1].temp, desc.c_str(), (int)(forecastData[1].pop * 100));
              responseText = forecastTTS;
              Serial1.write('n');
            } else {
              responseText = "Không có dữ liệu dự báo ngày kia, vui lòng thử lại sau";
            }
            xSemaphoreGive(forecastSemaphore);
          }
        } else if (lowerTranscript.indexOf("ba ngay") != -1) {
          Serial.println("Dự báo thời tiết ngày thứ 3");
          if (xSemaphoreTake(forecastSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (forecastData[2].timestamp > 0) {
              String desc = translateWeatherDesc(forecastData[2].desc);
              char forecastTTS[256];
              snprintf(forecastTTS, sizeof(forecastTTS),
                       "Dự báo ngày %d tháng %d, Hà Nội, %d độ C, %s, xác suất mưa %d phần trăm",
                       thirdDay.day(), thirdDay.month(),
                       (int)forecastData[2].temp, desc.c_str(), (int)(forecastData[2].pop * 100));
              responseText = forecastTTS;
              Serial1.write('n');
            } else {
              responseText = "Không có dữ liệu dự báo ngày thứ 3, vui lòng thử lại sau";
            }
            xSemaphoreGive(forecastSemaphore);
          }
        } else {
          responseText = "Vui lòng chỉ định ngày dự báo (ngày mai, ngày kia, hoặc ba ngày)";
        }
      } else {
        responseText = "Không hiểu lệnh, vui lòng thử lại";
      }
    }
  } else {
    Serial.println("Lỗi HTTP: " + http.errorToString(httpResponseCode));
    Serial1.write('c');
    responseText = "Lỗi, vui lòng thử lại";
  }

  if (WiFi.status() == WL_CONNECTED && responseText != "") {
    Serial.println("Chuẩn bị phát TTS: " + responseText);
    playTTS(responseText.c_str(), "vi");
  } else {
    Serial.println("Không phát TTS: WiFi ngắt hoặc responseText rỗng");
  }

  http.end();
}

void fetchWeather() {
  HTTPClient http;
  http.begin(weather_api);
  int httpResponseCode = http.GET();

  if (httpResponseCode == 200) {
    String payload = http.getString();
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    float temp = doc["main"]["temp"];
    String desc = doc["weather"][0]["description"];
    String weatherStr = String("Hanoi: ") + String(temp, 1) + "C, " + desc;

    if (xSemaphoreTake(weatherSemaphore, portMAX_DELAY) == pdTRUE) {
      weatherInfo = weatherStr;
      xSemaphoreGive(weatherSemaphore);
    }
  } else {
    if (xSemaphoreTake(weatherSemaphore, portMAX_DELAY) == pdTRUE) {
      weatherInfo = "Weather: Error";
      xSemaphoreGive(weatherSemaphore);
    }
  }
  http.end();
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  int timeout = 10;
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    timeout--;
  }
}
void wifiTask(void* pvParameters) {
  connectWiFi();
  fetchWeather();
  fetchForecast();  // Thêm gọi fetchForecast
  syncTimeFromAPI();

  unsigned long lastWeatherUpdate = 0;
  unsigned long lastForecastUpdate = 0;  // Biến cho dự báo
  unsigned long lastTimeSync = 0;
  const unsigned long weatherInterval = 600000;      // 10 phút
  const unsigned long forecastInterval = 3600000;    // 1 giờ cho dự báo
  const unsigned long timeSyncInterval = 604800000;  // 7 ngày

  isOnline = (WiFi.status() == WL_CONNECTED);
  lastWiFiState = isOnline;
  lastWiFiCheck = xTaskGetTickCount();

  for (;;) {
    bool wifiConnected = (WiFi.status() == WL_CONNECTED);
    if ((xTaskGetTickCount() - lastWiFiCheck) > wifiDebounceTime) {
      if (wifiConnected != lastWiFiState) {
        isOnline = wifiConnected;
        lastWiFiState = wifiConnected;
        lastWiFiCheck = xTaskGetTickCount();
        Serial.printf("Trạng thái WiFi thay đổi: %s\n", isOnline ? "Online" : "Offline");
        if (wifiConnected) {
          if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            displayMode2 = false;
            fetchWeather();
            fetchForecast();  // Cập nhật dự báo khi kết nối lại
            xSemaphoreGive(displaySemaphore);
          }
          syncTimeFromAPI();
        } else {
          connectWiFi();
        }
      }
    }

    // Cập nhật thời tiết và dự báo
    if (wifiConnected && millis() - lastWeatherUpdate >= weatherInterval) {
      fetchWeather();
      lastWeatherUpdate = millis();
    }
    if (wifiConnected && millis() - lastForecastUpdate >= forecastInterval) {
      fetchForecast();
      lastForecastUpdate = millis();
    }

    if (wifiConnected && millis() - lastTimeSync >= timeSyncInterval) {
      if (syncTimeFromAPI()) {
        lastTimeSync = millis();
      }
    }

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) > 0) {
      if (!micI2SInitialized) {
        Serial.println("Lỗi: I2S mic chưa được khởi tạo từ setup!");
        if (WiFi.status() == WL_CONNECTED) {
          playTTS("Lỗi micro, vui lòng khởi động lại thiết bị.", "vi");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }

      uint8_t* pcmData = NULL;
      size_t pcmSize = 0;

      if (recordAudio_new_driver(&pcmData, &pcmSize)) {
        if (pcmSize > 0 && pcmData != NULL) {
          sendToViettelAPI(pcmData, pcmSize);
        } else {
          Serial.println("Lỗi: Dữ liệu PCM không hợp lệ sau khi thu âm.");
          if (pcmData) heap_caps_free(pcmData);
          if (WiFi.status() == WL_CONNECTED) {
            playTTS("Lỗi thu âm, vui lòng thử lại.", "vi");
          }
        }
      } else {
        Serial.println("Lỗi thu âm với driver mới!");
        if (WiFi.status() == WL_CONNECTED) {
          playTTS("Lỗi thu âm, vui lòng thử lại.", "vi");
        }
        if (pcmData) heap_caps_free(pcmData);
        if (mic_rx_chan != NULL) {
          i2s_channel_disable(mic_rx_chan);
          vTaskDelay(pdMS_TO_TICKS(10));
          esp_err_t enable_err = i2s_channel_enable(mic_rx_chan);
          if (enable_err != ESP_OK) {
            Serial.printf("Lỗi enable lại I2S channel: %s\n", esp_err_to_name(enable_err));
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void SendDataMax7219(int HighByte, int LowByte) {
  // Gửi High Byte (địa chỉ)
  for (int i = 7; i >= 0; i--) {
    digitalWrite(DataOUT, bitRead(HighByte, i));
    delayMicroseconds(10);
    digitalWrite(CLK, HIGH);
    delayMicroseconds(10);
    digitalWrite(CLK, LOW);
    delayMicroseconds(10);
  }

  // Gửi Low Byte (dữ liệu)
  for (int i = 7; i >= 0; i--) {
    digitalWrite(DataOUT, bitRead(LowByte, i));
    delayMicroseconds(10);
    digitalWrite(CLK, HIGH);
    delayMicroseconds(10);
    digitalWrite(CLK, LOW);
    delayMicroseconds(10);
  }

  // Load dữ liệu
  digitalWrite(Load, HIGH);
  delayMicroseconds(10);
  digitalWrite(Load, LOW);
  delayMicroseconds(10);
}

void ShowDigits7SegmentLED(int d3, int d2, int d1, int d0) {
  SendDataMax7219(0x01, digits_seg0[d3]);
  SendDataMax7219(0x02, digits_seg1[d2]);
  SendDataMax7219(0x03, digits_seg2[d1]);
  SendDataMax7219(0x04, digits_seg3[d0]);
}

void buzzerOn() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void buzzerOff() {
  digitalWrite(BUZZER_PIN, LOW);
}

void triggerAlarm() {
  for (int i = 0; i < 3; i++) {
    buzzerOn();
    vTaskDelay(pdMS_TO_TICKS(500));
    buzzerOff();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

bool getNTPTime(struct tm* timeinfo) {
  if (WiFi.status() == WL_CONNECTED) {
    configTime(7 * 3600, 0, "pool.ntp.org");
    if (getLocalTime(timeinfo)) {
      return true;
    }
  }
  return false;
}

bool syncTimeFromAPI() {
  struct tm timeinfo;
  if (getNTPTime(&timeinfo)) {
    DateTime ntpTime = DateTime(
      timeinfo.tm_year + 1900,
      timeinfo.tm_mon + 1,
      timeinfo.tm_mday,
      timeinfo.tm_hour,
      timeinfo.tm_min,
      timeinfo.tm_sec);

    // Lưu thời gian vào DS1307
    rtc.adjust(ntpTime);

    // Kiểm tra thời gian DS1307 sau khi lưu
    DateTime now = rtc.now();
    if (now.year() == ntpTime.year() && now.month() == ntpTime.month() && now.day() == ntpTime.day() && now.hour() == ntpTime.hour() && now.minute() == ntpTime.minute()) {
      Serial.println("Đồng bộ thời gian từ NTP vào DS1307 thành công: " + ntpTime.timestamp());
      return true;
    } else {
      Serial.println("Lỗi: Thời gian DS1307 không khớp sau khi đồng bộ! DS1307: " + now.timestamp() + ", NTP: " + ntpTime.timestamp());
      return false;
    }
  }
  Serial.println("Lỗi đồng bộ thời gian từ NTP");
  return false;
}

void readDS1307Time(byte& hour, byte& minute, byte& second) {
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 3);
  second = bcdToDec(Wire.read());
  minute = bcdToDec(Wire.read());
  hour = bcdToDec(Wire.read() & 0x3F);
}

byte bcdToDec(byte val) {
  return (val / 16 * 10 + val % 16);
}

WeatherIcon getWeatherIcon(const String& desc) {
  String descLower = desc;
  descLower.toLowerCase();
  //Serial.print("Weather condition (lowercase): ");
  //Serial.println(descLower);

  WeatherIcon icon = { 64, "u8g2_font_open_iconic_weather_6x_t" };  // Default: Cloud

  if (descLower.indexOf("clear") != -1) {
    icon.code = 69;  // Sun
    icon.font = "u8g2_font_open_iconic_weather_6x_t";
  } else if (descLower.indexOf("cloud") != -1) {
    if (descLower.indexOf("few") != -1 || descLower.indexOf("scattered") != -1) {
      icon.code = 65;  // Sun Cloud
      icon.font = "u8g2_font_open_iconic_weather_6x_t";
    } else {
      icon.code = 64;  // Cloud
      icon.font = "u8g2_font_open_iconic_weather_6x_t";
    }
  } else if (descLower.indexOf("rain") != -1) {
    icon.code = 67;  // Rain
    icon.font = "u8g2_font_open_iconic_weather_6x_t";
  } else if (descLower.indexOf("thunder") != -1) {
    icon.code = 67;  // Thunder
    icon.font = "u8g2_font_open_iconic_embedded_6x_t";
  } else if (descLower.indexOf("snow") != -1 || descLower.indexOf("mist") != -1 || descLower.indexOf("fog") != -1 || descLower.indexOf("haze") != -1) {
    icon.code = 64;  // Cloud
    icon.font = "u8g2_font_open_iconic_weather_6x_t";
  }
  return icon;
}

void displayTask(void* pvParameters) {
  static DateTime lastTime = DateTime((uint32_t)0);
  bool alarmTriggered = false;
  String lastTemperature = "";
  String lastWeatherDesc = "";
  char lastTimeStr[9] = "";
  char lastSecStr[4] = "";
  char lastDateStr[10] = "";
  char lastTimerStr[16] = "";
  char lastAlarmStr[16] = "";
  int lastMode = 1;

  pinMode(DataOUT, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(Load, OUTPUT);
  digitalWrite(Load, LOW);

  SendDataMax7219(0x0C, 0x01);  // Bật hiển thị
  SendDataMax7219(0x0A, 0x02);  // Độ sáng
  SendDataMax7219(0x0B, 0x03);  // 4 chữ số
  SendDataMax7219(0x09, 0x00);  // Tắt chế độ giải mã
  SendDataMax7219(0x01, 0x00);  // Xóa LED 0
  SendDataMax7219(0x02, 0x00);  // Xóa LED 1
  SendDataMax7219(0x03, 0x00);  // Xóa LED 2
  SendDataMax7219(0x04, 0x00);  // Xóa LED 3

  for (;;) {
    DateTime now = rtc.now();
    bool wifiConnected = WiFi.isConnected();

    if (isOnline) {
      lastTime = now;
    } else {
      now = rtc.now();
    }

    String weather;
    if (xSemaphoreTake(weatherSemaphore, portMAX_DELAY) == pdTRUE) {
      weather = weatherInfo;
      xSemaphoreGive(weatherSemaphore);
    }

    String temperature = "--";
    String weather_desc = "--";
    int commaIndex = weather.indexOf(", ");
    int colonIndex = weather.indexOf(": ");
    if (colonIndex != -1 && commaIndex != -1) {
      String tempStr = weather.substring(colonIndex + 2, commaIndex);
      temperature = tempStr.substring(0, tempStr.indexOf("C")) + " C";
      weather_desc = weather.substring(commaIndex + 2);
      weather_desc.replace("clear sky", "clear");
      weather_desc.replace("few clouds", "few clouds");
      weather_desc.replace("scattered clouds", "scattered clouds");
      weather_desc.replace("broken clouds", "clouds");
      weather_desc.replace("overcast clouds", "clouds");
      weather_desc.replace("light rain", "rain");
      weather_desc.replace("moderate rain", "rain");
      weather_desc.replace("heavy rain", "rain");
    }

    char timeStr[9];
    char secStr[4];
    sprintf(timeStr, "%02d:%02d", now.hour(), now.minute());
    sprintf(secStr, ":%02d", now.second());
    char dateStr[10];
    sprintf(dateStr, "%02d/%02d/%02d", now.day(), now.month(), now.year() % 100);

    const char* day = daysOfWeek[now.dayOfTheWeek()];

    char timerStr[16];
    if (timerEnabled && now.unixtime() < timerEndTime) {
      int secondsLeft = timerEndTime - now.unixtime();
      int minutes = secondsLeft / 60;
      int seconds = secondsLeft % 60;
      snprintf(timerStr, sizeof(timerStr), "Timer: %02d:%02d", minutes, seconds);
    } else if (timer5MinEnabled && now.unixtime() < timer5MinEnd) {
      int secondsLeft = timer5MinEnd - now.unixtime();
      int minutes = secondsLeft / 60;
      int seconds = secondsLeft % 60;
      snprintf(timerStr, sizeof(timerStr), "Timer: %02d:%02d", minutes, seconds);
    } else if (timer2MinEnabled && now.unixtime() < timer2MinEnd) {
      int secondsLeft = timer2MinEnd - now.unixtime();
      int minutes = secondsLeft / 60;
      int seconds = secondsLeft % 60;
      snprintf(timerStr, sizeof(timerStr), "Timer: %02d:%02d", minutes, seconds);
    } else {
      snprintf(timerStr, sizeof(timerStr), "Timer: Off");
    }

    char alarmStr[16];
    if (alarmEnabled && alarmHour >= 0) {
      snprintf(alarmStr, sizeof(alarmStr), "Alarm: %02d:%02d", alarmHour, alarmMinute);
    } else {
      snprintf(alarmStr, sizeof(alarmStr), "Alarm: Off");
    }

    if (alarmEnabled && now.hour() == alarmHour && now.minute() == alarmMinute && now.second() == 0) {
      if (!alarmTriggered) {
        triggerAlarm();
        alarmTriggered = true;
        if (xSemaphoreTake(displaySemaphore, portMAX_DELAY) == pdTRUE) {
          alarmEnabled = false;
          alarmHour = -1;
          alarmMinute = 0;
          currentCommand = "Báo thức đã kích hoạt";
          xSemaphoreGive(displaySemaphore);
        }
        if (WiFi.status() == WL_CONNECTED) {
          char alarmText[64];
          snprintf(alarmText, sizeof(alarmText), "Báo thức lúc %d giờ %d phút", now.hour(), now.minute());
          playTTS(alarmText, "vi");
        }
      }
    } else {
      alarmTriggered = false;
    }

    if (timerEnabled && now.unixtime() >= timerEndTime) {
      triggerAlarm();
      timerEnabled = false;
      if (WiFi.status() == WL_CONNECTED) {
        playTTS("Hẹn giờ đã kết thúc.", "vi");
      } else {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
      }
    }

    if (timer5MinEnabled && now.unixtime() >= timer5MinEnd) {
      triggerAlarm();
      timer5MinEnabled = false;
      if (WiFi.status() == WL_CONNECTED) {
        playTTS("Hẹn giờ năm phút đã kết thúc.", "vi");
      } else {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
      }
    }

    if (timer2MinEnabled && now.unixtime() >= timer2MinEnd) {
      triggerAlarm();
      timer2MinEnabled = false;
      if (WiFi.status() == WL_CONNECTED) {
        playTTS("Hẹn giờ hai phút đã kết thúc.", "vi");
      } else {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
      }
    }

    int mode = (timerEnabled || timer5MinEnabled || timer2MinEnabled || displayMode2) ? 2 : 1;

    // Sửa lỗi DAO thành 0
    if (strcmp(timeStr, lastTimeStr) != 0 || strcmp(secStr, lastSecStr) != 0 || strcmp(dateStr, lastDateStr) != 0 || strcmp(timerStr, lastTimerStr) != 0 || strcmp(alarmStr, lastAlarmStr) != 0 || temperature != lastTemperature || weather_desc != lastWeatherDesc || mode != lastMode) {
      u8g2.clearBuffer();
      u8g2.drawFrame(0, 0, 128, 64);

      if (mode == 1) {
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(5, 12, day);
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(5, 37, temperature.c_str());
      } else {
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.drawStr(5, 12, dateStr);
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(5, 37, timeStr);
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(55, 37, secStr);
      }

      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(5, 52, timerStr);

      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(5, 60, alarmStr);

      WeatherIcon icon = getWeatherIcon(weather_desc);
      if (strcmp(icon.font, "u8g2_font_open_iconic_weather_6x_t") == 0) {
        u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      } else {
        u8g2.setFont(u8g2_font_open_iconic_embedded_6x_t);
      }
      u8g2.drawGlyph(70, 56, icon.code);

      u8g2.sendBuffer();

      strcpy(lastTimeStr, timeStr);
      strcpy(lastSecStr, secStr);
      strcpy(lastDateStr, dateStr);
      strcpy(lastTimerStr, timerStr);
      strcpy(lastAlarmStr, alarmStr);
      lastTemperature = temperature;
      lastWeatherDesc = weather_desc;
      lastMode = mode;
    }

    // Cập nhật LED 7 đoạn
    if (timerEnabled && now.unixtime() < timerEndTime) {
      int secondsLeft = timerEndTime - now.unixtime();
      if (secondsLeft < 0) secondsLeft = 0;
      int displayMin = secondsLeft / 60;
      int displaySec = secondsLeft % 60;
      Digit3 = displayMin / 10;
      Digit2 = displayMin % 10;
      Digit1 = displaySec / 10;
      Digit0 = displaySec % 10;
    } else if (timer5MinEnabled && now.unixtime() < timer5MinEnd) {
      int secondsLeft = timer5MinEnd - now.unixtime();
      if (secondsLeft < 0) secondsLeft = 0;
      int displayMin = secondsLeft / 60;
      int displaySec = secondsLeft % 60;
      Digit3 = displayMin / 10;
      Digit2 = displayMin % 10;
      Digit1 = displaySec / 10;
      Digit0 = displaySec % 10;
    } else if (timer2MinEnabled && now.unixtime() < timer2MinEnd) {
      int secondsLeft = timer2MinEnd - now.unixtime();
      if (secondsLeft < 0) secondsLeft = 0;
      int displayMin = secondsLeft / 60;
      int displaySec = secondsLeft % 60;
      Digit3 = displayMin / 10;
      Digit2 = displayMin % 10;
      Digit1 = displaySec / 10;
      Digit0 = displaySec % 10;
    } else {
      int displayHour = now.hour();
      int displayMin = now.minute();
      Digit3 = displayHour / 10;
      Digit2 = displayHour % 10;
      Digit1 = displayMin / 10;
      Digit0 = displayMin % 10;
    }

    ShowDigits7SegmentLED(Digit3, Digit2, Digit1, Digit0);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
void uartTask(void* pvParameters) {
  for (;;) {
    if (Serial1.available()) {
      char receivedChar = Serial1.read();
      Serial.printf("Nhận ký tự UART: %c\n", receivedChar);
      if (isOnline) {
        // Online mode: Handle 'a' for voice trigger with buzzer
        switch (receivedChar) {
          case 'a':
            digitalWrite(BUZZER_PIN, HIGH);   // Bật buzzer
            delay(200);                       // Kêu 200ms
            digitalWrite(BUZZER_PIN, LOW);    // Tắt buzzer
            xTaskNotifyGive(wifiTaskHandle);  // Trigger voice recording
            Serial.println("Online: Nhận 'a', buzzer kêu 200ms, kích hoạt ghi âm");
            break;
          default:
            break;
        }
      } else {
        // Offline mode: Handle UART commands
        switch (receivedChar) {
          case 'a':
            digitalWrite(BUZZER_PIN, LOW);    // Tắt buzzer
            xTaskNotifyGive(wifiTaskHandle);  // Trigger voice recording attempt
            Serial.println("Offline: Nhận 'a', buzzer tắt");
            break;
          case 'g':
            if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
              DateTime now = rtc.now();
              displayMode2 = true;  // Switch to mode 2
              Serial.println("Offline: Nhận 'g', chuyển OLED chế độ 2");
              xSemaphoreGive(displaySemaphore);
            }
            break;
          case 'h':
            if (xSemaphoreTake(weatherSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
              xSemaphoreGive(weatherSemaphore);
            }
            break;
          case 'i':
            if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
              alarmEnabled = true;
              alarmHour = 6;
              alarmMinute = 0;
              Serial.println("Offline: Nhận 'i', báo thức 6:00");
              xSemaphoreGive(displaySemaphore);
            }
            break;
          case 'k':
            if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
              DateTime now = rtc.now();
              timer5MinEnabled = true;
              timer5MinEnd = now.unixtime() + 300;  // 5 minutes
              timer2MinEnabled = false;             // Hủy hẹn giờ 2 phút
              displayMode2 = true;                  // Switch to mode 2
              digitalWrite(BUZZER_PIN, HIGH);       // Bật buzzer
              delay(200);                           // Kêu 200ms
              digitalWrite(BUZZER_PIN, LOW);        // Tắt buzzer
              // Reset LED 7 đoạn
              ShowDigits7SegmentLED(0, 5, 0, 0);  // Hiển thị 05:00
              Serial.println("Offline: Nhận 'k', hẹn giờ 5 phút, hủy hẹn giờ 2 phút, buzzer kêu, reset LED 7 đoạn");
              xSemaphoreGive(displaySemaphore);
            }
            break;
          case 'l':
            if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
              DateTime now = rtc.now();
              timer2MinEnabled = true;
              timer2MinEnd = now.unixtime() + 120;  // 2 minutes
              timer5MinEnabled = false;             // Hủy hẹn giờ 5 phút
              displayMode2 = true;                  // Switch to mode 2
              digitalWrite(BUZZER_PIN, HIGH);       // Bật buzzer
              delay(200);                           // Kêu 200ms
              digitalWrite(BUZZER_PIN, LOW);        // Tắt buzzer
              // Reset LED 7 đoạn
              ShowDigits7SegmentLED(0, 2, 0, 0);  // Hiển thị 02:00
              Serial.println("Offline: Nhận 'l', hẹn giờ 2 phút, hủy hẹn giờ 5 phút, buzzer kêu, reset LED 7 đoạn");
              xSemaphoreGive(displaySemaphore);
            }
            break;
          default:
            break;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void checkTaskStack(void* pvParameters) {
  for (;;) {
    Serial.printf("WiFiTask stack high water mark: %u bytes\n", uxTaskGetStackHighWaterMark(wifiTaskHandle));
    Serial.printf("DisplayTask stack high water mark: %u bytes\n", uxTaskGetStackHighWaterMark(displayTaskHandle));
    Serial.printf("UartTask stack high water mark: %u bytes\n", uxTaskGetStackHighWaterMark(uartTaskHandle));
    vTaskDelay(pdMS_TO_TICKS(60000));  // Kiểm tra mỗi 60 giây
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Free heap tại setup: %d bytes\n", ESP.getFreeHeap());

  Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  leds.begin();
  leds.clear();
  leds.show();

  Wire.begin(12, 13);
  u8g2.begin();
  if (!rtc.begin()) {
    Serial.println("RTC failed to begin!");
    while (1)
      ;
  }

  // Kích hoạt chế độ pin cho DS1307
  enableRTCBatteryMode();

  DateTime now = rtc.now();
  if (!rtc.isrunning()) {
    Serial.println("RTC không chạy, đặt thời gian: 19:03, 18/05/2025");
    rtc.adjust(DateTime(2025, 5, 18, 19, 3, 0));
  } else if (now.year() < 2020 || now.year() > 2030) {
    Serial.println("Thời gian RTC không hợp lệ: " + now.timestamp() + ", đặt thời gian mặc định");
    rtc.adjust(DateTime(2025, 5, 18, 19, 3, 0));
  } else {
    Serial.println("RTC đang chạy, sử dụng thời gian từ DS1307: " + now.timestamp());
  }

  audio = new Audio();
  if (audio) {
    audio->setPinout(I2S_SPK_BCLK, I2S_SPK_LRC, I2S_SPK_DOUT, I2S_NUM_1);
    audio->setVolume(70);
    Serial.println("Đối tượng Audio cho loa đã được khởi tạo.");
  } else {
    Serial.println("Lỗi khởi tạo đối tượng Audio cho loa!");
  }

  setupI2S_new_driver();
  if (!micI2SInitialized) {
    Serial.println("LỖI NGHIÊM TRỌNG: Không thể khởi tạo I2S cho micro trong setup!");
  }

  // Tạo các semaphore với kiểm tra lỗi chi tiết
  forecastSemaphore = xSemaphoreCreateMutex();
  if (forecastSemaphore == NULL) {
    Serial.println("Lỗi tạo forecastSemaphore!");
    while (1)
      ;
  }

  weatherSemaphore = xSemaphoreCreateMutex();
  if (weatherSemaphore == NULL) {
    Serial.println("Lỗi tạo weatherSemaphore!");
    while (1)
      ;
  }

  commandSemaphore = xSemaphoreCreateMutex();
  if (commandSemaphore == NULL) {
    Serial.println("Lỗi tạo commandSemaphore!");
    while (1)
      ;
  }

  displaySemaphore = xSemaphoreCreateMutex();
  if (displaySemaphore == NULL) {
    Serial.println("Lỗi tạo displaySemaphore!");
    while (1)
      ;
  }

  // Khởi tạo giá trị mặc định cho forecastData
  for (int i = 0; i < 3; i++) {
    forecastData[i].temp = 0.0;
    forecastData[i].desc = "N/A";
    forecastData[i].timestamp = 0;
    forecastData[i].pop = 0.0;  // Khởi tạo xác suất mưa
  }

  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 16384, NULL, 2, &wifiTaskHandle, 0);
  xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, &displayTaskHandle, 1);
  xTaskCreatePinnedToCore(uartTask, "UartTask", 2048, NULL, 1, &uartTaskHandle, 1);
  xTaskCreatePinnedToCore(checkTaskStack, "CheckStackTask", 2048, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}