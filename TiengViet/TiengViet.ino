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
const token_api
const char* ssid = "Fire And Blood";
const char* password = "12345678";
const char* viettel_api = "https://viettelai.vn/asr/recognize";
const char* viettel_token = token_api;
const char* weather_api = "http://api.openweathermap.org/data/2.5/weather?lat=21.0285&lon=105.8542&appid=f5044cf6ac87465b7e240e4cbbf7d3f2&units=metric";
const char* forecast_api = "http://api.openweathermap.org/data/2.5/forecast?lat=21.0285&lon=105.8542&appid=f5044cf6ac87465b7e240e4cbbf7d3f2&units=metric";

#define SUN 0
#define SUN_CLOUD 1
#define CLOUD 2
#define RAIN 3
#define THUNDER 4

#define I2S_WS 41
#define I2S_SCK 42
#define I2S_SD 40
#define RATE 16000
#define CHUNK 1024
#define RECORD_SECONDS 3

#define I2S_SPK_BCLK 5
#define I2S_SPK_LRC 4
#define I2S_SPK_DOUT 15
#define BUZZER_PIN 14

#define UART_NUM UART_NUM_1
#define UART_TX_PIN 17
#define UART_RX_PIN 18
#define UART_BAUD_RATE 115200

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 13, 12);
RTC_DS1307 rtc;
int DataOUT = 21;
int CLK = 47;
int Load = 20;

String currentCommand = "Chua co lenh";
String weatherInfo = "Weather: N/A";
SemaphoreHandle_t commandSemaphore;
SemaphoreHandle_t weatherSemaphore;
SemaphoreHandle_t forecastSemaphore;
SemaphoreHandle_t displaySemaphore;
SemaphoreHandle_t appointmentSemaphore;

bool isOnline = true;
bool lastWiFiState = false;
bool timer2MinEnabled = false;
uint32_t timer2MinEnd = 0;
bool displayMode2 = false;
TickType_t lastWiFiCheck = 0;
const TickType_t wifiDebounceTime = pdMS_TO_TICKS(5000);
bool alarmEnabled = false;
int alarmHour = -1;
int alarmMinute = 0;
bool timerEnabled = false;
uint32_t timerEndTime = 0;
bool timer5MinEnabled = false;
uint32_t timer5MinEnd = 0;

const char* daysOfWeek[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

struct WeatherIcon {
  uint16_t code;
  const char* font;
};
struct DailyForecast {
  float temp;
  String desc;
  uint32_t timestamp;
  float pop;  //0-1
};
DailyForecast forecastData[3];
struct Appointment {
  uint32_t dateUnix;
  char message[64];
  bool active;
};
Appointment appointment = { 0, "", false };

i2s_chan_handle_t mic_rx_chan = NULL;
Audio* audio = NULL;
bool micI2SInitialized = false;

TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t uartTaskHandle = NULL;

int Digit3 = 0;
int Digit2 = 0;
int Digit1 = 0;
int Digit0 = 0;

const byte digits_seg0[10] = { 0b11111100, 0b00110000, 0b11101001, 0b01111001, 0b00110101, 0b01011101, 0b11011101, 0b01110000, 0b11111101, 0b01111101 };
const byte digits_seg1[10] = { 0b11111101, 0b00110001, 0b11101011, 0b01111011, 0b00110111, 0b01011111, 0b11011111, 0b01110001, 0b11111111, 0b01111111 };
const byte digits_seg2[10] = { 0b11111101, 0b10000101, 0b11101011, 0b11001111, 0b10010111, 0b01011111, 0b01111111, 0b10001101, 0b11111111, 0b11011111 };
const byte digits_seg3[10] = { 0b11111100, 0b10000100, 0b11101010, 0b11001110, 0b10010110, 0b01011110, 0b01111110, 0b10001100, 0b11111110, 0b11011110 };

WeatherIcon getWeatherIcon(const String& desc) {
  String descLower = desc;
  descLower.toLowerCase();
  //Serial.print("Weather condition (lowercase): ");
  //Serial.println(descLower);
  WeatherIcon icon = { 64, "u8g2_font_open_iconic_weather_6x_t" };
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
    icon.code = 67;
    icon.font = "u8g2_font_open_iconic_embedded_6x_t";
  } else if (descLower.indexOf("snow") != -1 || descLower.indexOf("mist") != -1 || descLower.indexOf("fog") != -1 || descLower.indexOf("haze") != -1) {
    icon.code = 64;  // Cloud
    icon.font = "u8g2_font_open_iconic_weather_6x_t";
  }
  return icon;
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
byte bcdToDec(byte val) {
  return (val / 16 * 10 + val % 16);
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
void enableRTCBatteryMode() {
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  uint8_t seconds = Wire.read();
  if (seconds & 0x80) {
    Wire.beginTransmission(0x68);
    Wire.write(0x00);
    Wire.write(seconds & 0x7F);
    Wire.endTransmission();
    Serial.println("Đã bật bộ dao động DS1307");
  }
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
void connectWiFi() {
  WiFi.begin(ssid, password);
  int timeout = 10;
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    timeout--;
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
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
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
      readErrorCount = 0;
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
  audio->setVolume(100);
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

  const char* vietnameseNumbers[] = {
    "khong", "mot", "hai", "ba", "bon", "nam", "sau", "bay", "tam", "chin", "muoi", "muoi mot", "muoi hai", "muoi ba", "muoi bon", "muoi nam", "muoi sau",
    "muoi bay", "muoi tam", "muoi chin", "hai muoi", "hai muoi mot", "hai muoi hai", "hai muoi ba", "hai muoi bon", "hai muoi nam", "hai muoi sau", "hai muoi bay",
    "hai muoi tam", "hai muoi chin", "ba muoi", "ba muoi mot", "ba muoi hai", "ba muoi ba", "ba muoi bon", "ba muoi nam", "ba muoi sau", "ba muoi bay",
    "ba muoi tam", "ba muoi chin", "bon muoi", "bon muoi mot", "bon muoi hai", "bon muoi ba", "bon muoi bon", "bon muoi nam", "bon muoi sau", "bon muoi bay",
    "bon muoi tam", "bon muoi chin", "nam muoi", "nam muoi mot", "nam muoi hai", "nam muoi ba", "nam muoi bon", "nam muoi nam", "nam muoi sau", "nam muoi bay",
    "nam muoi tam", "nam muoi chin", "sau muoi", "sau muoi mot", "sau muoi hai", "sau muoi ba", "sau muoi bon", "sau muoi nam", "sau muoi sau", "sau muoi bay",
    "sau muoi tam", "sau muoi chin", "bay muoi", "bay muoi mot", "bay muoi hai", "bay muoi ba", "bay muoi bon", "bay muoi nam", "bay muoi sau", "bay muoi bay",
    "bay muoi tam", "bay muoi chin", "tam muoi", "tam muoi mot", "tam muoi hai", "tam muoi ba", "tam muoi bon", "tam muoi nam", "tam muoi sau", "tam muoi bay",
    "tam muoi tam", "tam muoi chin", "chin muoi", "chin muoi mot", "chin muoi hai", "chin muoi ba", "chin muoi bon", "chin muoi nam", "chin muoi sau", "chin muoi bay", "chin muoi tam", "chin muoi chin"
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

// Hàm tách số cho báo thức
bool extractAlarmTime(const String& transcript, int& hour, int& minute) {
  String lowerTranscript = removeVietnameseDiacritics(transcript);
  lowerTranscript.toLowerCase();
  Serial.println("Lower transcript (alarm): [" + lowerTranscript + "]");
  int foundHour = 0;
  int foundMinute = 0;

  const char* vietnameseNumbers[] = {
    "khong", "mot", "hai", "ba", "bon", "nam", "sau", "bay", "tam", "chin", "muoi", "muoi mot", "muoi hai", "muoi ba", "muoi bon", "muoi lam", "muoi sau",
    "muoi bay", "muoi tam", "muoi chin", "hai muoi", "hai muoi mot", "hai muoi hai", "hai muoi ba", "hai muoi tu", "hai muoi lam", "hai muoi sau",
    "hai muoi bay", "hai muoi tam", "hai muoi chin", "ba muoi", "ba muoi mot", "ba muoi hai", "ba muoi ba", "ba muoi bon", "ba muoi lam", "ba muoi sau",
    "ba muoi bay", "ba muoi tam", "ba muoi chin", "bon muoi", "bon muoi mot", "bon muoi hai", "bon muoi ba", "bon muoi bon", "bon muoi lam", "bon muoi sau",
    "bon muoi bay", "bon muoi tam", "bon muoi chin", "nam muoi", "nam muoi mot", "nam muoi hai", "nam muoi ba", "nam muoi bon", "nam muoi lam", "nam muoi sau", "nam muoi bay", "nam muoi tam", "nam muoi chin"
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
bool extractAppointmentDate(const String& transcript, int& day, int& month) {
  String lowerTranscript = removeVietnameseDiacritics(transcript);
  lowerTranscript.toLowerCase();
  lowerTranscript.trim();
  lowerTranscript.replace(".", "");
  Serial.println("Lower transcript (appointment): [" + lowerTranscript + "]");
  int foundDay = -1;
  int foundMonth = -1;
  // Tìm "lich" hoặc "ngay" để xác định điểm bắt đầu
  int startIndex = lowerTranscript.indexOf("lich");
  if (startIndex == -1) {
    startIndex = lowerTranscript.indexOf("ngay");
  }
  Serial.printf("startIndex sau lich/ngay: %d\n", startIndex);
  // Nếu tìm thấy "lich" hoặc "ngay", bỏ qua từ này và khoảng trắng
  if (startIndex != -1) {
    startIndex = lowerTranscript.indexOf(" ", startIndex);
    if (startIndex == -1) {
      startIndex = lowerTranscript.length();
    }
  } else {
    startIndex = 0;
  }
  Serial.printf("startIndex sau xử lý khoảng trắng: %d\n", startIndex);
  // Lấy chuỗi sau "lich" hoặc "ngay"
  String datePart = lowerTranscript.substring(startIndex);
  datePart.trim();
  Serial.println("Date part: [" + datePart + "]");
  const char* vietnameseNumbers[] = {
    "khong", "mot", "hai", "ba", "bon", "nam", "sau", "bay", "tam", "chin", "muoi", "muoi mot", "muoi hai", "muoi ba", "muoi bon", "muoi nam", "muoi sau", "muoi bay", "muoi tam", "muoi chin",
    "hai muoi", "hai muoi mot", "hai muoi hai", "hai muoi ba", "hai muoi bon", "hai muoi nam", "hai muoi sau", "hai muoi bay", "hai muoi tam", "hai muoi chin", "ba muoi", "ba muoi mot"
  };
  // Tìm "/" để xử lý định dạng DD/MM
  int slashIndex = datePart.indexOf("/");
  if (slashIndex != -1) {
    String dayStr = datePart.substring(0, slashIndex);
    String monthStr = datePart.substring(slashIndex + 1);
    dayStr.trim();
    monthStr.trim();
    Serial.println("Day string trước làm sạch: [" + dayStr + "], Month string trước làm sạch: [" + monthStr + "]");
    // Làm sạch chuỗi ngày
    String cleanDayStr = "";
    for (int i = 0; i < dayStr.length(); i++) {
      if (isDigit(dayStr[i])) {
        cleanDayStr += dayStr[i];
      }
    }
    // Làm sạch chuỗi tháng
    String cleanMonthStr = "";
    for (int i = 0; i < monthStr.length(); i++) {
      if (!isDigit(monthStr[i])) {
        cleanMonthStr = monthStr.substring(0, i);
        break;
      }
      cleanMonthStr += monthStr[i];
    }
    Serial.println("Day string sau làm sạch: [" + cleanDayStr + "], Month string sau làm sạch: [" + cleanMonthStr + "]");
    if (cleanDayStr.toInt() > 0 && cleanDayStr.toInt() <= 31) {
      foundDay = cleanDayStr.toInt();
    }
    if (cleanMonthStr.toInt() > 0 && cleanMonthStr.toInt() <= 12) {
      foundMonth = cleanMonthStr.toInt();
    }
    Serial.printf("Parsed day: %d, month: %d\n", foundDay, foundMonth);
  } else {
    // Xử lý định dạng chữ (ví dụ: "21 thang 5")
    String words[10];
    int wordCount = 0;
    int start = 0;
    for (int i = 0; i <= datePart.length(); i++) {
      if (i == datePart.length() || datePart[i] == ' ') {
        if (start < i) {
          words[wordCount] = datePart.substring(start, i);
          words[wordCount].trim();
          Serial.printf("Từ %d: [%s]\n", wordCount, words[wordCount].c_str());
          wordCount++;
        }
        start = i + 1;
      }
    }
    // Tìm ngày và tháng
    int thangIndex = -1;
    for (int i = 0; i < wordCount; i++) {
      if (words[i] == "thang") {
        thangIndex = i;
        break;
      }
    }
    if (thangIndex != -1) {
      // Tìm ngày trước "thang"
      for (int i = thangIndex - 1; i >= 0; i--) {
        for (int j = 1; j <= 31; j++) {
          if (words[i] == vietnameseNumbers[j] || words[i].toInt() == j) {
            foundDay = j;
            break;
          }
        }
        if (foundDay != -1) break;
      }
      // Tìm tháng sau "thang"
      for (int i = thangIndex + 1; i < wordCount; i++) {
        for (int j = 1; j <= 12; j++) {
          if (words[i] == vietnameseNumbers[j] || words[i].toInt() == j) {
            foundMonth = j;
            break;
          }
        }
        if (foundMonth != -1) break;
      }
    } else {
      // Không tìm thấy "thang", thử tìm hai số liên tiếp
      for (int i = 0; i < wordCount - 1; i++) {
        int num1 = words[i].toInt();
        int num2 = words[i + 1].toInt();
        if (num1 >= 1 && num1 <= 31 && num2 >= 1 && num2 <= 12) {
          foundDay = num1;
          foundMonth = num2;
          break;
        }
        for (int j = 1; j <= 31; j++) {
          if (words[i] == vietnameseNumbers[j]) {
            foundDay = j;
            break;
          }
        }
        if (foundDay != -1) {
          for (int j = 1; j <= 12; j++) {
            if (words[i + 1] == vietnameseNumbers[j]) {
              foundMonth = j;
              break;
            }
          }
          if (foundMonth != -1) break;
        }
      }
    }
    Serial.printf("Parsed day: %d, month: %d\n", foundDay, foundMonth);
  }
  if (foundDay == -1 || foundMonth == -1) {
    Serial.println("Không tìm thấy ngày hoặc tháng hợp lệ");
    return false;
  }
  day = foundDay;
  month = foundMonth;
  Serial.printf("Trích xuất ngày hẹn: %d/%d\n", day, month);
  return true;
}
// Hàm loại bỏ dấu tiếng Việt
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
//
// Hàm chuyển đổi thời gian sang văn bản tiếng Việt
// Hàm này chuyển đổi giờ, phút, ngày, tháng, năm thành chuỗi văn bản tiếng Việt
// Ví dụ: 10:30, 21/5/2023 → "Bây giờ là mười giờ ba mươi phút, ngày hai mươi mốt tháng năm năm hai
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

      for (int i = 0; i < 3; i++) {
        uint32_t targetMidnight = todayMidnight + (i + 1) * 86400;
        uint32_t targetNoon = targetMidnight + 12 * 3600;  // 12:00 ngày tiếp theo
        if (dt >= targetMidnight && dt < targetMidnight + 86400) {
          int64_t timeDiff = (int64_t)dt - (int64_t)targetNoon;
          if (timeDiff < 0) timeDiff = -timeDiff;
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

    if (xSemaphoreTake(forecastSemaphore, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 3; i++) {
        forecastData[i].timestamp = candidates[i].timestamp;
        forecastData[i].temp = candidates[i].temp;
        forecastData[i].desc = candidates[i].desc;
        forecastData[i].pop = candidates[i].pop;

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
// Hàm gửi dữ liệu PCM đến API Viettel xử lý tiếng viẹte
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
  bool isAppointmentRecording = false;
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Phản hồi API: " + response);

    String transcript = parseTranscript(response);
    if (transcript == "") {
      Serial.println("Lỗi: Transcript rỗng, không nhận diện được giọng nói");
      // Serial1.print('c');
      responseText = "Không nhận diện được giọng nói, vui lòng thử lại";
    } else {
      Serial.print("Transcript: ");
      Serial.println(transcript);

      // Loại bỏ dấu chấm cuối và khoảng trắng thừa
      transcript.trim();
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

      if (lowerTranscript.indexOf("do") != -1) {
        Serial.println("Bật đèn đỏ");
        responseText = "Đã bật đèn đỏ";
        Serial1.print('c');
      } else if (lowerTranscript.indexOf("vang") != -1) {
        Serial.println("Bật đèn vàng");
        responseText = "Đã bật đèn vàng";
        Serial1.print('d');
      } else if (lowerTranscript.indexOf("xanh") != -1) {
        Serial.println("Bật đèn xanh");
        responseText = "Đã bật đèn xanh";
        Serial1.print('e');
      } else if (lowerTranscript.indexOf("trang") != -1) {
        Serial.println("Bật đèn trắng");
        responseText = "Đã bật đèn trắng";
        Serial1.print('f');
      } else if (lowerTranscript.indexOf("tat") != -1) {
        Serial.println("Tắt đèn");
        responseText = "Đã tắt đèn";
        Serial1.print('b');
      } else if (lowerTranscript.indexOf("bay gio") != -1 || lowerTranscript.indexOf("may gio") != -1 || lowerTranscript.indexOf("may") != -1) {
        Serial.println("Hỏi giờ");
        if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
          DateTime now = rtc.now();
          char timeText[150];
          timeToVietnameseText(now.hour(), now.minute(), now.day(), now.month(), now.year(), timeText, sizeof(timeText));
          responseText = timeText;
          xSemaphoreGive(displaySemaphore);
          Serial1.print('m');
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
          Serial1.print('n');
        }
      } else if ((lowerTranscript.indexOf("bao thuc") != -1 || lowerTranscript.indexOf("thuc") != -1 || lowerTranscript.indexOf("luc") != -1) && (lowerTranscript.indexOf("huy") == -1 && lowerTranscript.indexOf("tat") == -1)) {
        Serial.println("Báo thức");
        int hour, minute;
        if (extractAlarmTime(transcript, hour, minute)) {
          if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            alarmEnabled = true;
            alarmHour = hour;
            alarmMinute = minute;
            currentCommand = "Đã đặt báo thức vào lúc " + String(hour) + ":" + (minute < 10 ? "0" : "") + String(minute) + " phút";
            responseText = currentCommand;
            Serial.printf("Đã đặt báo thức: %d:%02d\n", hour, minute);
            xSemaphoreGive(displaySemaphore);
            Serial1.print('p');
          }
        } else {
          responseText = "Không rõ thời gian báo thức, vui lòng thử lại";
        }
      } else if ((lowerTranscript.indexOf("hen") != -1 || lowerTranscript.indexOf("phut") != -1) && (lowerTranscript.indexOf("huy") == -1 && lowerTranscript.indexOf("tat") == -1 && lowerTranscript.indexOf("lich") == -1)) {
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
            Serial1.print('o');
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
              Serial1.print('o');
            }
          } else {
            responseText = "Vui lòng cung cấp thời gian hẹn giờ hợp lệ";
          }
        } else {
          responseText = "Vui lòng cung cấp thời gian hẹn giờ";
        }
      } else if (lowerTranscript.indexOf("du bao") != -1) {
        Serial.println("Hỏi dự báo thời tiết");
        DateTime now = rtc.now();
        DateTime tomorrow = DateTime(now.unixtime() + 86400);
        DateTime dayAfterTomorrow = DateTime(now.unixtime() + 2 * 86400);
        DateTime thirdDay = DateTime(now.unixtime() + 3 * 86400);

        if (lowerTranscript.indexOf("ngay mai") != -1 || lowerTranscript.indexOf("mot ngay") != -1) {
          Serial.println("Dự báo thời tiết ngày mai");
          if (xSemaphoreTake(forecastSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (forecastData[0].timestamp > 0) {
              String desc = translateWeatherDesc(forecastData[0].desc);
              char forecastTTS[150];
              snprintf(forecastTTS, sizeof(forecastTTS),
                       "Dự báo ngày mai, %d tháng %d, Hà Nội, %d độ C, %s, xác suất mưa %d phần trăm",
                       tomorrow.day(), tomorrow.month(),
                       (int)forecastData[0].temp, desc.c_str(), (int)(forecastData[0].pop * 100));
              responseText = forecastTTS;
              Serial1.print('n');
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
              char forecastTTS[150];
              snprintf(forecastTTS, sizeof(forecastTTS),
                       "Dự báo ngày kia, %d tháng %d, Hà Nội, %d độ C, %s, xác suất mưa %d phần trăm",
                       dayAfterTomorrow.day(), dayAfterTomorrow.month(),
                       (int)forecastData[1].temp, desc.c_str(), (int)(forecastData[1].pop * 100));
              responseText = forecastTTS;
              Serial1.print('n');
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
              char forecastTTS[150];
              snprintf(forecastTTS, sizeof(forecastTTS),
                       "Dự báo ngày %d tháng %d, Hà Nội, %d độ C, %s, xác suất mưa %d phần trăm",
                       thirdDay.day(), thirdDay.month(),
                       (int)forecastData[2].temp, desc.c_str(), (int)(forecastData[2].pop * 100));
              responseText = forecastTTS;
              Serial1.print('n');
            } else {
              responseText = "Không có dữ liệu dự báo ngày thứ 3, vui lòng thử lại sau";
            }
            xSemaphoreGive(forecastSemaphore);
          }
        } else {
          responseText = "Vui lòng chỉ định ngày dự báo (ngày mai, ngày kia, hoặc ba ngày)";
        }
      } else if ((lowerTranscript.indexOf("ngay") != -1 || lowerTranscript.indexOf("lich") != -1) && (lowerTranscript.indexOf("huy") == -1 && lowerTranscript.indexOf("tat") == -1 && lowerTranscript.indexOf("kiem") == -1 && lowerTranscript.indexOf("tra") == -1)) {
        Serial.println("Hẹn lịch");
        int day, month;
        if (extractAppointmentDate(transcript, day, month)) {
          if (xSemaphoreTake(appointmentSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
            DateTime now = rtc.now();
            int year = now.year();
            if (month < now.month()) year++;
            DateTime apptDate(year, month, day, 6, 0, 0);  // Đặt mặc định 6h
            appointment.dateUnix = apptDate.unixtime();
            appointment.active = true;
            appointment.message[0] = '\0';  // Reset message
            xSemaphoreGive(appointmentSemaphore);
            responseText = "Đã đặt lịch ngày " + String(day) + " tháng " + String(month) + ". Vui lòng nói lời hẹn.";
            Serial.printf("Đã đặt lịch: %d/%d/%d 06:00\n", day, month, year);
            isAppointmentRecording = true;  // Chuyển sang ghi âm lời hẹn
            Serial1.print('q');
          }
        } else {
          responseText = "Không rõ ngày hẹn, vui lòng thử lại";
        }
      } else if (lowerTranscript.indexOf("huy hen lich") != -1 || lowerTranscript.indexOf("tat lich") != -1 || lowerTranscript.indexOf("huy lich") != -1) {
        Serial.println("Hủy lịch");
        if (xSemaphoreTake(appointmentSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
          if (appointment.active) {
            appointment.dateUnix = 0;
            appointment.message[0] = '\0';
            appointment.active = false;
            responseText = "Đã hủy lịch hẹn.";
            Serial.println("Đã hủy lịch hẹn");
            Serial1.print('q');
          } else {
            responseText = "Không có lịch hẹn để hủy.";
            Serial.println("Không có lịch hẹn để hủy");
          }
          xSemaphoreGive(appointmentSemaphore);
        } else {
          responseText = "Lỗi khi hủy lịch, vui lòng thử lại";
        }
      } else if (lowerTranscript.indexOf("huy bao thuc") != -1 || lowerTranscript.indexOf("tat bao thuc") != -1) {
        Serial.println("Hủy báo thức");
        if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
          if (alarmEnabled) {
            alarmEnabled = false;
            alarmHour = -1;
            alarmMinute = 0;
            responseText = "Đã hủy báo thức.";
            Serial.println("Đã hủy báo thức");
            Serial1.print('p');
          } else {
            responseText = "Không có báo thức để hủy.";
            Serial.println("Không có báo thức để hủy");
          }
          xSemaphoreGive(displaySemaphore);
        } else {
          responseText = "Lỗi khi hủy báo thức, vui lòng thử lại";
        }
      } else if (lowerTranscript.indexOf("huy hen gio") != -1 || lowerTranscript.indexOf("tat hen gio") != -1) {
        Serial.println("Hủy hẹn giờ");
        if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
          if (timerEnabled || timer5MinEnabled || timer2MinEnabled) {
            timerEnabled = false;
            timer5MinEnabled = false;
            timer2MinEnabled = false;
            timerEndTime = 0;
            timer5MinEnd = 0;
            timer2MinEnd = 0;
            responseText = "Đã hủy hẹn giờ.";
            Serial.println("Đã hủy hẹn giờ");
            Serial1.print('o');
          } else {
            responseText = "Không có hẹn giờ để hủy.";
            Serial.println("Không có hẹn giờ để hủy");
          }
          xSemaphoreGive(displaySemaphore);
        } else {
          responseText = "Lỗi khi hủy hẹn giờ, vui lòng thử lại";
        }
      } else if (lowerTranscript.indexOf("kiem") != -1 || lowerTranscript.indexOf("tra") != -1) {
        Serial.println("Kiểm tra lịch");
        if (xSemaphoreTake(appointmentSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
          if (appointment.active && appointment.dateUnix > 0 && appointment.message[0] != '\0') {
            DateTime apptTime(appointment.dateUnix);
            char ttsText[128];
            snprintf(ttsText, sizeof(ttsText),
                     "Ngày %d tháng %d, lời nhắc: %s",
                     apptTime.day(), apptTime.month(), appointment.message);
            responseText = ttsText;
            Serial.println("Kiểm tra lịch: " + String(ttsText));
            Serial1.print('q');
          } else {
            responseText = "Không có lịch hẹn.";
            Serial.println("Không có lịch hẹn");
          }
          xSemaphoreGive(appointmentSemaphore);
        } else {
          responseText = "Lỗi khi kiểm tra lịch, vui lòng thử lại";
        }
      } else {
        responseText = "Không hiểu lệnh, vui lòng thử lại";
      }
    }
  } else {
    Serial.println("Lỗi HTTP: " + http.errorToString(httpResponseCode));
    // Serial1.print('c');
    responseText = "Lỗi, vui lòng thử lại";
  }
  http.end();
  // Xử lý ghi âm lời hẹn nếu cần
  if (isAppointmentRecording && WiFi.status() == WL_CONNECTED) {
    playTTS(responseText.c_str(), "vi");
    vTaskDelay(pdMS_TO_TICKS(450));  // Chờ TTS hoàn tất
    Serial.println("Bắt đầu ghi âm lời hẹn...");
    uint8_t* apptPcmData = NULL;
    size_t apptPcmSize = 0;
    if (recordAudio_new_driver(&apptPcmData, &apptPcmSize)) {
      if (apptPcmSize > 0 && apptPcmData != NULL) {
        size_t apptWavSize = 44 + apptPcmSize;
        uint8_t* apptWavData = (uint8_t*)heap_caps_malloc(apptWavSize, MALLOC_CAP_8BIT);
        if (apptWavData) {
          uint8_t wavHeader[44];
          createWavHeader(wavHeader, apptPcmSize);
          memcpy(apptWavData, wavHeader, 44);
          memcpy(apptWavData + 44, apptPcmData, apptPcmSize);
          heap_caps_free(apptPcmData);

          String part1 = String("--") + boundary + "\r\n" + "Content-Disposition: form-data; name=\"file\"; filename=\"appointment.wav\"\r\n" + "Content-Type: audio/wav\r\n\r\n";
          String part2 = "\r\n--" + String(boundary) + "--\r\n";
          size_t part1Len = part1.length();
          size_t part2Len = part2.length();
          size_t payloadSize = part1Len + apptWavSize + part2Len;

          uint8_t* payload = (uint8_t*)heap_caps_malloc(payloadSize, MALLOC_CAP_8BIT);
          if (payload) {
            memcpy(payload, part1.c_str(), part1Len);
            memcpy(payload + part1Len, apptWavData, apptWavSize);
            memcpy(payload + part1Len + apptWavSize, part2.c_str(), part2Len);
            heap_caps_free(apptWavData);

            HTTPClient http;
            http.setTimeout(15000);
            http.begin(viettel_api);
            http.addHeader("Content-Type", "multipart/form-data; boundary=" + String(boundary));
            http.addHeader("Authorization", "Bearer " + String(viettel_token));

            int httpResponseCode = http.POST(payload, payloadSize);
            heap_caps_free(payload);

            if (httpResponseCode > 0) {
              String response = http.getString();
              String apptTranscript = parseTranscript(response);
              if (apptTranscript != "") {
                // Loại bỏ dấu chấm và khoảng trắng thừa trong lời hẹn
                apptTranscript.trim();
                apptTranscript.replace(".", "");
                if (xSemaphoreTake(appointmentSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
                  strncpy(appointment.message, apptTranscript.c_str(), sizeof(appointment.message) - 1);
                  appointment.message[sizeof(appointment.message) - 1] = '\0';
                  DateTime apptTime(appointment.dateUnix);
                  char ttsText[128];
                  snprintf(ttsText, sizeof(ttsText),
                           "Đã đặt lịch ngày %d tháng %d, lời nhắc: %s",
                           apptTime.day(), apptTime.month(), appointment.message);
                  responseText = ttsText;
                  Serial.println("Lưu lời hẹn: " + String(appointment.message));
                  Serial.println("TTS: " + String(ttsText));
                  xSemaphoreGive(appointmentSemaphore);
                } else {
                  responseText = "Lỗi lưu lời hẹn, vui lòng thử lại";
                }
              } else {
                responseText = "Không nhận diện được lời hẹn, vui lòng thử lại";
              }
            } else {
              responseText = "Lỗi gửi lời hẹn, vui lòng thử lại";
            }
            http.end();
          } else {
            heap_caps_free(apptWavData);
            responseText = "Lỗi bộ nhớ khi gửi lời hẹn";
          }
        } else {
          heap_caps_free(apptPcmData);
          responseText = "Lỗi bộ nhớ khi xử lý lời hẹn";
        }
      } else {
        heap_caps_free(apptPcmData);
        responseText = "Lỗi: Dữ liệu âm thanh lời hẹn không hợp lệ";
      }
    } else {
      responseText = "Lỗi ghi âm lời hẹn, vui lòng thử lại";
    }
  }

  if (WiFi.status() == WL_CONNECTED && responseText != "") {
    Serial.println("Chuẩn bị phát TTS: " + responseText);
    playTTS(responseText.c_str(), "vi");
  } else if (responseText != "") {
    Serial.println("Không phát TTS: WiFi ngắt hoặc responseText rỗng");
  }
}

void wifiTask(void* pvParameters) {
  connectWiFi();
  fetchWeather();
  fetchForecast();
  syncTimeFromAPI();

  unsigned long lastWeatherUpdate = 0;
  unsigned long lastForecastUpdate = 0;
  unsigned long lastTimeSync = 0;
  const unsigned long weatherInterval = 600000;
  const unsigned long forecastInterval = 3600000;
  const unsigned long timeSyncInterval = 604800000;

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
            fetchForecast();
            xSemaphoreGive(displaySemaphore);
          }
          syncTimeFromAPI();
        } else {
          connectWiFi();
        }
      }
    }

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
void displayTask(void* pvParameters) {
  static DateTime lastTime = DateTime((uint32_t)0);
  bool alarmTriggered = false;
  bool appointmentTriggered = false;
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
    char apptStr[12] = "";
    if (xSemaphoreTake(appointmentSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (appointment.active && appointment.dateUnix > 0) {
        DateTime apptTime(appointment.dateUnix);
        snprintf(apptStr, sizeof(apptStr), "Day : %02d-%02d", apptTime.day(), apptTime.month());
      }
      xSemaphoreGive(appointmentSemaphore);
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
    // Kiểm tra và phát lời hẹn vào 6:00 sáng
    if (xSemaphoreTake(appointmentSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (appointment.active && appointment.dateUnix > 0) {
        DateTime apptTime(appointment.dateUnix);
        if (now.day() == apptTime.day() && now.month() == apptTime.month() && now.hour() == 6 && now.minute() == 0 && now.second() == 0 && !appointmentTriggered) {
          appointmentTriggered = true;
          if (appointment.message[0] != '\0' && WiFi.status() == WL_CONNECTED) {
            Serial.println("Phát lại lời hẹn: " + String(appointment.message));
            playTTS(appointment.message, "vi");
            // Xóa lịch sau khi phát
            appointment.dateUnix = 0;
            appointment.message[0] = '\0';
            appointment.active = false;
            if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
              currentCommand = "Lịch hẹn đã kích hoạt";
              xSemaphoreGive(displaySemaphore);
            }
          }
        }
        // Reset trigger sau 6:01 cùng ngày
        if (now.hour() >= 6 && now.minute() >= 1 && now.day() == apptTime.day() && now.month() == apptTime.month()) {
          appointmentTriggered = false;
        }
      }
      xSemaphoreGive(appointmentSemaphore);
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

      if (apptStr[0] != '\0') {
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(5, 44, apptStr);
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
        switch (receivedChar) {
          case 'a':
            digitalWrite(BUZZER_PIN, HIGH);
            delay(200);
            digitalWrite(BUZZER_PIN, LOW);
            xTaskNotifyGive(wifiTaskHandle);
            Serial.println("Online: Nhận 'a', buzzer kêu 200ms, kích hoạt ghi âm");
            break;
          default:
            break;
        }
      } else {
        switch (receivedChar) {
          case 'a':
            digitalWrite(BUZZER_PIN, LOW);
            xTaskNotifyGive(wifiTaskHandle);
            Serial.println("Offline: Nhận 'a', buzzer tắt");
            break;
          case 'g':
            if (xSemaphoreTake(displaySemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
              DateTime now = rtc.now();
              displayMode2 = true;
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
              displayMode2 = true;
              digitalWrite(BUZZER_PIN, HIGH);
              delay(200);
              digitalWrite(BUZZER_PIN, LOW);
              // Reset LED 7 đoạn
              ShowDigits7SegmentLED(0, 5, 0, 0);
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
              displayMode2 = true;
              digitalWrite(BUZZER_PIN, HIGH);
              delay(200);
              digitalWrite(BUZZER_PIN, LOW);
              // Reset LED 7 đoạn
              ShowDigits7SegmentLED(0, 2, 0, 0);
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
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Free heap tại setup: %d bytes\n", ESP.getFreeHeap());
  Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(12, 13);
  u8g2.begin();
  if (!rtc.begin()) {
    Serial.println("RTC failed to begin!");
    while (1)
      ;
  }
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
    audio->setVolume(100);
    Serial.println("Đối tượng Audio cho loa đã được khởi tạo.");
  } else {
    Serial.println("Lỗi khởi tạo đối tượng Audio cho loa!");
  }
  setupI2S_new_driver();
  if (!micI2SInitialized) {
    Serial.println("LỖI NGHIÊM TRỌNG: Không thể khởi tạo I2S cho micro trong setup!");
  }
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
  for (int i = 0; i < 3; i++) {
    forecastData[i].temp = 0.0;
    forecastData[i].desc = "N/A";
    forecastData[i].timestamp = 0;
    forecastData[i].pop = 0.0;
  }
  appointmentSemaphore = xSemaphoreCreateMutex();
  if (appointmentSemaphore == NULL) {
    Serial.println("Lỗi tạo appointmentSemaphore!");
    while (1)
      ;
  }
  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 16384, NULL, 2, &wifiTaskHandle, 0);
  xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, &displayTaskHandle, 1);
  xTaskCreatePinnedToCore(uartTask, "UartTask", 2048, NULL, 1, &uartTaskHandle, 1);
  xTaskCreatePinnedToCore(checkTaskStack, "CheckStackTask", 2048, NULL, 1, NULL, 1);
}
void loop() {
  vTaskDelay(portMAX_DELAY);
}