#define BLYNK_TEMPLATE_ID "TMPL6pIN-O2Db"
#define BLYNK_TEMPLATE_NAME "da"
#define BLYNK_AUTH_TOKEN "WJqkbjQbWq4b7Jbf23GPZEd5YTnqNDpr"
#include <Arduino.h>
#include<DHT20.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <WiFi.h>
////////////////////////////////////////////
#define FAN 2
#define TRIGGER_PIN 19  // Chân Trigger của cảm biến ORC
#define ECHO_PIN 23    // Chân Echo của cảm biến ORC
#define NUM_LEDS 4
#define CD_PIN 27         //chuyển động
#define Light_PIN  32       ///ánh sáng
#define LED_PIN 33      //led
////////// #define Servo 13
Servo myServo;  // Khai báo đối tượng Servo
const int servoPin = 18;  // **Chân kết nối Servo (GPIO 13)**
int button =0;
int varFan=0;
unsigned long servoStartTime = 0;
unsigned long ultrasonicStartTime = 0;
unsigned long totalServoTime = 0;
unsigned long totalUltrasonicTime = 0;

unsigned long fanStartTime = 0;
unsigned long totalFanTime = 0;

unsigned long ledStartTime = 0;
unsigned long totalLedTime = 0;

unsigned long dhtStartTime = 0;
unsigned long totalDhtTime = 0;

unsigned long camBienCDStartTime = 0;
unsigned long totalCamBienCDTime = 0;
char auth[] = BLYNK_AUTH_TOKEN;
char  ssid[] = "ACLAB";
char pass[]="ACLAB2023";
DHT20 dht20;  // Khởi tạo đối tượng cảm biến DHT20
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
long duration;
int distance;
//set 3 led color
// Hàm để theo dõi và cộng dồn thời gian hoạt động cho từng thiết bị
void trackTime(unsigned long &startTime, unsigned long &totalTime) {
  if (startTime == 0) {
    startTime = millis();  // Ghi lại thời gian bắt đầu khi thiết bị hoạt động
  }

  totalTime += millis() - startTime;  // Cộng dồn thời gian hoạt động
  startTime = millis();  // Cập nhật thời gian bắt đầu khi thiết bị hoạt động tiếp
}

// Hàm mở cửa điều khiển Servo
void openDoor() {
  myServo.write(120);  // Mở cửa (Servo xoay 90 độ)
  delay(1000); // Giữ cửa mở trong 5 giây
  myServo.write(0);   // Đóng cửa (Servo xoay về 0 độ)
}

// Hàm kiểm tra cảm biến siêu âm và điều khiển Servo
void checkUltrasonic() {
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  if (distance < 15) {
    Serial.println("🚶 Phát hiện vật thể! Mở cửa.");
    digitalWrite(LED_PIN, HIGH); // Bật LED
    openDoor();  // Gọi hàm mở cửa
    trackTime(servoStartTime, totalServoTime);  // Cộng dồn thời gian hoạt động của Servo
  }
}

// Hàm điều khiển quạt
void controlFan(bool isOn) {
  if (isOn) {
    digitalWrite(FAN, HIGH);  // Bật quạt
    trackTime(fanStartTime, totalFanTime);  // Cộng dồn thời gian quạt hoạt động
  } else {
    digitalWrite(FAN, LOW);  // Tắt quạt
  }
}

// Hàm điều khiển LED
void controlLED(bool isOn) {
  if (isOn) {
    digitalWrite(LED_PIN, HIGH);  // Bật LED
    trackTime(ledStartTime, totalLedTime);  // Cộng dồn thời gian LED hoạt động
  } else {
    digitalWrite(LED_PIN, LOW);  // Tắt LED
  }
}

// Hàm điều khiển cảm biến DHT20 (Giả sử DHT20 được kết nối với chân DHT_PIN)
void controlDHT20() {
 
  trackTime(dhtStartTime, totalDhtTime);  // Cộng dồn thời gian hoạt động của DHT20
}

// Hàm điều khiển cảm biến cửa
void controlCamBienCD() {
  // Giả sử cảm biến cửa được bật khi có tín hiệu mở cửa
  trackTime(camBienCDStartTime, totalCamBienCDTime);  // Cộng dồn thời gian cảm biến cửa hoạt động
}

void setColor(uint32_t color) {   
  for (int i = 0; i < NUM_LEDS-1; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}
void setup() {  
  //set up blynk
 
  // put your setup code here, to run once:
  Serial.begin(115200);  // Khởi động Serial Monitor
  Wire.begin();          // Khởi động giao tiếp I2C
  Blynk.begin(auth, ssid, pass);
  if (dht20.begin()) {
    Serial.println("DHT20 đã khởi động thành công!");
  } else {
    Serial.println("Không tìm thấy DHT20, kiểm tra kết nối!");
  }
  pinMode(CD_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);  // Chân Trigger là Output
  pinMode(ECHO_PIN, INPUT);     // Chân Echo là Input
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN, OUTPUT);
  myServo.attach(servoPin, 500, 2400);
  strip.begin();
  strip.show(); // Tắt tất cả LED ban đầu
}
void http_send(){

}
void loop() {

  // put your main code here, to run repeatedly:
  Blynk.run();
  int light_value = analogRead(Light_PIN);
  float light_percent = (light_value / 4095.0) * 100.0;  // ESP32 có độ phân giải ADC 12-bit (0-4095)
  int motion = digitalRead(CD_PIN);

  if (motion == HIGH) { // Nếu phát hiện chuyển động
    Serial.println("🚶 Phát hiện chuyển động! Bật LED // mo cua.");
    setColor(strip.Color(0, 255, 0));//xanh
   // myServo.write(120);  // Mở cửa (Servo xoay 90 độ)
     // Giữ LED sáng 5 giây
    //myServo.write(0);   // Đóng cửa (Servo xoay về 0 độ)
    openDoor();
  } else {
    setColor(strip.Color(0, 0, 0)); // Tắt LED
  }
  // Đọc nhiệt độ và độ ẩm từ DHT20
  if (dht20.read()) {
    Serial.println("❌ Lỗi khi đọc cảm biến DHT20!");
  } else {
    Serial.print("🌡 Nhiệt độ: ");
    Serial.print(dht20.getTemperature());
    Serial.println("°C");
    Serial.print("💡 Ánh sáng : ");
    Serial.print(light_percent);
    Serial.println("%");

    Serial.print("💧 Độ ẩm: ");
    Serial.print(dht20.getHumidity());
    Serial.println("%");
    Serial.println("--------------------------------------------------");
  }
  Blynk.virtualWrite(V0, dht20.getTemperature());
  Blynk.virtualWrite(V4, dht20.getHumidity());
  Blynk.virtualWrite(V1, light_percent);
   // Gửi tín hiệu Trigger để đo khoảng cách
   digitalWrite(TRIGGER_PIN, LOW);  
   delayMicroseconds(2);
   digitalWrite(TRIGGER_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(TRIGGER_PIN, LOW);
 
   // Đo thời gian phản hồi từ Echo
   duration = pulseIn(ECHO_PIN, HIGH);
   
   // Tính khoảng cách (cm) dựa trên thời gian phản hồi
   distance = duration * 0.034 / 2;
   if (distance < 20) {
    Serial.println("🚶 Phát hiện vật thể! Mở cửa.");    
    myServo.write(120);  // Mở cửa (Servo xoay 90 độ)
    delay(1000); // Giữ cửa mở trong 5 giây
    myServo.write(0);   // Đóng cửa (Servo xoay về 0 độ)
  }
  delay(1000); // Đợi 1 giây
}
 


// put function definitions here:
BLYNK_WRITE(V5) {
  int varFan = param.asInt();  // Đọc giá trị từ Blynk (Slider)
  int fanSpeed = map(varFan, 0, 100, 0, 255); // Chuyển đổi giá trị từ 0-100% sang PWM (0-255)

  if (varFan == 0) {
    analogWrite(FAN, 0);  // Tắt quạt hoàn toàn
  } else {
    analogWrite(FAN, fanSpeed);  // Điều chỉnh tốc độ quạt theo PWM
  }
}
BLYNK_WRITE(V3)
{
  button = param.asInt();
  if (button == 1)
  {
    strip.setPixelColor(3, strip.Color(255, 0, 0)); // Cũng là đỏ, nếu LED dùng GRB

  }
  else
  {
    strip.setPixelColor(3, strip.Color(0, 0, 0)); // Cũng là đỏ, nếu LED dùng GRB

  }
}
