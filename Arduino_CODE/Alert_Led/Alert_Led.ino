#include <Adafruit_NeoPixel.h>

#define LED_PIN_1     16   // 첫 번째 링 데이터 핀
#define LED_PIN_2     17   // 두 번째 링 데이터 핀
#define LED_COUNT     16   // 링당 LED 개수 (링 2개 직렬: 8 x 2)
#define BAUD_RATE 115200
#define BLINK_INTERVAL 1000  // ms, 1초 점멸 주기

Adafruit_NeoPixel ring1(LED_COUNT, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ring2(LED_COUNT, LED_PIN_2, NEO_GRB + NEO_KHZ800);

bool          warning_active = false;
bool          blink_state    = false;
unsigned long last_blink_ms  = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  ring1.begin();
  ring1.setBrightness(150);
  ring1.clear();
  ring1.show();

  ring2.begin();
  ring2.setBrightness(150);
  ring2.clear();
  ring2.show();
}

void fillColor(uint32_t color) {
  for (int i = 0; i < LED_COUNT; i++) {
    ring1.setPixelColor(i, color);
    ring2.setPixelColor(i, color);
  }
  ring1.show();
  ring2.show();
}

void clearAll() {
  ring1.clear();
  ring2.clear();
  ring1.show();
  ring2.show();
}

void process_serial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "ON") {
    warning_active = true;
    blink_state    = true;
    last_blink_ms  = millis();
    fillColor(ring1.Color(255, 0, 0));  // 빨강

  } else if (cmd == "OFF") {
    warning_active = false;
    blink_state    = false;
    clearAll();
  }
}

void update_blink() {
  if (!warning_active) return;

  unsigned long now = millis();
  if (now - last_blink_ms >= (unsigned long)BLINK_INTERVAL) {
    blink_state   = !blink_state;
    last_blink_ms = now;

    if (blink_state) {
      fillColor(ring1.Color(255, 0, 0));  // 빨강 ON
    } else {
      clearAll();                          // OFF
    }
  }
}

void loop() {
  process_serial();
  update_blink();
}
