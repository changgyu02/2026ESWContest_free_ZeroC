#include <Arduino.h>

// ==================== 엔코더 핀 설정 ====================
#define ENCODER_LEFT_A   22
#define ENCODER_LEFT_B   23
#define ENCODER_RIGHT_A  19
#define ENCODER_RIGHT_B  21

// 변수 오버플로우 방지 및 인터럽트 안전을 위해 volatile 선언
volatile long leftCount = 0;
volatile long rightCount = 0;

// ==================== 로봇 물리 파라미터 ====================
const float CPR = 13.0;
const float GEAR_RATIO = 144.0;
const float PPR = CPR * GEAR_RATIO;  // 1바퀴당 총 펄스 수 (약 1872)

// ROS2 mapping_launch.py 파라미터와 일치
const float WHEEL_RADIUS = 0.08;   // 바퀴 반지름 [m]
const float WHEEL_BASE   = 0.19;   // 바퀴 간 거리 (윤거) [m]

// ROS2 패킷 시퀀스 번호
uint16_t packet_seq = 0;

// ==================== 인터럽트 서비스 루틴 (ISR) ====================
void IRAM_ATTR handleLeftA() {
  if (digitalRead(ENCODER_LEFT_A) == HIGH) {
    if (digitalRead(ENCODER_LEFT_B) == LOW) leftCount++;
    else leftCount--;
  }
}

void IRAM_ATTR handleRightA() {
  if (digitalRead(ENCODER_RIGHT_A) == HIGH) {
    if (digitalRead(ENCODER_RIGHT_B) == LOW) rightCount--;
    else rightCount++;
  }
}

// ==================== Setup ====================
void setup() {
  // 시리얼 통신 초기화 (ROS 2와 통신할 USB 포트)
  Serial.begin(115200);           
  
  // ★ [수정 1] 시리얼 모니터가 열릴 때까지 무한 대기하는 독소 조항 삭제
  // while(!Serial); 

  // 엔코더 핀 설정 및 인터럽트 부착
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), handleLeftA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), handleRightA, RISING);
}

// ==================== Loop ====================
void loop() {
  // ---------------------------------------------------------
  // 엔코더 Odometry 계산 및 ROS 2 바이너리 송신 (50ms 마다 실행)
  // ---------------------------------------------------------
  static long lastLeft = 0;
  static long lastRight = 0;
  static unsigned long lastTime = millis();
  unsigned long now = millis();
  
  // 50ms 주기 (20Hz)
  if (now - lastTime >= 50) {
    uint16_t dt_ms = (uint16_t)(now - lastTime);
    float dt = dt_ms / 1000.0; 
    lastTime = now;

    // 1. 인터럽트 도중 값이 바뀌는 것을 방지하기 위해 순간 캡처
    noInterrupts();
    long currentLeftCount = leftCount;
    long currentRightCount = rightCount;
    interrupts();

    // 2. 주기 동안의 펄스 변화량(변위) 계산
    long dLeft = currentLeftCount - lastLeft;
    long dRight = currentRightCount - lastRight;
    lastLeft = currentLeftCount;
    lastRight = currentRightCount;

    // 3. 바퀴 이동 거리 계산 [m]
    float distPerCount = (2.0 * PI * WHEEL_RADIUS) / PPR;
    float dLeft_m  = dLeft * distPerCount;
    float dRight_m = dRight * distPerCount;

    // 4. 각 바퀴의 속도 계산 [m/s]
    float vLeft  = dLeft_m / dt;
    float vRight = dRight_m / dt;

    // 5. 로봇 중심 기준의 선속도(Linear) 및 각속도(Angular) 계산
    float linear_x  = (vRight + vLeft) / 2.0;
    float angular_z = (vRight - vLeft) / WHEEL_BASE;

    // 6. ROS 2 encoder_driver.cpp에 맞춘 17바이트 바이너리 패킷 생성
    uint8_t tx_buf[17];
    
    // 헤더 및 메타데이터
    tx_buf[0] = 0xAA; // FB_HDR0
    tx_buf[1] = 0x55; // FB_HDR1
    tx_buf[2] = 0x20; // FB_TYPE_VEL
    tx_buf[3] = 12;   // Payload Length (4+4+2+2)

    // float 변수를 byte 배열로 복사 (리틀 엔디안)
    memcpy(&tx_buf[4], &linear_x, 4);
    memcpy(&tx_buf[8], &angular_z, 4);
    memcpy(&tx_buf[12], &dt_ms, 2);
    memcpy(&tx_buf[14], &packet_seq, 2);

    // 체크섬 계산 (index 2 부터 15까지의 합)
    uint8_t sum = 0;
    for (int i = 2; i < 16; i++) {
        sum += tx_buf[i];
    }
    tx_buf[16] = sum; // 마지막 바이트는 체크섬

    Serial.write(tx_buf, 17);
    
    packet_seq++;
  }
}
