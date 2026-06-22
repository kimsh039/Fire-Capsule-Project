#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

// 핀 맵 구성
#define PUSH_BUTTON 3       // 초기 세팅용 푸쉬 버튼
#define LATCHING_RELAY 5    // 전원 차단용 래칭 릴레이
#define THERMAL_SWITCH 2    // KSD-01F 열 스위치 입력

// EEPROM 주소 할당
#define ADDR_LAT 0
#define ADDR_LON 10
#define ADDR_FLAG 20

double init_lat = 0.0;
double init_lon = 0.0;
bool is_configured = false;

// 사용자 정의 헤더 포함
#include "kalman_filter.h"
#include "lora_transmitter.h"

void setup() {
  Serial.begin(9600);
  
  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  pinMode(LATCHING_RELAY, OUTPUT);
  pinMode(THERMAL_SWITCH, INPUT_PULLUP);
  
  digitalWrite(LATCHING_RELAY, LOW); 

  // 기기 부팅 시 세팅 플래그 확인
  if (EEPROM.read(ADDR_FLAG) == 1) {
    is_configured = true;
    EEPROM.get(ADDR_LAT, init_lat);
    EEPROM.get(ADDR_LON, init_lon);
  }

  // [화재 발생 시나리오] 기준 위치가 잡힌 상태에서 열스위치 접점 붙어서 켜진 경우
  if (is_configured && digitalRead(THERMAL_SWITCH) == LOW) {
    process_fire_event();
  }
}

void loop() {
  // [초기 세팅 시나리오] 기기 배치 전 버튼 눌러서 현재 위치 수렴 및 EEPROM 저장
  if (digitalRead(PUSH_BUTTON) == LOW && !is_configured) {
    delay(50); // 디바운스
    if (digitalRead(PUSH_BUTTON) == LOW) {
      save_anchor_position();
    }
  }
}

// 초기 배치 위치 잡고 셧다운하는 함수
void save_anchor_position() {
  double c_lat = 0.0, c_lon = 0.0;
  double p_lat = 0.0, p_lon = 0.0;
  const double EPSILON = 0.0001; // 수렴 판단 임계치
  
  gps_init(); 
  
  while (true) {
    get_gps_raw(c_lat, c_lon);
    
    // GPS 오차가 범위 이내로 들어오면 수렴한 것으로 판단
    if (p_lat != 0.0 && 
        abs(c_lat - p_lat) < EPSILON && 
        abs(c_lon - p_lon) < EPSILON) {
      
      init_lat = c_lat;
      init_lon = c_lon;
      
      EEPROM.put(ADDR_LAT, init_lat);
      EEPROM.put(ADDR_LON, init_lon);
      EEPROM.write(ADDR_FLAG, 1); 
      break;
    }
    
    p_lat = c_lat;
    p_lon = c_lon;
    delay(2000); // 2초 주기로 체크
  }
  
  // 래칭 릴레이 트리거해서 전원 완전 차단 (슬립모드 진입)
  delay(500); 
  digitalWrite(LATCHING_RELAY, HIGH); 
}

// 화재 감지 시 탈출 및 송신 루틴
void process_fire_event() {
  lora_init();
  max31855_init();
  gps_init();
  
  // 1. EEPROM 백업 위치로 즉시 불난 위치 1차 서빙
  lora_send(init_lat, init_lon, 0.0, "REF_POS");

  // 칼만필터 초기값 세팅 (EEPROM 값 서빙해서 수렴 속도 단축)
  kalman_init(init_lat, init_lon);

  // 2. 이후 칼만필터로 보정하면서 실시간 데이터 전송
  for (int i = 0; i < 10; i++) { 
    double raw_lat, raw_lon;
    double filtered_lat, filtered_lon;
    
    get_gps_raw(raw_lat, raw_lon);
    double current_temp = get_temp();
    
    kalman_update(raw_lat, raw_lon, filtered_lat, filtered_lon);
    lora_send(filtered_lat, filtered_lon, current_temp, "FIRE_ALERT");
    
    delay(3000); 
  }
}

// 하드웨어 종속 가상 함수 인터페이스 (컴파일 에러 방지용)
void gps_init() {}
void lora_init() {}
void max31855_init() {}
void get_gps_raw(double &lat, double &lon) { lat = 37.450; lon = 126.653; }
double get_temp() { return 115.5; }
