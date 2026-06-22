#ifndef LORA_TRANSMITTER_H
#define LORA_TRANSMITTER_H

#include <SoftwareSerial.h>

// 회로 배선도(D4, D1) 기준 아두이노 Nano 핀 맵 구성
// RYLR898 모듈의 TX -> 아두이노 D4 (RX)
// RYLR898 모듈의 RX -> 아두이노 D1 (TX)
#define LORA_RX 4
#define LORA_TX 1

SoftwareSerial loraSerial(LORA_RX, LORA_TX);

// LoRa 모듈 초기화 (AT 커맨드 세팅)
void lora_init() {
    loraSerial.begin(115200); // RYLR898 기본 보레이트
    delay(500);

    // 모듈 정상 연결 확인용 AT 테스트
    loraSerial.println("AT");
    delay(100);
    
    // 무선 주소 설정 (예: ADDRESS=1)
    loraSerial.println("AT+ADDRESS=1");
    delay(100);
    
    // 네트워크 ID 설정 (송수신기 매칭용 기본값 18)
    loraSerial.println("AT+NETWORKID=18");
    delay(100);
}

// 데이터를 패킷 문자열로 변환하여 수신측(메인 컴퓨터)으로 송신하는 함수
void lora_send(double lat, double lon, double temp, const char* msg_type) {
    char payload[64];
    
    // 수신측에서 파싱하기 쉽도록 콤마(,) 구문 분리 포맷팅
    // 형식: [메시지타입,위도,경도,온도]
    // 예: [FIRE_ALERT,37.450123,126.653456,115.50]
    sprintf(payload, "%s,%.6f,%.6f,%.2f", msg_type, lat, lon, temp);
    
    int packet_length = strlen(payload);
    
    // RYLR898 송신 커맨드: AT+SEND=<수신측ADDRESS>,<데이터길이>,<데이터>
    // 게이트웨이/메인 컴퓨터 주소를 0번으로 가정
    loraSerial.print("AT+SEND=0,");
    loraSerial.print(packet_length);
    loraSerial.print(",");
    loraSerial.println(payload);
    
    // 전송 안정성을 위한 딜레이
    delay(500); 
}

#endif // LORA_TRANSMITTER_H
