#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// 칼만 필터 상태 변수 정의 (위도, 경도 각각 독립적으로 운영)
struct KalmanVars {
    double x; // 최적 추정 상태값 (Position)
    double P; // 추정 오차 공분산 (Error Covariance)
    double Q; // 프로세스 노이즈 공분산 (Process Noise)
    double R; // 측정 노이즈 공분산 (Measurement Noise)
    double K; // 칼만 이득 (Kalman Gain)
};

KalmanVars k_lat;
KalmanVars k_lon;

// 칼만 필터 초기화 함수 (EEPROM에서 불러온 초기 기준 위치를 적용)
void kalman_init(double init_lat, double init_lon) {
    // 위도 필터 초기화
    k_lat.x = init_lat;
    k_lat.P = 1.0;       // 초기 오차 공분산 (초기값 설정에 따라 수렴 속도 결정)
    k_lat.Q = 0.00001;   // 프로세스 노이즈 (시스템 자체의 시스템 모델 오차)
    k_lat.R = 0.001;     // 측정 노이즈 (GPS 모듈 자체의 센서 노이즈)

    // 경도 필터 초기화
    k_lon.x = init_lon;
    k_lon.P = 1.0;
    k_lon.Q = 0.00001;
    k_lon.R = 0.001;
}

// 단일 축 칼만 필터 연산 (보고서 수식 반영)
double kalman_filter_step(KalmanVars &k, double measurement) {
    // 1. 예측 (Prediction Phase)
    // x_minus = A * x + B * u (여기서 A=1, B=0 가정 기본 모델)
    double x_minus = k.x; 
    // P_minus = A * P * A^T + Q
    double P_minus = k.P + k.Q;

    // 2. 갱신 (Update Phase)
    // K = P_minus * H^T * (H * P_minus * H^T + R)^-1 (여기서 H=1 가정)
    k.K = P_minus / (P_minus + k.R);
    
    // x = x_minus + K * (z - H * x_minus)
    k.x = x_minus + k.K * (measurement - x_minus);
    
    // P = (I - K * H) * P_minus
    k.P = (1.0 - k.K) * P_minus;

    return k.x;
}

// 메인 루프에서 호출할 위도/경도 업데이트 인터페이스
void kalman_update(double raw_lat, double raw_lon, double &filtered_lat, double &filtered_lon) {
    filtered_lat = kalman_filter_step(k_lat, raw_lat);
    filtered_lon = kalman_filter_step(k_lon, raw_lon);
}

#endif // KALMAN_FILTER_H
