# Smart_Planter
- 6가지 센서를 이용한 자동 식물 관리 장치
- atmega 128 사용(atmel studio 7.0)
- Uart 통신
- Low Pass Filter(LPF/1차) 사용

1. 가변저항
- 화분 무게에 따라 필요한 모터의 속도가 다르기 때문에, 가변저항 값에 따라 하단의 컨베이어 벨트를 움직이는 모터의 속도를 변경

2. 조도센서(GL5537) CDS
- 빛의량을 측정해 어두워지면 천장의 LED를 작동
- LUX

3. 온도센서(LM35)
- 내부 온도를 측정하여 현재 온도를 보드의 LED를 통해 시각화
- C'

4. 거리센서(C29) PSD
- 화분과 PSD센서 사이의 거리를 측정하여 화분에 크기에 맞게 화분이 수분 공급원 위치에 위치할 수 있도록 하단의 컨베이어 벨트를 작동(양방향 이동)
- cm

5. 토양 습도 센서
- 토양의 수분 함량을 측정하여 수분 함량이 일정 수치 이하일 때, 수분 공급 장치를 작동시켜 식물에게 필요한 물을 자동으로 공급
- Per(%)

6. 가스센서(MQ2)
- ‘Smart Planter’ 안에 담배 연기 등 내부 유해가스르 측정하여 가스량이 일정 수치 이상일 때, 팬을 작동시켜 환기
- ppm
