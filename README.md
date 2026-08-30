# 2026_Capstone_Design_Zero 사이다

# 🏷️ Intro
**ZERO 사이다** 팀이 개발한 시스템은 **AidGO(자율주행 Aid Kit 이송 로봇)**로, 실내 공간에서 발생할 수 있는 낙상, 심정지 등 응급상황 발생 시 **CCTV를 통한 실시간 이벤트 감지 → Main Server 명령 전송 → AidGO 자율주행 → 응급 Kit(AED 및 구급상자) 전달 및 안내 → 상황 종료 후 원점 복귀** 과정을 자동화하는 지능형 응급 지원 시스템입니다[cite: 1, 2].

# 💡 Inspiration
심정지 환자의 골든타임은 약 4분으로, 사고 발생 직후 신속한 심폐소생술과 자동심장충격기(AED) 사용이 생존율을 2~3배 이상 높이는 핵심입니다[cite: 1, 2]. 그러나 기존 고정형 AED 비상용품함은 당황한 사용자가 정확한 위치를 찾지 못하거나 직접 가져오는 과정에서 상당한 시간이 소요되어 골든타임을 놓치는 한계가 있었습니다[cite: 1, 2].

저희는 이러한 문제의 원인을 "사람이 비상용품을 찾아가는 수동적 응급 대응 방식"에서 찾았습니다[cite: 1, 2]. 만약 CCTV가 AI 기반으로 쓰러짐/낙상 등 응급상황을 스스로 감지하고, 자율주행 로봇이 사고 위치까지 직접 AED 및 응급 Kit를 전달한다면 초기 대응 시간을 획기적으로 줄일 수 있을 것이라 기대했습니다[cite: 1, 2]. 나아가 실시간 장비 및 소모품 관리를 결합해 공공장소의 안전 대응 체계에 새로운 기준을 제시하고자 합니다[cite: 1, 2, 3].

# 📸 Overview
*(시스템 전체 프로세스 도식화 이미지)*

1. **CCTV 실시간 감지**: YOLO11 Pose 기반 객체 인식 및 행동 분석으로 낙상/응급상황 감지[cite: 3]
2. **서버 분석 및 명령 전송**: Homography 변환으로 실제 공간 좌표를 추정하여 로봇에 출동 명령 및 목적지 전송[cite: 3]
3. **AidGO 자율주행 이동**: 2D LiDAR, AMCL 위치 추정, A* 경로 계획, Pure Pursuit 경로 추종 알고리즘 기반 자율주행 시작[cite: 1]
4. **목적지 도착 및 전달**: 디스플레이/스피커를 통한 AED 사용 방법 안내 및 응급 Kit 전달[cite: 1, 3]
5. **실시간 모니터링 및 알림**: 관리자 App/Web을 통해 로봇 위치, 배터리, 소모품 현황 실시간 공유[cite: 1, 3]
6. **원점 복귀 (Go Home)**: '상황 종료' 버튼 클릭 시 로봇이 원점으로 자율주행 복귀[cite: 1]


# 🚀 Main feature(HW & SW)
## 🤖 HW
*(HW 3D 모델링 및 내부 회로 구성 이미지)*

- **내부 다층 구조 프레임**: 질량이 큰 배터리와 모터를 하단 1층에 배치하여 무게중심을 낮추고 주행 안정성 확보[cite: 1]. 2층 제어부 배치로 유지보수성 향상[cite: 1].
- **응급 차 콘셉트 외관**: 흰색/빨간색 조합 외관, 적·청색 LED 경광등(WS2812 LED Ring) 및 부저/스피커를 장착하여 보행자 양보 유도 및 위치 식별성 강화[cite: 1].
- **안내 디스플레이**: 7인치 터치스크린을 통해 AED 사용법 동영상 및 로봇 상태/탈거 안내 표시[cite: 1, 3].
- **전장 및 구동부**: Jetson Orin Nano(메인), ESP32(모터/엔코더/센서 제어), 듀얼 모터 드라이버(MDDS10), INA260(전력 측정), BNO055(IMU 센서), RPLIDAR 2D[cite: 1].

## 🖥️ SW (주요 기능 및 프로세스)
*(SW 동작 및 앱 UI / CCTV 감지 이미지)*

- **AI 기반 CCTV 응급 감지**: YOLO11 Pose로 관절 좌표 추출 후, ByteTrack ID 추적 및 어깨-골반 기울기·Bounding Box 비율 분석을 통한 낙상 자동 감지[cite: 3].
- **Homography 좌표 변환**: CCTV 영상 내 4개 기준점을 실내 Grid Map 좌표와 대응시켜 환자의 실공간 위치 추정[cite: 3].
- **SLAM & Navigation2 자율주행**:
  - **Localization**: RPLIDAR 2D와 AMCL 알고리즘 기반 실시간 위치/방향 추정[cite: 1].
  - **Path Planning & Tracking**: A* 알고리즘으로 최적 경로 계획 후 Pure Pursuit 기반 안정적 경로 추종[cite: 1, 3].
- **관리자 App & Web Dashboard**: Flutter 기반 앱 및 React/TypeScript 기반 웹 화면을 통해 로봇 배터리, 실시간 위치, AED 탈거 여부, 구급 소모품 유효기간 통합 관리[cite: 1, 3].

## 🖥️ SW (장애 요인 및 해결 방안)
- **맵 스캔 노이즈 및 벽면 일그러짐**: Hector SLAM / slam_toolbox 재매핑 및 LiDAR 높이/각도 최적화, occupancy grid 후처리를 통해 정밀 Map 구축[cite: 1, 3].
- **자율주행 주행 진동 및 경로 지그재그**: Lookahead 거리(Ld) 튜닝, 가감속 속도 파라미터 및 Costmap 안전거리 최적화로 부드러운 곡선 주행 구현[cite: 1, 3].


# ⚙️ Environment

<table>
  <tr>
    <td><b>🖥️ Main Processor & Sub-Controller</b></td>
    <td>
      <img src="https://img.shields.io/badge/Jetson%20Orin%20Nano-76B900?style=for-the-badge&logo=nvidia&logoColor=white"/>
      <img src="https://img.shields.io/badge/ESP32-E7352C?style=for-the-badge&logo=espressif&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>🐧 OS & Middleware</b></td>
    <td>
      <img src="https://img.shields.io/badge/Ubuntu%2022.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white"/>
      <img src="https://img.shields.io/badge/ROS2%20Humble-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>💻 Programming Languages</b><br/>(Autonomous Driving, Control)</td>
    <td>
      <img src="https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white"/>
      <img src="https://img.shields.io/badge/Python%203.10-3776AB?style=for-the-badge&logo=python&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>🎨 Frontend</b></td>
    <td>
      <img src="https://img.shields.io/badge/Dart-0175C2?style=for-the-badge&logo=dart&logoColor=white"/>
      <img src="https://img.shields.io/badge/Flutter-02569B?style=for-the-badge&logo=flutter&logoColor=white"/>
      <img src="https://img.shields.io/badge/TypeScript-3178C6?style=for-the-badge&logo=typescript&logoColor=white"/>
      <img src="https://img.shields.io/badge/React-61DAFB?style=for-the-badge&logo=react&logoColor=black"/>
      <img src="https://img.shields.io/badge/Tailwind%20CSS-06B6D4?style=for-the-badge&logo=tailwindcss&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>🔧 Backend & Infra</b></td>
    <td>
      <img src="https://img.shields.io/badge/Django-092E20?style=for-the-badge&logo=django&logoColor=white"/>
      <img src="https://img.shields.io/badge/MySQL-4479A1?style=for-the-badge&logo=mysql&logoColor=white"/>
      <img src="https://img.shields.io/badge/Docker-2496ED?style=for-the-badge&logo=docker&logoColor=white"/>
      <img src="https://img.shields.io/badge/AWS%20EC2-FF9900?style=for-the-badge&logo=amazonaws&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>🤖 AI Model & Perception</b></td>
    <td>
      <img src="https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white"/>
      <img src="https://img.shields.io/badge/YOLO11-FF0000?style=for-the-badge&logo=yolo&logoColor=white"/>
      <img src="https://img.shields.io/badge/ByteTrack-000000?style=for-the-badge"/>
      <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>📡 Network & Protocol</b></td>
    <td>
      <img src="https://img.shields.io/badge/REST%20API-FF6F00?style=for-the-badge&logo=swagger&logoColor=white"/>
      <img src="https://img.shields.io/badge/WebSocket-2196F3?style=for-the-badge&logo=websocket&logoColor=white"/>
    </td>
  </tr>
  <tr>
    <td><b>📚 Libraries & Tools</b></td>
    <td>
      <img src="https://img.shields.io/badge/rclcpp-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
      <img src="https://img.shields.io/badge/rclpy-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
      <img src="https://img.shields.io/badge/RViz2-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
      <img src="https://img.shields.io/badge/Fritzing-138B88?style=for-the-badge"/>
      <img src="https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=github&logoColor=white"/>
    </td>
  </tr>
</table>

# 🎯Expected Effects
*(기대 효과 관련 이미지)*
AidGO 시스템은 실내 복합 공간 및 공공시설에서 응급상황 발생 시 비상용품 탐색 및 운반 지연 시간을 대폭 감소시켜 골든타임(4분 이내)을 확보합니다[cite: 1, 3]. 고정형 AED의 한계를 극복하고 로봇이 직접 사고 장소로 찾아감으로써, 일반인도 당황하지 않고 빠른 초기 응급처치를 시행할 수 있습니다[cite: 1, 2, 3]. 또한 실시간 모니터링을 통한 소모품 관리 효율화 및 공공장소의 복지/안전성을 극대화하여 스마트 응급 대응 체계의 새로운 패러다임을 제공합니다[cite: 1, 3].
