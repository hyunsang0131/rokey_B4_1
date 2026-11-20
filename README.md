# rokey_B4_1: 협동1 프로젝트

두산 **M0609** + **OnRobot RG2**로 실리콘 모듈 매트리스의 쿠션감(강성)을 측정하고, 무른 모듈을 찾아 교체하는 자동화 프로젝트의 ROS 2 코드입니다.

이 저장소에는 지금 **로봇을 노트북에서 다루기 위한 도구**와 **측정 준비용 테스트 코드**가 들어 있습니다.

| 패키지 | 역할 | 실행 파일 |
|---|---|---|
| [`robot_setting`](robot_setting/README.md) | 티칭 펜던트 기능을 노트북에서 실행 (상태, 서보, 홈, 직접 교시, 툴/TCP) | `status`, `servo_on`, `servo_off`, `home`, `direct_teach`, `tool_tcp` |
| [`pjt_test1`](pjt_test1/README.md) | 측정 기능 테스트 (센서 읽기, 교시 위치 이동, 위치별 힘 측정) | `read_sensor`, `move_to_teach`, `measure_teach_force` |

## 환경

- Ubuntu 24.04, ROS 2 Jazzy
- Doosan ROS 2 드라이버 [`doosan-robot2`](https://github.com/DoosanRobotics/doosan-robot2) (`dsr_msgs2`, `dsr_common2`)
- RG2 브링업 패키지 `m0609_rg2_bringup` (강의 제공)
- `dsr_tactile_probing_sim.py`의 2D 상단 이미지 생성에는 Matplotlib이 필요합니다: `sudo apt install python3-matplotlib`

## 설치

워크스페이스 `src` 안에 클론하면 colcon이 두 패키지를 찾습니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr/src
git clone https://github.com/hyunsang0131/rokey_B4_1.git
cd ..
colcon build --packages-select robot_setting pjt_test1
source install/setup.bash
```

## 실행 준비

1. **브링업**: 실물 로봇

   ```bash
   ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=<컨트롤러 IP> port:=12345 model:=m0609
   ```

   에뮬레이터로 먼저 시험하려면 `mode:=virtual`을 사용합니다.

2. **ROS_DOMAIN_ID 맞추기**: 브링업과 같은 값이어야 서비스에 연결됩니다. 연결되지 않으면 스크립트가 `서비스 연결 실패`를 출력합니다.

   ```bash
   export ROS_DOMAIN_ID=80   # 브링업을 실행한 터미널과 같은 값
   ```

3. **기본 사용 순서**

   ```bash
   ros2 run robot_setting status            # 상태 확인
   ros2 run robot_setting servo_on          # 서보 켜기
   ros2 run robot_setting home              # 작업 홈 [0,0,90,0,90,0]
   ros2 run robot_setting tool_tcp show     # 툴/TCP 확인 (툴 무게가 설정돼 있어야 힘 값이 맞음)
   ros2 run robot_setting direct_teach      # 콕핏으로 위치 교시 → JSON 저장
   ros2 run pjt_test1 move_to_teach         # 저장한 위치로 이동
   ros2 run pjt_test1 measure_teach_force   # 위치마다 Z축 힘 측정
   ```

## 데이터 저장 위치

| 내용 | 위치 | 만드는 실행 파일 |
|---|---|---|
| 교시 위치 (JSON) | `~/ws_cobot_pjt/teach_points/` | `robot_setting direct_teach` |
| 위치별 힘 측정 (CSV) | `~/ws_cobot_pjt/force_logs/` | `pjt_test1 measure_teach_force` |
| 센서 읽기 (CSV) | 실행한 폴더 | `pjt_test1 read_sensor` |

## 공통 주의사항

- **처음 실행할 때는 펜던트 비상정지를 손에 들고** 실행하세요. 동작 속도는 낮게 잡아 두었습니다(`movej` 30 deg/s).
- 로봇 모드: 프로그램 동작(`movej`, `movel`)은 **AUTONOMOUS**, 툴/TCP 설정과 직접 교시는 **MANUAL**에서만 됩니다. 각 스크립트가 필요한 모드로 전환하고, 설정 스크립트는 끝나면 원래 모드로 되돌립니다.
- 동기 `movel`/`movej`가 실행되는 동안에는 컨트롤러가 다른 서비스 요청을 처리하지 못합니다. 그래서 다른 터미널에서 읽는 센서 값이 이동 중에는 멈췄다가 이어집니다.
