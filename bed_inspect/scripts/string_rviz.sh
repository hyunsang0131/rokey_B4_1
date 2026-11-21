#!/bin/bash
# 나사 실 고정/해제를 RViz 가상 모드(에뮬레이터)로 보기.
#   ~/ws_cobot_pjt/ws_dsr/src/bed_inspect/scripts/string_rviz.sh v1 fix     (v1|v2, fix|unfix|all)
#   all = 같은 버전으로 fix 다음 unfix 연속
# 실물(도메인 80)과 안 겹치게 ROS_DOMAIN_ID=77 로 가상 bringup 을 띄우고, 컨트롤러가 뜨면 동작을 실행한다.
# 첫 실행 때 에뮬레이터가 늦게 떠서 컨트롤러가 죽으면(INITIAL STATE CALL FAILURE) bringup 을 자동으로 한 번 더 띄운다.
# 동작이 끝나도 RViz 는 남겨 둠. Ctrl+C 로 전부 끔.
VER=${1:-v1}; ACT=${2:-fix}
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
export ROS_DOMAIN_ID=77
export BED_VIRTUAL=1
BR=
cleanup() { [ -n "$BR" ] && kill -INT $BR 2>/dev/null; wait $BR 2>/dev/null; }
trap cleanup EXIT

ready() { ros2 service list 2>/dev/null | grep -q /dsr01/dsr_controller2/system/get_robot_state; }

for try in 1 2 3; do
  echo "=== 가상 bringup 시작 (시도 $try) ==="
  ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual &
  BR=$!
  ok=0
  for i in $(seq 1 30); do
    sleep 2
    if ready; then ok=1; break; fi
    if ! kill -0 $BR 2>/dev/null; then break; fi
  done
  if [ $ok = 1 ]; then
    sleep 3
    if ready && ros2 service call /dsr01/dsr_controller2/system/get_robot_state dsr_msgs2/srv/GetRobotState "{}" 2>/dev/null | grep -q "success=True"; then
      break
    fi
  fi
  echo "=== 컨트롤러가 안 떴습니다 → bringup 다시 띄움 ==="
  kill -INT $BR 2>/dev/null; wait $BR 2>/dev/null; BR=
  sleep 3
done
[ -z "$BR" ] && { echo "가상 bringup 실패"; exit 1; }

if [ "$ACT" = all ]; then
  ros2 run bed_inspect string_$VER fix --virtual --yes && ros2 run bed_inspect string_$VER unfix --virtual --yes
else
  ros2 run bed_inspect string_$VER $ACT --virtual --yes
fi
echo "끝. RViz 를 더 보려면 그대로 두고, 닫으려면 Ctrl+C"
wait $BR
