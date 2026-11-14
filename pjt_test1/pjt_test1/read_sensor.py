# 1단계 테스트: 로봇을 움직이지 않고 TCP 위치와 툴 힘만 읽는다.
#
# 확인할 것
#   1) 가만히 있을 때 힘 값의 노이즈(표준편차)가 몇 N 인지
#      → 접촉 감지 임계값(2–5 N)과 열화 단계 간격을 정하는 기준
#   2) 툴을 손으로 위/아래로 눌렀을 때 Fz 가 부호에 맞게 변하는지
#
# 실행: ros2 run pjt_test1 read_sensor
# 종료: Ctrl+C (종료 시 축별 평균/표준편차 출력, CSV 저장)
#
# 주의: 힘 값은 펜던트에 설정된 툴 무게(Tool Weight) 보상이 반영된 값이다.
#       툴 무게 설정이 실제와 다르면 가만히 있어도 Fz 가 0 이 아니게 나온다.

import csv
import statistics
import time

import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
PERIOD = 0.1  # 읽기 주기 [s]. 서비스 호출이라 너무 짧게 잡으면 의미 없음

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pjt_read_sensor", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    log = node.get_logger()

    # 노드를 만든 뒤에 import 해야 한다 (import 시점에 서비스 클라이언트를 만든다)
    try:
        import DSR_ROBOT2
        from DSR_ROBOT2 import get_current_posx, get_tool_force, DR_BASE
    except ImportError as e:
        log.error(f"Error importing DSR_ROBOT2 : {e}")
        rclpy.shutdown()
        return

    for cli in (DSR_ROBOT2._ros2_get_current_posx,
                DSR_ROBOT2._ros2_get_tool_force):
        while not cli.wait_for_service(timeout_sec=1.0):
            log.info(f"Waiting for service: {cli.srv_name}")

    rows = []  # [t, x, y, z, a, b, c, fx, fy, fz, mx, my, mz]
    t0 = time.time()
    log.info("읽기 시작 (Ctrl+C 로 종료). 로봇은 움직이지 않습니다.")

    try:
        while rclpy.ok():
            pos, _ = get_current_posx(ref=DR_BASE)
            force = get_tool_force(ref=DR_BASE)
            if pos is None or force == -1:
                log.warn("읽기 실패, 다시 시도")
                time.sleep(PERIOD)
                continue

            t = time.time() - t0
            rows.append([t, *pos, *force])
            log.info(
                f"t={t:6.1f}s  z={pos[2]:8.2f} mm  "
                f"F=[{force[0]:6.2f} {force[1]:6.2f} {force[2]:6.2f}] N"
            )
            time.sleep(PERIOD)

    except KeyboardInterrupt:
        pass

    finally:
        if len(rows) >= 2:
            print("\n=== 축별 통계 (BASE 기준) ===")
            for name, idx in (("Fx", 7), ("Fy", 8), ("Fz", 9)):
                vals = [r[idx] for r in rows]
                print(f"{name}: 평균 {statistics.mean(vals):7.3f} N, "
                      f"표준편차 {statistics.stdev(vals):6.3f} N")

            fname = time.strftime("read_sensor_%Y%m%d_%H%M%S.csv")
            with open(fname, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "x", "y", "z", "a", "b", "c",
                            "fx", "fy", "fz", "mx", "my", "mz"])
                w.writerows(rows)
            print(f"저장: {fname} ({len(rows)}개 샘플)")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
