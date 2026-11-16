"""gear_assembly.py - 썬기어(유성기어) 조립 노드.

실행 (먼저 m0609_rg2_bringup 이 떠 있어야 합니다)
  실제 로봇: ros2 run rokey1 gear_assembly
  시뮬레이션: ros2 run rokey1 gear_assembly --ros-args -p sim:=true
    - 에뮬레이터에 없는 TCP는 경고만 하고 진행, 잡힘 감지(DI) 검사 끔
"""
import rclpy
import DR_init

from rokey1 import gear_config as cfg

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL


def _setup_tool(dr, log, sim=False):
    """툴/TCP 설정. 좌표가 통째로 어긋나는 사고를 막기 위해 확인이 안 되면 False."""
    if cfg.TOOL_NAME is not None:
        dr.set_tool(cfg.TOOL_NAME)
    current = dr.get_tcp()
    log.info("현재 컨트롤러 TCP: '%s'" % current)
    if cfg.TCP_NAME is None:
        log.warn("TCP를 바꾸지 않고 진행합니다. 티칭할 때와 같은 TCP인지 확인하세요.")
        return True
    if current != cfg.TCP_NAME:
        dr.set_tcp(cfg.TCP_NAME)
        current = dr.get_tcp()
    if current != cfg.TCP_NAME and sim:
        log.warn("[sim] TCP '%s' 가 에뮬레이터에 없어 현재 TCP('%s')로 진행합니다."
                 % (cfg.TCP_NAME, current))
        return True
    if current != cfg.TCP_NAME:
        log.error("TCP를 '%s' 로 설정하지 못했습니다(현재 '%s'). 움직이지 않고 종료합니다."
                  % (cfg.TCP_NAME, current))
        return False
    return True


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("gear_assembly", namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node
    log = node.get_logger()
    sim = node.declare_parameter("sim", False).value
    if sim:
        log.warn("시뮬레이션 모드(sim:=true)")

    try:
        import DSR_ROBOT2 as dr
    except ImportError as e:
        log.error("DSR_ROBOT2 를 불러오지 못했습니다: %s" % e)
        return

    from rokey1.gear_gripper import GearGripper, GearError
    from rokey1.gear_motion import GearMotion

    try:
        if not _setup_tool(dr, log, sim):
            return
        dr.set_singular_handling(dr.DR_AVOID)
        dr.set_velx(*cfg.VELX)
        dr.set_accx(*cfg.ACCX)

        test_mode = cfg.Z_TEST_OFFSET > 0
        if test_mode:
            log.warn("테스트 모드: 모든 Z를 +%.0f mm 띄워서 실행합니다." % cfg.Z_TEST_OFFSET)
        gripper = GearGripper(dr, log, node=node, strict=cfg.STRICT_GRIP_CHECK and not (test_mode or sim))
        motion = GearMotion(dr, gripper, log)

        motion.run_sequence(cfg.SLOTS)
        log.info("조립 완료")

    except KeyboardInterrupt:
        log.info("사용자가 중단했습니다.")
    except GearError as e:
        # 실패 시에는 자동으로 움직이지 않고 그 자리에서 멈춥니다(부품을 쥔 채일 수 있음).
        log.error("공정 중단: %s" % e)
    except Exception as e:
        # DSR_ROBOT2 의 DR_Error(잘못된 인자 등)나 예상 못한 오류도 로그를 남기고 정지
        log.error("예상하지 못한 오류로 중단: %r" % e)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
