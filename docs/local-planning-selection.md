# Local Planning 코드 선정

원본 코드를 실행하거나 수정하지 않고 전체 26개 Python 파일의 AST 문법, 함수 구성, import, ROS 입출력을 조사했습니다. 추월 후보의 함수 호출과 실행 루프, 두 독립 추적 후보의 인터페이스를 추가 비교했습니다. 실차·시뮬레이션 성능 순위가 아닌 현재 프로젝트에 맞춘 정적 검토 결과입니다.

## 복사한 5개 파일
- follow_the_gap.py: 사용자 지정
- lattice_planner3.py: 사용자 지정
- lattice_planner_follow111.py: 사용자 지정. 추월 함수 정의는 있으나 try_start_overtake/make_active_overtake_path 호출은 없어 현재 루프의 추월 실행 증거로 사용할 수 없습니다.
- lattice_planner3_dynamic_overtake.py: 추가 추월 대표. 실제 loop에서 추월 진입과 활성 경로 함수를 호출합니다. FOLLOW/SHIFT/PASS/RETURN, 후보 지속성, 목표 방향 고정, 벽·장애물 검사와 타임아웃을 구현했습니다. 경로를 못 찾는 분기의 이전 명령 처리 등은 실행 검증이 필요합니다.
- obstacle_velocity_estimator.py: 추가 추적 대표. 기존 global_path 및 raw_obstacles 토픽 입력, 프레임 간 일대일 최근접 매칭, ID 부여, Frenet 순환 보정, EMA 속도 추정, tracked_obstacles 출력이 있습니다. EKF 추적기는 아니며 프레임 누락 시 ID 유지, 시간 역전, 횡이동 위주 장애물 분류 등에 한계가 있습니다.

## 비교 기준
- tracking.py: EKF 기반의 더 풍부한 추적 구현이나 /global_waypoints, /car_state/* 토픽, frenet_converter 및 filterpy 의존성이 현재 구성과 다릅니다. 현재 실차 연결 적합성을 우선해 선택하지 않았습니다.
- dynamic_tracker_server.py: dynamic_reconfigure 설정 서버이며 추적 알고리즘이 아닙니다.
- dynamic_lattice_planner3_.py 및 real333 계열: 동적 분류·추종·회피 후보 로직을 포함하나 별도 추월 단계가 실제 호출되는 대표로 dynamic_overtake 파일을 선정했습니다.
- line_planner_real.py, map_frenet_follow_planner_real.py: 동적 장애물 FOLLOW 위주 구성이므로 추월 대표에서 제외했습니다.
- detect 계열: 장애물 검출이 주 역할이므로 독립 속도 추적 대표와 구분했습니다.

## 의존성과 사용 범위
- 이 폴더에는 요청한 Python 파일 5개만 복사했습니다. 개별 실험 대안이며 동시 실행용 구성이 아닙니다.
- frenet_utils.py, f110_msgs, ROS 메시지/TF, NumPy/SciPy 등이 별도로 필요합니다. frenet_utils.py 사본은 현재 f1tenth-perception/perception/opponent_tracker/src에 있습니다. 이 폴더에서 자동 import되는 상태는 아닙니다.
- 추적 출력과 플래너의 ~obstacle_topic을 확인해 연결해야 합니다. launch 연결이나 알고리즘은 변경하지 않았습니다.
- 추적기는 기능상 Perception이지만 사용자 요청에 따라 local-planning에 함께 보관했습니다.
- 모든 복사본과 원본의 SHA-256 일치 및 원본 폴더 Python 파일 해시 불변을 확인했습니다. Git 업로드와 ROS 실행은 하지 않았습니다.

## 파일별 조사 목록
- detect.py: 문법 통과; 이번 복사 제외; 함수 17개.
- detect_2.py: 문법 통과; 이번 복사 제외; 함수 23개.
- detect_22.py: 문법 통과; 이번 복사 제외; 함수 18개.
- detect_3.py: 문법 통과; 이번 복사 제외; 함수 19개.
- detect_4.py: 문법 통과; 이번 복사 제외; 함수 19개.
- detect_real.py: 문법 통과; 이번 복사 제외; 함수 22개.
- dynamic_lattice_planner3_.py: 문법 통과; 이번 복사 제외; 함수 57개.
- dynamic_tracker_server.py: 문법 통과; 이번 복사 제외; 함수 1개.
- follow_the_gap.py: 문법 통과; 선정; 함수 32개.
- frenet_utils.py: 문법 통과; 이번 복사 제외; 함수 8개.
- lattice_planner.py: 문법 통과; 이번 복사 제외; 함수 32개.
- lattice_planner1.py: 문법 통과; 이번 복사 제외; 함수 33개.
- lattice_planner2.py: 문법 통과; 이번 복사 제외; 함수 47개.
- lattice_planner3.py: 문법 통과; 선정; 함수 57개.
- lattice_planner3_dynamic_overtake.py: 문법 통과; 선정; 함수 66개.
- lattice_planner_follow111.py: 문법 통과; 선정; 함수 77개.
- lattice_planner_real.py: 문법 통과; 이번 복사 제외; 함수 54개.
- lattice_planner_real333.py: 문법 통과; 이번 복사 제외; 함수 47개.
- lattice_planner_real3331.py: 문법 통과; 이번 복사 제외; 함수 47개.
- lattice_planner_real333_follow.py: 문법 통과; 이번 복사 제외; 함수 48개.
- lattice_planner_real333_follow2.py: 문법 통과; 이번 복사 제외; 함수 48개.
- line_planner_real.py: 문법 통과; 이번 복사 제외; 함수 61개.
- map_frenet_follow_planner_real.py: 문법 통과; 이번 복사 제외; 함수 52개.
- obstacle_velocity_estimator.py: 문법 통과; 선정; 함수 5개.
- static_obs_spline_planner_2.py: 문법 통과; 이번 복사 제외; 함수 8개.
- tracking.py: 문법 통과; 이번 복사 제외; 함수 36개.
