# 복사 상태와 확인 사항

사용자가 마지막에 지정한 Localization, Perception, Global Planning, Control, Mapping 파일 목록을 복사했습니다. 원본 코드는 수정하지 않았습니다.

- maps/0824.yaml 및 PNG는 assets/maps에 보관했습니다.
- Global Planning 지도와 결과는 최종 사용 지도 미지정으로, _final을 제외한 지도 파일과 허용한 출력 파일 종류만 보존했습니다. 모든 결과가 기존 생성기의 산출물이라는 뜻은 아니며 생성 이력 확인이 필요합니다.
- _final 코드, 수동 편집 도구, 편집 프로필, edited CSV, 디버그 이미지, 가상환경, 빌드 산출물은 복사하지 않았습니다.
- 현재 params.yaml은 0824_final을 가리키지만 그 지도는 제외되어 있습니다. lane_generator.py는 0816 지도 절대경로를 사용합니다. 설정은 원본 그대로이며 실행 준비 상태가 아닙니다.
- perception의 opponent_track.launch 설정 경로만 $(find perception) 형식으로 수정했습니다. 다른 절대경로와 외부 의존성은 검토가 필요하며 ROS 빌드는 실행하지 않았습니다.
- AMCL, robot_localization, map_server, Cartographer 등 외부 패키지 소스는 이번 목록에 포함하지 않았습니다.
- local-planning에는 후속 요청에 따라 5개 코드를 추가했습니다. 선정 근거는 local-planning-selection.md에 기록했습니다. vehicle-interface, common-interfaces, assets/trajectories는 아직 빈 폴더입니다.
- 코드 출처와 라이선스 검토는 완료되지 않았습니다. Git 초기화, commit, push는 수행하지 않았습니다.

## 설정 대체 완료
- 미발견 opponent_tracker_params2.yaml 대신 사용자 지정 opponent_tracker_params221.yaml을 원본 그대로 복사했습니다.
- 사본 opponent_track.launch의 rosparam 경로를 $(find perception)/cfg/opponent_tracker_params221.yaml로 변경했습니다.
- 실행 노드는 기존 detect_4.py와 dynamic_tracker_server.py를 유지했습니다.
- 원본 opponent_track221.launch는 detect_22.py와 obstacle_velocity_estimator.py를 실행하는 별도 구성이라 이번에는 복사하거나 교체하지 않았습니다.
- copy-manifest.csv의 sha256은 최초 복사한 원본 내용 기준입니다. 이후 변경 내용은 local-changes.md에 기록합니다.
