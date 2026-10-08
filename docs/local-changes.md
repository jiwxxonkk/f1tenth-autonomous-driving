
## Perception 설정 대체
- 파일: f1tenth-perception/perception/opponent_tracker/launch/opponent_track.launch
- 변경: 누락된 params2.yaml 절대경로를 $(find perception)/cfg/opponent_tracker_params221.yaml로 교체
- 실행 노드 변경 없음
- 수정 후 SHA-256: 4f118af9f0697078f06c09ed21dd4fde938b8cc7d569be4f3a3d8ec850e2bad4
- 원본 YAML 및 launch의 변경 전후 SHA-256 일치 확인
- 검증: YAML 복사 해시 일치, launch XML 파싱 및 설정 참조 확인. ROS 실행 검증은 수행하지 않음.
