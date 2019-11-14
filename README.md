# Learning finite state machine-based biped controller using deep reinforcement learning 

다음 논문에 관한 코드
초록 : https://drive.google.com/open?id=1MLpaWqzolFIZ-QLsyJ6RH7mMI2F449wu
논문 : 현재 작업중이며 Pacific Graphics 2019에 제출 예정

# Description

물리 환경에서의 보행동작 제어는 지속적으로 연구되어왔던 주제이며, 최근엔 심층 강화학습을 이용하여 움직임을 제어하는 연구가 활발하다.
이 연구에서는 심층 강화학습을 사용하여 유한 상태 기계 기반의 보행동작 제어기를 학습한다. 



# Requirement



수정사항

Pydart 2 
- 학습속도 빠르게 하기위해 Contact 계산 안함
- world.py에서 recording disable 할것
- Skeleton의 Position Diff 바인딩 잘못된곳 수정(position과 velocity 반대로 바인딩 되어있음)]
- sdf파일 수정(backpack)


Baseline
- ShmemVecEnv와 subproc에서 env에 변수 세팅하기 위해 pipe로 연결하여 함수호출하는 코드 추가
  (silver에 reward cur, eldorado1에 pd controller gain cur 추후 통합)

brach
- master2 // swing hip Z tagetAngle 없음
- eldorado1_new2 // baseline의 Model부분 수정(Hidden Layer 부분)
