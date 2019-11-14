# Learning finite state machine-based biped controller using deep reinforcement learning 

# Paper
초록 : https://drive.google.com/open?id=1MLpaWqzolFIZ-QLsyJ6RH7mMI2F449wu

논문 : 현재 작업중이며 Pacific Graphics 2020에 제출 예정

# Description

물리 환경에서의 보행동작 제어는 지속적으로 연구되어왔던 주제이며, 최근엔 기존 연구에 심층 강화학습을 결합하여 보행동작을 제어하는 연구가 활발하다.
이 연구에서는 다양한 보행 제어 방법 중, 유한 상태 기계를 사용한 보행 제어기에 강화 학습을 결합한 보행 동작 제어기 모델을 제시한다. 이 모델은 심층 강화학습을 사용하여 보행동작 제어를 학습하며, 학습을 통해 상태 기계가 원하는 보행 동작을 나타낼 수 있도록 최적화 되기 때문에 유한 상태 기계를 미리 사용자가 원하는 보행 동작에 맞게 세심하게 최적화 할 필요가 없다. 강화학습을 통해 주어지는 보상을 최대로 하도록 상황에 맞게 유한 상태 기계가 지속적으로 변화 하기 때문에 사용자와의 상호 작용을 위해서 여러가지 상태 기계를 만들고 최적화 한 후 이를 전환하는 수고 없이 사용자와 보행제어기가 상호작용 할 수 있음을 보인다.
아울러 강화 학습을 사용한 보행동작 제어 연구는 모션 캡처 데이터등 기존의 레퍼런스가 될 수 있는 모션 데이터가 없으면 기괴한 동작으로 학습되는 경우가 잦으나, 이 연구에서는 유한 상태 기계의 특징을 이용하는것으로 모션 데이터 없이도 보다 자연스러운 동작을 생성해 내고 제어할 수 있는것을 보인다.

이를 보이기 위하여 아래와 같은 2가지 보행동작 제어기를 학습하였다.

<img src="https://user-images.githubusercontent.com/47879393/68868887-a8856200-073b-11ea-9413-66fc657a118f.jpg" width="90%"></img>
<img src="https://user-images.githubusercontent.com/47879393/68868890-a91df880-073b-11ea-8bc4-e0c576141b33.jpg" width="90%"></img>

1. 사용자와 상호작용 가능한 보행 방향에 따라 방향 전환을 하며 보행 할 수 있는 제어기
2. 사용자가 상호작용 가능한  매개변수(보폭, 보행 중 들어올리는 발의 높이, 한 걸음에 걸리는 시간)에 적합한 보행동작을 생성하고 직선 보행하는 제어기




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
