# SimbiconRL
수정사항

Pydart 2 
- 학습속도 빠르게 하기위해 Contact 계산 안함
- Skeleton의 Position Diff 바인딩 잘못된곳 수정(position과 velocity 반대로 바인딩 되어있음)


Baseline
- ShmemVecEnv와 subproc에서 env에 변수 세팅하기 위해 pipe로 연결하여 함수호출하는 코드 추가
  (silver에 reward cur, eldorado1에 pd controller gain cur 추후 통합)

brach
- master2 // swing hip Z tagetAngle 없음
