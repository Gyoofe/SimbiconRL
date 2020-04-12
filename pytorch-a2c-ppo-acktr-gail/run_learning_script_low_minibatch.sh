python3 main.py --algo ppo --lr 1e-4 --gamma 0.99 --use-gae --gae-lambda 0.95 --entropy-coef 0.01 --value-loss-coef 0.5 --num-processes 32 --num-steps 512 --ppo-epoch 10 --num-mini-batch 128 --clip-param 0.2 --log-interval 1 --save-interval 10 --num-env-steps 10000000000 --env-name 3d-v6 --use-proper-time-limits

