SimbiconRL=/home/qfe/SimbiconRL/pytorch-a2c-ppo-acktr-gail
SimbiconRL_visual=/home/qfe/SimbiconRL_visual/pytorch-a2c-ppo-acktr-gail

cd /home/qfe/SimbiconRL/pytorch-a2c-ppo-acktr-gail
cp State.py ../../SimbiconRL_visual/pytorch-a2c-ppo-acktr-gail
cp SimbiconController_3d.py ../../SimbiconRL_visual/pytorch-a2c-ppo-acktr-gail

cd /home/qfe/SimbiconRL_visual/pytorch-a2c-ppo-acktr-gail

rm -r trained_models/ppo
mkdir -p $SimbiconRL_visual/trained_models/ppo
mv $SimbiconRL/trained_models/ppo/3d-v61050.pt $Simbicon_visual/trained_models/ppo/3d-v6.pt

