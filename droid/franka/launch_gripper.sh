source ~/miniconda3/etc/profile.d/conda.sh
conda activate polymetis-local
pkill -9 gripper
chmod a+rw /dev/ttyUSB0
launch_gripper.py gripper=franka_hand gripper.comport=/dev/ttyUSB0
