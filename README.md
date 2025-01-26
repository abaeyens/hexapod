# Hexapod

## Get up and running
```bash
git clone git@github.com:abaeyens/hexapod-MPC.git
cd hexapod-MPC
git submodule update --init --recursive

echo -e USER_ID=$(id -u $USER)\\nGROUP_ID=$(id -g $USER) >> .env
docker compose build --pull
docker compose run --rm app bash

colcon build
source install/setup.bash
```
To finish, do the host setup (see below) and reboot the host.

## Host setup
For improved DDS performance, a few changes on the host side are necessary.
From this repo's root directory:
```bash
sudo ln -s ${PWD}/etc/sysctl.d/ros2-dds.conf /etc/sysctl.d/ros2-dds.conf
```

## Leg and joint naming and indexing
@pedro feel free to comment on this!
- Legs are indexed numerically against the clock
  starting from the left front leg. Visualized:
  ```
         /front\
  leg 0 |       | leg 5
        |       |
  leg 1 |       | leg 4
        |       |
  leg 2 |       | leg 3
         \_____/
  ```
- For each leg, leg joints are indexed alphabetically starting from the torso.
  Drawing:
  ```
  feet -- joint c -- joint b -- joint a -- torso
  ```
  So, for example, joint 3a points to the shoulder joint
  of the leg on the rear right.
- Leg joints are indexed numerically
  following the expression `leg_index*3 + joint_index_per_leg`.
  So, for example, from its feet to leg, the second leg has as structure:
  ```
  feet1 -- joint 5 -- joint 4 -- joint 3 -- torso
  ```
