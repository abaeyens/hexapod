# Hexapod

## Get up and running
Do the host setup (see below) and then
```bash
git clone TODO
git submodule update --init --recursive
docker compose build --pull
docker compose run --rm app bash
colcon build
source install/setup.bash
```


## Host setup
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
