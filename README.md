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

