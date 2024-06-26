
# A smart wheelchair based on ROS2.

## Install Docker on Ubuntu by `apt`

Please refer to the [official link](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) to install Docker components.
For Ubuntu 20.04, you need to add `/usr/libexec/docker/cli-plugins` to `PATH`, otherwise `docker-compose` cannot be found.

## Setup Devices

### oradar_lidar

to be finished

### wit_ros2_imu
to be finished

## Build and Run

We utilize the Make system to build, run, and debug this project. Please first clone the repository:
```bash
git clone https://github.com/makaspacex/SmartWCHR.git
cd SmartWCHR
```

### On the Wheelchair Machine Side

```bash
# Pull the latest Docker image
docker pull makaspacex/smartwchr:dev

# Open an interactive shell in Docker on the wheelchair side
make docker wc

# Build the wheelchair sensor library
make build wc

# Run wheelchair sensor packages
make run wc
```

### On the Development Machine Side

```bash
# Pull the latest Docker image
docker pull makaspacex/smartwchr:dev

# Open an interactive shell in Docker on the development machine side
make docker dc

# Build the decision-center library
make build dc

# Run decision-center packages
make run dc
```

## Run All Services on the Wheelchair

```bash
# Start all services using Docker Compose
docker-compose up
```

## How to Develop and Debug
A virtual machine in VMware is recommended for high-performance graphical rendering when running rviz and debugging code.
