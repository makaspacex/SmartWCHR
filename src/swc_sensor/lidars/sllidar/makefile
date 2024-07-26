# ROS 2 workspace Makefile
SHELL := /bin/sh

# Variables
ROS2_WS = .
SRC_DIR = $(ROS2_WS)/src
BUILD_DIR = $(ROS2_WS)/build
INSTALL_DIR = $(ROS2_WS)/install
LOG_DIR = $(ROS2_WS)/log
VALID_TARGETS = all build clean run launch docker up
OPTS = $(filter-out $(VALID_TARGETS),$(MAKECMDGOALS))
# BUILD_CC = colcon build --symlink-install --base-paths $(SRC_DIR) --build-base $(BUILD_DIR) --install-base $(INSTALL_DIR) --parallel-workers 4
BUILD_CC = colcon build --base-paths $(SRC_DIR) --build-base $(BUILD_DIR) --install-base $(INSTALL_DIR) --parallel-workers 4
RUN_CC = ros2 run
LAUNCH_CC = ros2 launch

# WHEEL_PACKAGES =  lslidar_driver lslidar_msgs oradar_lidar system_controller swc_nav_bringup wit_ros2_imu 
WHEEL_PACKAGES = driver imu lidar wcmain
DC_PACKAGES = nav2 slam_2d swc_nav_bringup wcmain

WC_NAME = wc
DC_NAME = dc

DOCKER_RUN_ARGS = run -e CUID=$$(id -u) -it --rm smartwchr /bin/zsh
# Check if opts is empty
ifeq ($(strip $(OPTS)),)
    BUILD_PACKAGES = 
	RUN_ARGS =
	LAUNCH_ARGS = 
	DOCKER_ARGS =  -f docker-compose.dc.yml $(DOCKER_RUN_ARGS)
	UP_ARGS =  -f docker-compose.yml up -d
# 处理特殊选项名字
else ifeq ($(OPTS),$(WC_NAME))
	BUILD_PACKAGES = --packages-select $(WHEEL_PACKAGES)
	RUN_ARGS =
	LAUNCH_ARGS = 
	DOCKER_ARGS = -f docker-compose.wc.yml $(DOCKER_RUN_ARGS)
	UP_ARGS =  -f docker-compose.wc.yml up -d

else ifeq ($(OPTS),$(DC_NAME))
	BUILD_PACKAGES = --packages-select $(DC_PACKAGES)
	RUN_ARGS =
	LAUNCH_ARGS =
	DOCKER_ARGS =  -f docker-compose.dc.yml $(DOCKER_RUN_ARGS)
	UP_ARGS =  -f docker-compose.dc.yml up -d

# 否则等于指定选项名字
else
    BUILD_PACKAGES = --packages-select $(OPTS)
	RUN_ARGS = $(OPTS)
	LAUNCH_ARGS = $(OPTS)
endif

# .SILENT:

# Targets
all: build

build:
	$(BUILD_CC) $(BUILD_PACKAGES)
	@echo "\n\nSummary: \n  build options$(BUILD_PACKAGES)"

clean:
	@rm -rf $(BUILD_DIR) $(INSTALL_DIR) $(LOG_DIR)

# @. /opt/ros/humble/setup.zsh && \
# . $(INSTALL_DIR)/local_setup.zsh &&

run:
	$(RUN_CC) $(RUN_ARGS)

launch:
	$(LAUNCH_CC) $(LAUNCH_ARGS)

docker:
	docker-compose $(DOCKER_ARGS)

up:
	docker-compose $(UP_ARGS)

# Phony targets
.PHONY: all build clean run launch docker up

# Ignore targets that are not defined
%:
	@:
