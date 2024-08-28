#!/bin/env bash

cp scripts/rules/* /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger