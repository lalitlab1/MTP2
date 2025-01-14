#!/bin/bash

# Name of the container to enter
CONTAINER_NAME="ros_noetic_container"

# Enter the container using bash
docker exec -it $CONTAINER_NAME /bin/bash
