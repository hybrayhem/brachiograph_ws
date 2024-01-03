#!/bin/bash

# Default value
rebuild=true

# Iterate over all arguments
for arg in "$@"
do
    case $arg in --rebuild=*)
        rebuild="${arg#*=}"
        shift
        ;;
    esac
done

image1="hybrayhem/brachiograph-sim"
image2="theasp/novnc"

# Check if the images exist
if [ "$rebuild" != "false" ] || [ -z "$(docker images -q $image1)" ] || [ -z "$(docker images -q $image2)" ]; then
    docker-compose build
fi