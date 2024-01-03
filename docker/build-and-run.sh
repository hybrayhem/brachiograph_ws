#!/bin/bash

./01-build-image.sh #--rebuild=false
./02-run-container.sh

./05-get-container-ip.sh

# ./03-connect-to-container.sh