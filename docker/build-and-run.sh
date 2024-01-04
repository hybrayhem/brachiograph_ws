#!/bin/bash

./01-build-image.sh #--rebuild=false
./02-run-container.sh

./06-get-container-ip.sh

# ./05-connect-to-container.sh