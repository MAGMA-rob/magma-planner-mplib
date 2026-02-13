#!/bin/bash
set -e

CONTAINER_NAME="magma_mplib"
IMAGE_NAME="magma_mplib_image"

# Build the image
docker build -t "${IMAGE_NAME}" .

# Run container
docker run --rm \
    --name "${CONTAINER_NAME}" \
    -p 8000:8000 \
    "${IMAGE_NAME}"
