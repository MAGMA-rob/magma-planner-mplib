#!/bin/bash
set -e

PORT=""

while getopts "p:" opt; do
  case $opt in
    p) PORT=$OPTARG ;;
    *) echo "Usage: $0 -p <PORT>"; exit 1 ;;
  esac
done

if [ -z "$PORT" ]; then
  echo "Usage: $0 -p <PORT>"
  exit 1
fi

CONTAINER_NAME="magma_mplib"
IMAGE_NAME="magma_mplib_image"

# Build the image
docker build -t "${IMAGE_NAME}" .

# Run container
docker run --rm \
    --name "${CONTAINER_NAME}" \
    -p "${PORT}:8000" \
    -e MAGMA_PLANNER_DEBUG="${MAGMA_PLANNER_DEBUG:-0}" \
    "${IMAGE_NAME}"
