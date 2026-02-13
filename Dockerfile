FROM python:3.10-slim

# System dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    gcc \
    g++ \
    python3-dev \
    libffi-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY pyproject.toml .
COPY src ./src


RUN pip install --upgrade pip
RUN pip install .


EXPOSE 8000

# Run planner
CMD ["python3", "-m", "magma_mplib"]