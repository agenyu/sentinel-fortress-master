
FROM ubuntu:latest

RUN apt-get update && apt-get install -y \
    g++ \
    cmake \
    make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["bash"]
