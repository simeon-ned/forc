FROM ghcr.io/astral-sh/uv:0.11.25 AS uv
FROM python:3.12-slim-bookworm
COPY --from=uv /uv /uvx /usr/local/bin/
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ffmpeg libosmesa6 libgl1 libegl1 \
    && rm -rf /var/lib/apt/lists/*
RUN useradd --create-home --shell /bin/bash vscode
USER vscode
ENV MUJOCO_GL=osmesa
