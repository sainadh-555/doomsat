# Multi-stage lightweight Docker container for GARUDA-1U Mission Control Dashboard
FROM python:3.11-slim AS builder

WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt pyproject.toml ./
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Final runtime image
FROM python:3.11-slim

WORKDIR /app

# Copy python packages from builder
COPY --from=builder /usr/local/lib/python3.11/site-packages /usr/local/lib/python3.11/site-packages
COPY --from=builder /usr/local/bin /usr/local/bin

COPY . /app
RUN pip install --no-cache-dir -e .

EXPOSE 8000

ENV PORT=8000
ENV PYTHONPATH=/app/src

CMD ["uvicorn", "doomsat.dashboard.app:app", "--host", "0.0.0.0", "--port", "8000"]
