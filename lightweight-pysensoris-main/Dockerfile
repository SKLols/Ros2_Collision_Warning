# syntax=docker/dockerfile:1

FROM python:3.8-slim-buster

WORKDIR /app

COPY requirements.txt requirements.txt
RUN pip3 install -r requirements.txt

COPY . .

CMD [ "python3", "main_sending.py", "-ip", "127.0.0.1", "-pt", "10001", "-tn", "mast_5/fused_tracked_objects", "-nn", "test_listener"]


