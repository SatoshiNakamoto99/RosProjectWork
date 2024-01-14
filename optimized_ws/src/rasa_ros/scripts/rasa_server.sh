#!/bin/bash

BOT_DIR="/home/satoshinakamoto/CognitiveRobotics/Bot_Cognitive_ChatGpt"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
