#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BOT_DIR="${SCRIPT_DIR}/../Bot_Cognitive_ChatGpt"

cd $BOT_DIR

rasa run actions