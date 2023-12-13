# Requirements:
bash install_req.bash

# How to run:
1. Put package into workspace
2. ``` catkin build ```
3. ``` source devel/setup.bash ```
4. ``` roslaunch ros_audio_pkg speech_recognition.launch ```

# References:
1. VAD settings: https://github.com/Uberi/speech_recognition/blob/1b737c5ceb3da6ad59ac573c1c3afe9da45c23bc/speech_recognition/__init__.py#L332
2. SpeechRecognition library documentation: https://pypi.org/project/SpeechRecognition/
3. 'Whisper' is built into the library, check the documentation attached above to understand how to configure it
