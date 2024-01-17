import os


# The time of calibration when the audio start
# -- 0.2 seems working with low noises scene
CALIBRATION_TIME = 0.2 

# Flag: if False, the working of the chatbot will be simulated with the command line.
# -- In the use must be True.
CHATBOT_RUNNING = False

# Size of the time window for the computation of the mel spectrogram
CHUNK_SIZE = 1024


# This parameter is the minimum confidence to the detection.
FACE_MIN_DETECTION_CONFIDENCE = 0.8

# The maximum number of frame with different state before changing that
HUMAN_PRESENCE_GHOST_FRAME = 5

# Topic in witch a std_msgs/Bool will be published if an human is present or not. 
HUMAN_PRESENCE_TOPIC = '/track/human_presence'

# The language for the speech recognition.
# -- Chatbot is english, so en-GB
LANGUAGE='en-GB'

# pyAudio microphone idex. That need to be changed basing on the device. 
MICROPHONE_INDEX = None


#If Model Classe is True I load the model opencv else resnet + ssd
MODEL_CLASSE = False

# Flag: If False, the services who need Pepper are not called. Useful for debug.
# -- In the use must be True.
ON_PEPPER = False

# If true, the raw audio will be saved in a folder. We used that to made test at home.
# -- In the use must be False (to improve the fastness).
SAVE_RAW_AUDIO = True

# If true, all the frame from the camera will be saved to make test at home.
# -- In the use must be False (to improve the fastness).
SAVE_RAW_FRAME = True

# Audio sample rate
RATE = 16000


# Path of scripts folder obtained dinamically.
REF_PATH = os.path.dirname(os.path.abspath(__file__))

# User input audio topic with std_msgs/Int16MultiArray
USER_AUDIO_TOPIC = '/audio/user_input'

# User input text topic with std_msgs/String
USER_INPUT_TOPIC = '/chatbot/input'

# Chatbot output text topic with std_msgs/String
CHATBOT_OUTPUT_TOPIC = '/chatbot/output'

# Pepper Talk Topic with std_msgs/Bool
PEPPER_TALK_TOPIC = '/pepper/talk'

# The fps published by the image publisher. This must be replicated in 
# pepper_nodes
VIDEO_FPS = 10

# Topic on which will be published the image received by the Pepper camera
VIDEO_TOPIC = '/in_rgb'



VERBOSE = False
