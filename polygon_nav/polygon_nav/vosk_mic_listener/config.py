MODEL_PATH = "/ros2_ws/src/vosk_mic_listener/vosk_mic_listener/model/vosk-model-de-0.21"

SAMPLE_RATE = 16000.0

DEVICE_PREFERRED_NAME = "Samson Q2U Microphone"

WAKE_WORDS = [
    "snoopy",
]

# Commands that are valid after a wake word
# key: phrase to recognize in the text
# value: dict with what to publish and what to log
COMMANDS_AFTER_WAKE = {
    "sitz": {
        "publish": "dog_sit",
        "log": "Bello macht sitz!",
    },
    "platz": {
        "publish": "dog_down",
        "log": "Bello macht platz!",
    },
    "beifuß": {
        "publish": "dog_heel",
        "log": "Bello geht bei Fuß!",
    },
    "bei fuß": {
        "publish": "dog_heel",
        "log": "Bello geht bei Fuß!",
    },
}

GRAMMAR = list({
    "snoopy", "sitz", "platz", "beifuß", "bei fuß"
    })
