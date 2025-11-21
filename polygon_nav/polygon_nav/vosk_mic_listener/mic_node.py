import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import String as RosString

import pyaudio
from vosk import Model, KaldiRecognizer

from .config import (
    MODEL_PATH,
    SAMPLE_RATE,
    DEVICE_PREFERRED_NAME,
    WAKE_WORDS,
    COMMANDS_AFTER_WAKE,
    GRAMMAR,
)


class VoskMicNode(Node):
    def __init__(self):
        super().__init__("vosk_mic_node")

        self.awaiting_command = False
        self.declare_parameter("model_path", MODEL_PATH)
        self.declare_parameter("sample_rate", SAMPLE_RATE)
        self.declare_parameter("grammar", GRAMMAR)

        model_path = self.get_parameter("model_path").value
        sample_rate = self.get_parameter("sample_rate").value
        grammar_list = self.get_parameter("grammar").value

        self.sample_rate = int(sample_rate)

        self.get_logger().info(f"Loading Vosk model from: {model_path}")
        model = Model(model_path)

        if grammar_list:
            grammar_json = json.dumps(grammar_list, ensure_ascii=False)
            self.get_logger().info(f"Using grammar: {grammar_json}")
            self.recognizer = KaldiRecognizer(model, self.sample_rate, grammar_json)
        else:
            self.get_logger().info("Free speech mode enabled (no grammar)")
            self.recognizer = KaldiRecognizer(model, self.sample_rate)

        # HIER MUSS ROSSTRING SEIN FABIAN
        self.command_pub = self.create_publisher(RosString, "/voice_commands", 10)
        self.log_pub = self.create_publisher(RosString, "/voice_commands_log", 10)
        self.mic = pyaudio.PyAudio()

        self.get_logger().info("Available audio devices:")
        device_index = None
        preferred_lower = DEVICE_PREFERRED_NAME.lower()

        for i in range(self.mic.get_device_count()):
            info = self.mic.get_device_info_by_index(i)
            name = info.get("name", "unknown")
            max_in = info.get("maxInputChannels", 0)
            self.get_logger().info(f"  {i}: {name} (maxInputChannels={max_in})")

            if max_in > 0 and preferred_lower in name.lower():
                device_index = i

        if device_index is None:
            for i in range(self.mic.get_device_count()):
                info = self.mic.get_device_info_by_index(i)
                max_in = info.get("maxInputChannels", 0)
                if max_in > 0:
                    device_index = i
                    break

        if device_index is None:
            raise RuntimeError("No input audio device found")

        self.get_logger().info(f"Using input device index: {device_index}")

        self.stream = self.mic.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=4000,
        )
        self.stream.start_stream()

        self.timer = self.create_timer(0.1, self.process_audio)
        self.get_logger().info("VoskMicNode started, listening on microphone...")

    def process_audio(self):
        if not self.stream.is_active():
            return

        data = self.stream.read(4000, exception_on_overflow=False)
        if len(data) == 0:
            return

        if self.recognizer.AcceptWaveform(data):
            result = self.recognizer.Result()
            self.get_logger().info(f"Final raw result: {result}")

            try:
                res = json.loads(result)
            except json.JSONDecodeError:
                self.get_logger().warn("Failed to parse recognizer result")
                return

            text = res.get("text", "").strip().lower()
            if text:
                self.get_logger().info(f"Recognized text: '{text}'")
                self.handle_text(text)
        else:
            partial_msg = RosString()
            partial = self.recognizer.PartialResult()
            partial_msg.data = partial
            self.log_pub.publish(partial_msg)

    def handle_text(self, text: str):
        # Check for wake word
        if any(w in text for w in WAKE_WORDS):
            self.awaiting_command = True
            self.get_logger().info(f"Wake word detected in: '{text}'")
            return

        if self.awaiting_command:
            for cmd_phrase, info in COMMANDS_AFTER_WAKE.items():
                if cmd_phrase in text:
                    publish_value = info.get("publish", cmd_phrase)
                    log_msg = info.get("log", f"Command: {cmd_phrase}")

                    self.get_logger().info(f"Command sequence detected: '{cmd_phrase}'")
                    self.get_logger().info(log_msg)
                    msg = RosString()
                    msg.data = publish_value
                    self.command_pub.publish(msg)

                    self.awaiting_command = False
                    return

            self.get_logger().info(f"Unknown command after wake word: '{text}'")
            self.awaiting_command = False
        else:
            self.get_logger().info(f"Ignoring text without wake word: '{text}'")

    def destroy_node(self):
        try:
            if self.stream is not None:
                self.stream.stop_stream()
                self.stream.close()
            if self.mic is not None:
                self.mic.terminate()
        except Exception as e:
            self.get_logger().warn(f"Error while closing audio: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoskMicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
