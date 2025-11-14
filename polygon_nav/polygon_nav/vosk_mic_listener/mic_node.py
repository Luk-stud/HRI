import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import pyaudio
from vosk import Model, KaldiRecognizer


class VoskMicNode(Node):
    def __init__(self):
        super().__init__("vosk_mic_node")

        default_model_path = "/ros2_ws/src/vosk_mic_listener/vosk_mic_listener/model/vosk-model-de-0.21"

        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("sample_rate", 16000.0)
        self.declare_parameter("grammar", [])

        model_path = self.get_parameter("model_path").value
        sample_rate = self.get_parameter("sample_rate").value
        grammar_list = self.get_parameter("grammar").value

        self.get_logger().info(f"Loading Vosk model from: {model_path}")
        model = Model(model_path)

        if grammar_list:
            grammar_json = json.dumps(grammar_list)
            self.recognizer = KaldiRecognizer(model, sample_rate, grammar_json)
            self.get_logger().info(f"Using grammar: {grammar_json}")
        else:
            self.recognizer = KaldiRecognizer(model, sample_rate)
            self.get_logger().info("Free speech mode enabled")

        self.command_pub = self.create_publisher(String, "voice_commands", 10)

        self.sample_rate = int(sample_rate)
        self.mic = pyaudio.PyAudio()

        self.get_logger().info("Available audio devices:")
        input_device_index = None
        input_device_info = None

        preferred_name = "Samson Q2U"

        for i in range(self.mic.get_device_count()):
            info = self.mic.get_device_info_by_index(i)
            name = info.get("name", "unknown")
            max_in = info.get("maxInputChannels", 0)
            self.get_logger().info(f"  {i}: {name} (maxInputChannels={max_in})")

            if max_in > 0 and preferred_name.lower() in name.lower():
                input_device_index = i
                input_device_info = info
                break

        if input_device_index is None:
            for i in range(self.mic.get_device_count()):
                info = self.mic.get_device_info_by_index(i)
                name = info.get("name", "unknown")
                max_in = info.get("maxInputChannels", 0)
                if max_in > 0:
                    input_device_index = i
                    input_device_info = info
                    break

        if input_device_index is None:
            raise RuntimeError("No input audio device found in container")

        self.get_logger().info(f"Using input device index: {input_device_index}")

        self.stream = self.mic.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            input_device_index=input_device_index,
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

        self.get_logger().info(f"Audio chunk received: {len(data)} bytes")

        if self.recognizer.AcceptWaveform(data):
            result = self.recognizer.Result()
            self.get_logger().info(f"Final: {result}")

            try:
                res = json.loads(result)
            except json.JSONDecodeError:
                return

            text = res.get("text", "").strip()
            if text:
                msg = String()
                msg.data = text
                self.command_pub.publish(msg)
                self.handle_command(text)
        else:
            partial = self.recognizer.PartialResult()
            self.get_logger().info(f"Partial: {partial}")

    def handle_command(self, text: str):
        if "bello" in text:
            self.get_logger().info("Bello wurde gerufen")
        if "sitz" in text:
            self.get_logger().info("Bello macht sitz!")
        if "platz" in text:
            self.get_logger().info("Bello macht platz!")
        if "beifuß" in text or "bei fuß" in text:
            self.get_logger().info("Bello geht bei Fuß!")

    def destroy_node(self):
        try:
            self.stream.stop_stream()
            self.stream.close()
            self.mic.terminate()
        except:
            pass
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
