---
sidebar_position: 17
---

# باب 17: بات چیت کرنے والی روبوٹکس (Conversational Robotics)

## تعارف

جدید انسان نما روبوٹ (Humanoid Robots) کو انسانوں کے ساتھ فطری انداز میں بات چیت کرنے کی ضرورت ہے۔ یہ باب روبوٹ کنٹرول سسٹمز کے ساتھ لارج لینگویج ماڈلز (LLMs) کو مربوط کرنے کا احاطہ کرتا ہے تاکہ آواز کے ذریعے تعامل اور معنوی استدلال (Semantic Reasoning) ممکن ہو سکے۔

:::tip تعلیمی مقاصد
- ROS 2 کے ساتھ LLMs (جیسے GPT-4, Llama 2) کا انضمام
- اسپیچ ٹو ٹیکسٹ (STT) اور ٹیکسٹ ٹو اسپیچ (TTS) کا نفاذ
- "Voice-to-Action" پائپ لائنز کی تیاری
- روبوٹکس کے لیے پرامپٹ انجینئرنگ (Prompt Engineering) کا ڈیزائن
- معنوی استدلال (Semantic Reasoning) کا نفاذ
:::

## آواز کے تعامل کی پائپ لائن (Voice Interaction Pipeline)

### 1. اسپیچ ٹو ٹیکسٹ (Whisper)

```python
import whisper
import pyaudio
import wave

class SpeechRecognizer:
    def __init__(self):
        self.model = whisper.load_model("base")
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
    
    def record_audio(self, duration=5):
        """Record audio from microphone"""
        p = pyaudio.PyAudio()
        stream = p.open(format=self.audio_format,
                        channels=self.channels,
                        rate=self.rate,
                        input=True,
                        frames_per_buffer=1024)
        
        print("Listening...")
        frames = []
        for _ in range(0, int(self.rate / 1024 * duration)):
            data = stream.read(1024)
            frames.append(data)
            
        print("Processing...")
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        # Save temporary file
        with wave.open("temp.wav", "wb") as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(p.get_sample_size(self.audio_format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
            
        return "temp.wav"
    
    def transcribe(self, audio_file):
        """Convert speech to text"""
        result = self.model.transcribe(audio_file)
        return result["text"]
```

### 2. ٹیکسٹ ٹو اسپیچ (Coqui / EdgeTTS)

```python
import edge_tts
import asyncio

class TextToSpeech:
    def __init__(self):
        self.voice = "en-US-AriaNeural"
    
    async def speak(self, text, output_file="output.mp3"):
        """Convert text to speech"""
        communicate = edge_tts.Communicate(text, self.voice)
        await communicate.save(output_file)
        
        # Play audio
        import os
        os.system(f"mpg123 {output_file}")

# Usage wrapper
def say(text):
    tts = TextToSpeech()
    asyncio.run(tts.speak(text))
```

## LLM انضمام (LLM Integration)

### روبوٹس کے لیے پرامپٹ انجینئرنگ

```python
SYSTEM_PROMPT = """
You are a humanoid robot assistant. You can perform physical actions and answer questions.
Your capabilities:
1. pick_up(object_name)
2. move_to(location)
3. wave_hand()
4. look_at(target)

When asked to do something:
- If it's a physical action, output a JSON command: {"action": "name", "params": {...}}
- If it's a question, answer naturally.
- If it's ambiguous, ask for clarification.

User: "Pick up the red bottle."
Assistant: {"action": "pick_up", "params": {"object": "red bottle"}}

User: "Go to the kitchen."
Assistant: {"action": "move_to", "params": {"location": "kitchen"}}

User: "Hello!"
Assistant: "Hello! How can I help you today?"
"""
```

### ChatGPT / GPT-4 انضمام

```python
import openai
import json

class RobotBrain:
    def __init__(self, api_key):
        openai.api_key = api_key
        self.history = [{"role": "system", "content": SYSTEM_PROMPT}]
    
    def think(self, user_input):
        """Process user input and decide action"""
        self.history.append({"role": "user", "content": user_input})
        
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=self.history,
            temperature=0.7
        )
        
        reply = response.choices[0].message.content
        self.history.append({"role": "assistant", "content": reply})
        
        # Check if reply is a command
        try:
            command = json.loads(reply)
            return command, True  # Command, is_action
        except json.JSONDecodeError:
            return reply, False   # Text reply, is_action
```

## Voice-to-Action ROS 2 نوڈ

### مکمل انضمام (Complete Integration)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceActionNode(Node):
    def __init__(self):
        super().__init__('voice_action_node')
        
        # Modules
        self.ear = SpeechRecognizer()
        self.brain = RobotBrain("YOUR_API_KEY")
        self.mouth = TextToSpeech()
        
        # Publishers
        self.cmd_pub = self.create_publisher(String, '/robot_commands', 10)
        
        # Main loop
        self.timer = self.create_timer(1.0, self.loop)
        self.is_listening = False
        
    def loop(self):
        if self.is_listening:
            return
            
        self.is_listening = True
        
        # 1. Listen
        audio_file = self.ear.record_audio()
        text = self.ear.transcribe(audio_file)
        
        if text.strip():
            self.get_logger().info(f"Heard: {text}")
            
            # 2. Think
            response, is_action = self.brain.think(text)
            
            if is_action:
                self.get_logger().info(f"Command: {response}")
                
                # Publish to robot controller
                msg = String()
                msg.data = json.dumps(response)
                self.cmd_pub.publish(msg)
                
                # Confirm action
                action_name = response['action'].replace('_', ' ')
                say(f"Okay, I will {action_name}")
            else:
                # 3. Speak
                self.get_logger().info(f"Reply: {response}")
                say(response)
        
        self.is_listening = False

def main():
    rclpy.init()
    node = VoiceActionNode()
    rclpy.spin(node)
```

## معنوی استدلال (Semantic Reasoning)

### منظر کو سمجھنا (Scene Understanding)

```python
class SceneReasoning:
    def __init__(self, visual_module, brain):
        self.eyes = visual_module
        self.brain = brain
    
    def describe_scene(self):
        """Generate description of visible objects"""
        objects = self.eyes.detect_objects()
        
        description = "I see: " + ", ".join(
            [f"{obj['class']} at {obj['pos']}" for obj in objects]
        )
        return description
    
    def handle_complex_request(self, request):
        """
        User: "Give me something to drink"
        Robot: *looks* -> sees bottle -> "pick_up bottle"
        """
        scene_desc = self.describe_scene()
        
        prompt = f"""
        Context: You are in a room. {scene_desc}
        User Request: "{request}"
        Task: Which object should I interact with to satisfy the request?
        """
        
        response = self.brain.ask(prompt)
        return response
```

## مقامی LLMs (Llama 2 / Mistral)

رازداری اور آف لائن صلاحیت کے لیے، مقامی ماڈلز کا استعمال کریں:

```python
from langchain.llms import LlamaCpp

class LocalBrain:
    def __init__(self):
        self.llm = LlamaCpp(
            model_path="./llama-2-7b-chat.gguf",
            n_gpu_layers=30,  # GPU acceleration
            n_ctx=2048,
            temperature=0.7
        )
    
    def think(self, text):
        prompt = f"User: {text}\nAssistant:"
        return self.llm(prompt)
```

## بات چیت کرنے والی روبوٹکس کا مستقبل

- **ملٹی ماڈل ماڈلز:** GPT-4V (ویژن + زبان)
- **Embodied AI:** PaLM-E, RT-2 (براہ راست ویژن سے ایکشن تک)
- **جذباتی ذہانت:** صارف کے جذبات کا پتہ لگانا
- **ذاتی نوعیت:** وقت کے ساتھ صارف کی ترجیحات سیکھنا

## خلاصہ

✅ اسپیچ ٹو ٹیکسٹ اور ٹیکسٹ ٹو اسپیچ
✅ سٹرکچرڈ کمانڈز کے لیے پرامپٹ انجینئرنگ
✅ Voice-to-Action ROS 2 پائپ لائن
✅ معنوی منظر کا استدلال (Semantic Scene Reasoning)
✅ مقامی LLM انضمام

## پریکٹس مشقیں

1. ایک بنیادی "Simon Says" روبوٹ بنائیں
2. بصری QA لاگو کریں (روبوٹ جو دیکھتا ہے اس کے بارے میں سوالات کا جواب دینا)
3. ملٹی ٹرن کنورسیشن ہینڈلر (Multi-turn conversation handler) بنائیں
4. Jetson Orin پر ایک مقامی LLM تعینات کریں

## نتیجہ

مبارک ہو! آپ نے **فزیکل AI اور ہیومنائڈ روبوٹکس** کی نصابی کتاب مکمل کر لی ہے۔ اب آپ کے پاس جدید ہیومنائڈ روبوٹس بنانے، ان کی نقل (simulate) کرنے، کنٹرول کرنے اور ان سے بات کرنے کا علم موجود ہے۔

**آگے کیا ہے؟**
- روبوٹکس کمیونٹی میں شامل ہوں
- اپنا ہارڈ ویئر بنائیں
- اوپن سورس میں حصہ ڈالیں
- تجربات کرتے رہیں!

---

:::tip کورس مکمل! 🎓
آپ نے ہارڈ ویئر سے لے کر جدید AI تک پورے اسٹیک پر مہارت حاصل کر لی ہے۔ جائیں اور مستقبل بنائیں!
:::
