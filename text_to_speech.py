
from dotenv import load_dotenv
import os
from openai import OpenAI
from pydub import AudioSegment
from pydub.playback import play
import time


load_dotenv()

class Speech:
    def __init__(self):
        
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.AudioSegment = AudioSegment
        self.play = play
        self.time = time

    def get_text_to_speech(self, text="Hello Derek and Pranav, what are you guys up to"):
        filename = "output.mp3"

        with self.client.audio.speech.with_streaming_response.create(
            model="tts-1", voice="alloy", input=text
        ) as response:
            response.stream_to_file(filename)

        return filename

    def read_audio(self, file):
        # Load the audio file
        audio = self.AudioSegment.from_file(file)

        # Play the audio file
        self.play(audio)

    def execute(self):
        start_time = self.time.time()

        filename = self.get_text_to_speech()

        generation_time = self.time.time() - start_time
        print(f"Audio generation time: {generation_time:.2f} seconds")

        self.read_audio(filename)

        total_time = self.time.time() - start_time
        print(f"Total execution time: {total_time:.2f} seconds")


if __name__ == "__main__":
    speech = Speech()
    speech.execute()