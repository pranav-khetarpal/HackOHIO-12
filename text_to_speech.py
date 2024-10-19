

from dotenv import load_dotenv
import os
from openai import OpenAI
from pydub import AudioSegment
from pydub.playback import play
import time

load_dotenv()

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))


def getTextToSpeech(text="Hello Derek and Pranav, what are you guys up to"):
    filename = "output.mp3"

    with client.audio.speech.with_streaming_response.create(
        model="tts-1", voice="alloy", input=text
    ) as response:
        response.stream_to_file(filename)

    return filename


# Function to play the mp3
def readAudio(file):
    # Load the audio file
    audio = AudioSegment.from_file(file)

    # Play the audio file
    play(audio)


if __name__ == "__main__":
    start_time = time.time()

    filename = getTextToSpeech()

    generation_time = time.time() - start_time
    print(f"Audio generation time: {generation_time:.2f} seconds")

    readAudio(filename)

    total_time = time.time() - start_time
    print(f"Total execution time: {total_time:.2f} seconds")
