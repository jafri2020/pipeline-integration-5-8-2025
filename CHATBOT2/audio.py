import os
from dotenv import load_dotenv
from elevenlabs.client import ElevenLabs
from elevenlabs import VoiceSettings, stream

# Load the .env file
load_dotenv()

# Correct variable name: ELEVEN_API_KEY
api_key = os.getenv("ELEVENLABS_API_KEY")

# Check if API key is loaded
if not api_key:
    raise ValueError("ELEVENLABS_API_KEY not found. Make sure it's set in the .env file.")

# ✅ Initialize ElevenLabs client with API key
client = ElevenLabs(api_key=api_key)

import os
# from dotenv import load_dotenv
# from elevenlabs.client import ElevenLabs
# from elevenlabs import VoiceSettings, stream

# # Load the .env file
# load_dotenv()

# # Correct variable name: ELEVEN_API_KEY
# api_key = os.getenv("ELEVENLABS_API_KEY")

# # Check if API key is loaded
# if not api_key:
#     raise ValueError("ELEVENLABS_API_KEY not found. Make sure it's set in the .env file.")

# # ✅ Initialize ElevenLabs client with API key
# client = ElevenLabs(api_key=api_key)

# Define function
def speak(text: str):
    voice_settings = VoiceSettings(
        stability=0.5,
        similarity_boost=0.5,
        style=1,
        latency=4,
        speed=0.92, 
        language_code="en-US",
        apply_text_normalization="on",
        apply_language_text_normalization="auto"
    )

    audio_stream = client.text_to_speech.stream(
        text=text,
        voice_id="TbMNBJ27fH2U0VgpSNko",
        model_id="eleven_flash_v2_5",
        voice_settings=voice_settings
    )

    stream(audio_stream)



def speak_text(name, text: str):
    # Voice ID mapping
    voice_map = {
        "steve_harris": "SOYHLrjzK2X1ezoPC6cr",
        "mike_reynolds": "ZQe5CZNOzWyzPSCn5a3c",
        "jane_thompson": "TbMNBJ27fH2U0VgpSNko"
    }

    # Use lowercase to allow case-insensitive input
    voice_id = voice_map.get(name.lower(), "TbMNBJ27fH2U0VgpSNko")  # Default to Jane

    # Voice settings
    voice_settings = VoiceSettings(
        stability=0.5,
        similarity_boost=0.5,
        style=1,
        latency=4,
        speed=0.92,
        language_code="en-US",
        apply_text_normalization="on",
        apply_language_text_normalization="auto"
    )

    # Stream audio
    audio_stream = client.text_to_speech.stream(
        text=text,
        voice_id=voice_id,
        model_id="eleven_flash_v2_5",
        voice_settings=voice_settings
    )

    stream(audio_stream)


# # ✅ Test the function
# speak_text("I can't believe this moment is finally here — my heart's racing, and I’ve never felt more alive!")




# import os
# import sounddevice as sd
# import numpy as np
# from elevenlabs.client import ElevenLabs
# from elevenlabs import VoiceSettings
# from pydub import AudioSegment
# from dotenv import load_dotenv
# import threading

# # Load environment variables from .env file
# load_dotenv()

# # Get the API key
# ELEVENLABS_API_KEY = os.getenv("ELEVENLABS_API_KEY")

# # Directory for saving audio files
# AUDIO_SAVE_PATH = "saved_audio"
# os.makedirs(AUDIO_SAVE_PATH, exist_ok=True)

# def save_audio(phrase, filename="output.wav"):
#     """Generate speech using ElevenLabs with custom voice settings and save it as a WAV file."""
#     if not ELEVENLABS_API_KEY:
#         raise ValueError("API key is missing! Please provide a valid ElevenLabs API key.")

#     # Initialize ElevenLabs client
#     client = ElevenLabs(api_key=ELEVENLABS_API_KEY)

#     voice_settings = VoiceSettings(
#         stability=0.5,
#         similarity_boost=0.5,
#         style=1,
#         latency=4,
#         #use_speaker_boost=True,
#         speed=0.92,
#         language_code="en-US",
#         apply_text_normalization="on",
#         apply_language_text_normalization="auto"
#     )
#     # Generate speech in MP3 format with custom voice settings
#     audio_stream = client.text_to_speech.convert(
#         text=phrase,
#         # voice_id="XrExE9yKIg1WjnnlVkGX",  # Replace with your desired voice ID
#         voice_id="TbMNBJ27fH2U0VgpSNko",  # Replace with your desired voice ID
#         #voice_id="LcfcDJNUP1GQjkzn1xUU",
#         model_id="eleven_flash_v2_5",
#         output_format="mp3_44100_128", 
#           # Generate MP3 (convert to WAV later)
#         voice_settings=voice_settings
#     )

#     # Read audio data
#     audio_bytes = b"".join(audio_stream)

#     # Save as MP3
#     mp3_path = os.path.join(AUDIO_SAVE_PATH, filename.replace(".wav", ".mp3"))
#     with open(mp3_path, "wb") as f:
#         f.write(audio_bytes)

#     # Convert MP3 to WAV (for better playback control)
#     sound = AudioSegment.from_mp3(mp3_path)
#     wav_path = os.path.join(AUDIO_SAVE_PATH, filename)
#     sound.export(wav_path, format="wav")

#     #print(f"✅ Audio saved at: {wav_path}")
#     return wav_path  # Return the saved WAV file path

# def wait_for_enter_and_stop():
#     """Wait for Enter key press and stop audio."""
#     input("🔘 Press ENTER anytime to stop playback...\n")
#     #print("⏹ Stopping audio...")
#     sd.stop()





# def play_saved_audio_without_interrupt(text, output_device_index=5, volume=70):
#     """Play a saved WAV audio file with volume control (no manual stop)."""
#     file_path = save_audio(text, filename="test_audio.wav")
#     sound = AudioSegment.from_wav(file_path)
    
#     # Adjust volume
#     sound = sound - (20 * np.log10(100 / volume))  # Decrease dBFS to simulate volume change
    
#     # Ensure stereo, 44.1 kHz
#     sound = sound.set_frame_rate(44100).set_channels(2)
    
#     # Convert to NumPy array
#     samples = np.array(sound.get_array_of_samples(), dtype=np.int16)
#     if sound.channels == 2:
#         samples = samples.reshape(-1, 2)
#     samples = samples.astype(np.float32) / 32768.0  # Normalize
    
#     try:
#         #print(f"🔊 Playing on device {output_device_index} at {volume}% volume...")
#         sd.play(samples, samplerate=sound.frame_rate, device=output_device_index)
#         sd.wait()  # Wait until playback finishes
#         #print("✅ Audio playback finished.")
#     except Exception as e:
#         print(f"❌ Error playing audio: {e}")    




# def play_audio_file(file_path, output_device_index=5, volume=10):
#     """Play a saved WAV audio file with volume control. Stop on Enter key press."""
#     try:
#         sound = AudioSegment.from_wav(file_path)
#         sound = sound - (20 * np.log10(100 / volume))  # Adjust volume
#         sound = sound.set_frame_rate(16000).set_channels(2)
#         samples = np.array(sound.get_array_of_samples(), dtype=np.int16)

#         if sound.channels == 2:
#             samples = samples.reshape(-1, 2)

#         samples = samples.astype(np.float32) / 32768.0

#         #print(f"🔊 Playing {file_path} on device {output_device_index} at {volume}% volume...")

#         sd.play(samples, samplerate=sound.frame_rate, device=output_device_index)
#         sd.wait()
#         #print("✅ Audio playback finished.")
#     except Exception as e:
#         print(f"❌ Error playing audio: {e}")



# # def play_saved_audio(text, output_device_index=5, volume=10):
# #     """Play a saved WAV audio file with volume control. Stop on Enter key press."""
# #     file_path = save_audio(text, filename="test_audio.wav")
# #     sound = AudioSegment.from_wav(file_path)
# #     sound = sound - (20 * np.log10(100 / volume))
# #     sound = sound.set_frame_rate(44100).set_channels(2)
# #     samples = np.array(sound.get_array_of_samples(), dtype=np.int16)

# #     if sound.channels == 2:
# #         samples = samples.reshape(-1, 2)

# #     samples = samples.astype(np.float32) / 32768.0

# #     try:
# #         print(f"🔊 Playing on device {output_device_index} at {volume}% volume...")

# #         # Start background thread to listen for Enter key
# #         enter_thread = threading.Thread(target=wait_for_enter_and_stop, daemon=True)
# #         enter_thread.start()

# #         sd.play(samples, samplerate=sound.frame_rate, device=output_device_index)
# #         sd.wait()
# #         #print("✅ Audio playback finished.")
# #     except Exception as e:
# #         print(f"❌ Error playing audio: {e}")

# # Example usage
# # play_saved_audio_without_interrupt("Ohh yeahhh, have fun", output_device_index=2, volume=100)
# # play_saved_audio_without_interrupt("Hello, how are you. How can i assist you", output_device_index=1, volume=30)
# # play_saved_audio_without_interrupt("Ahhh, i am feeling so sad for you ", output_device_index=1, volume=30)
# # import subprocess
# # param_name = "/smiley_param"
# # param_value = "smiling"
# # param_value_listening = "sttrunning"
# # param_value_converstion = "conversation"


# # subprocess.run(["rosparam", "set", param_name, "conversation"])

# # play_saved_audio_without_interrupt("Mr Chen, I see you have been sitting for last 3 hours. I suggest that you take a short walk", output_device_index=6, volume=150)


# # subprocess.run(["rosparam", "set", param_name, "smiling"])
