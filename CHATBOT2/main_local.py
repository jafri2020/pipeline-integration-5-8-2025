import azure.cognitiveservices.speech as speechsdk
import os
import time
from dotenv import load_dotenv
import string
from chatbot import chatbot_with_memory
# from ztestaudio import speak
 

import subprocess

#added by hj
from speaker_detc2 import identify_speaker
from record_audio import record
from resemblyzer import VoiceEncoder, preprocess_wav
from pathlib import Path
from resemblyzer import VoiceEncoder, preprocess_wav
import threading


load_dotenv()

# Setup
speech_key = os.getenv("SPEECH_KEY")
service_region = "eastus"
keyword_model_path = "bella.table"
wake_word = "Bella"
encoder = VoiceEncoder()

speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)
audio_config = speechsdk.AudioConfig(use_default_microphone=True)
keyword_model = speechsdk.KeywordRecognitionModel(keyword_model_path)

import csv
import os
from datetime import datetime

def log_conversation_to_csv(user_name: str, user_query: str, ruyi_response: str):
    """
    Logs a conversation row into a single CSV file: conversation_logs/history.csv.
    Each row includes: timestamp, user_name, user_query, ruyi_response.
    """
    # Directory and fixed file path
    log_dir = "conversation_logs"
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, "history.csv")

    # Get timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Write header only if file does not exist
    write_header = not os.path.isfile(log_file)

    # Append row
    with open(log_file, mode='a', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        if write_header:
            writer.writerow(["timestamp", "user_name", "user_query", "ruyi_response"])
        writer.writerow([timestamp, user_name, user_query, ruyi_response])


#------------------ NAVIGATION Replies ---------------------------
def get_navigation_reply(command: str) -> str:
    command = command.lower().strip() 
 
    responses = {
        "stop following me": "Okay, I will stop following you now.",
        "stop follow me": "Alright, I will stay here and not follow you anymore.",
        "stop following": "Understood, I'm stopping now.",
        "pose": "Holding position. Please let me know when you're ready to continue.",
        "come closer": "Coming closer. Please make sure the path is clear.",
        "come close": "Approaching you now. Let me know if I’m too close.",
        "come here": "On my way to you. Stay where you are, please.",
        "go away": "Understood. I will move away and maintain a safe distance.",
        "follow me": "Following you now. Please move slowly and safely.",
        "move close": "Moving closer to you. Please stand still.",
        "stop": "I’ve stopped moving. Awaiting your next instruction.",
        "break": "Pausing all navigation tasks. Say 'resume' to continue.",
        "home": "Returning to the charging station now.",
        "go to home": "Heading back to my home base.",
        "go home": "Navigating to the home station now.",
        "sleep": "Entering sleep mode. Call me if you need assistance.",
        "go to sleep": "Activating sleep mode. I’ll be here if needed.",
        "leave": "Understood. I’ll leave the room quietly.",
        "pause": "Pausing movement. Let me know when to resume.",
        "go to dock": "Okay! Moving towards the dock!",
        "bye": "Byee! Going wo Wakeup word state"
    }
 
    # Find best match if exact key not found (to support slight misspellings or variants)
    for key in responses:
        if key in command:
            return responses[key]
 
    return "I'm sorry, I didn't understand that command. Could you please repeat it?"
 
 
# ------------ Robot Screen ------------
param_name = "/smiley_param"
param_value = "smiling"
param_value_listening = "sttrunning"
param_value_converstion = "conversation"
 
 #-------- Speaker ID------
speaker_audio=5
volume=50
 
#keyword_publisher = KeywordPublisher()
def publish_command(text):
    intent = 'control'
    print("intent is : ", intent)
    reply =get_navigation_reply(text)
   
    #keyword_publisher.publish_keyword(intent, text)
    #play_saved_audio_without_interrupt(reply,speaker_audio,volume)
    print(f"Control Published: {text}")
 
 

def remove_punctuation(text):
    """Removes punctuation from the input text."""
    return text.translate(str.maketrans('', '', string.punctuation))
 
 
 
def on_enter_BOT_PROCESS(text,selected):
    text = text.strip().lower()
    text = remove_punctuation(text)
    # if text in command_phrases:
    #     print("✅ Detected as a robot command.")

    #     publish_command(text)
    #     return


    # else:
  

    print(f"[🤖] Processing through chatbot: {text}")
    response = chatbot_with_memory(text, selected, verbose=False)

    log_conversation_to_csv(selected,text,response)
    on_enter_BOT_RESPONSE(response)

    

def on_enter_BOT_RESPONSE(response):
    print(f"[🔊] Speaking: {response}")
    #subprocess.run(["rosparam", "set", param_name, "conversation"])
    print("Robot is Speaking")
    # play_saved_audio_without_interrupt(response)
    print("speaking ended")
    #subprocess.run(["rosparam", "set", param_name, "smiling"])



# Wake word function
def wait_for_wake_word(speech_config, audio_config, keyword_model):
    recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)
    print("🎤 Listening for wake word...")
    detected = False

    def on_wake(evt):
        nonlocal detected
        if evt.result.reason == speechsdk.ResultReason.RecognizedKeyword:
            print(f"✅ Wake word '{wake_word}' detected!")
            detected = True
            recognizer.recognized.disconnect_all()
            recognizer.canceled.disconnect_all()
            recognizer.stop_keyword_recognition_async().get()

    def on_cancel(evt):
        print(f"⚠ Wake word canceled: {evt.reason}")
        detected = True
        recognizer.recognized.disconnect_all()
        recognizer.canceled.disconnect_all()
        recognizer.stop_keyword_recognition_async().get()

    recognizer.recognized.connect(on_wake)
    recognizer.canceled.connect(on_cancel)
    recognizer.start_keyword_recognition_async(keyword_model).get()

    while not detected:
        time.sleep(0.2)

    return True

def get_persona_name(speaker_name: str) -> str:
    """
    Maps a recognized speaker name to a predefined persona name.
    
    Args:
        speaker_name (str): The name identified by the speaker recognition module.

    Returns:
        str: The mapped persona name if found, else returns the original name.
    """
    mapping = {
        "hasnain": "jane_thompson",
        "kabeer": "mike_reynolds",
        "shahzaib": "steve_harris"
    }

    x=mapping.get(speaker_name.lower(), speaker_name)
    print("check:",x)
    return x
 ######## my changes ##############
speaker_result = {"name": None} 
def run_speaker_identification():
    try:
        input_wav = preprocess_wav(Path("temp_recorded.wav"))
        input_embedding = encoder.embed_utterance(input_wav)
        speaker = identify_speaker(input_emb = input_embedding)  #knownembeddings are aready in the funtion in the speaker_detc.py
        speaker_result["name"] = speaker
        print(f"🧠 Identified speaker: {speaker}")
    except Exception as e:
        print(f"❌ Error in speaker identification: {e}") 

# Speech recognition function
def recognize_speech_loop(speech_config, audio_config, max_attempts=3):
    
    print("🗣 Speech recognition active...")
    fail_count = 0

    while True:

        ######## my changes ##############
        
        record("temp_recorded.wav")
        # run_speaker_identification()
        speaker_thread = threading.Thread(target=run_speaker_identification)
        speaker_thread.start()

        audio_config = speechsdk.AudioConfig(filename="temp_recorded.wav")
        recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)
        result = recognizer.recognize_once_async().get()
        
        speaker_thread.join()

        if result.reason == speechsdk.ResultReason.RecognizedSpeech:
            text = result.text.strip()
            print(f"📝 Recognized: {text}")
            persona_name = get_persona_name(speaker_result["name"])
            on_enter_BOT_PROCESS(text,persona_name) #send the user name.

            if "goodbye" in text.lower():
                print("👋 Detected 'goodbye'. Exiting speech mode...\n")
                break

            fail_count = 0
        else:
            print("🤷 Speech not recognized.")
            fail_count += 1

        if fail_count >= max_attempts:
            print("❌ Too many failed attempts. Returning to wake word mode...\n")
            break

        time.sleep(0.5)


# Main loop
def run_pipeline():
    print("🚀 Wake word + speech loop started...\n")

    while True:
        try:
            if wait_for_wake_word(speech_config, audio_config, keyword_model):
                recognize_speech_loop(speech_config, audio_config, max_attempts=5)
        except Exception as e:
            print(f"❗ Exception: {e}")
            time.sleep(2)

run_pipeline()