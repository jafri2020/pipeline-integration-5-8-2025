from transitions import Machine
import azure.cognitiveservices.speech as speechsdk
import time
import os
import string
from chatbot import chatbot_with_memory
from audioV02 import play_saved_audio_without_interrupt
from dotenv import load_dotenv
from fastapi import FastAPI, Request
import threading
import uvicorn
from fall_detect_local import start_watching  # <- Import watcher module
# Load environment variables from .env file
 
# from class_keyword_publisher import KeywordPublisher
import subprocess
# import rospy
import re
import signal

#added by hj
from speaker_detc2 import identify_speaker
from record_audio import record
from resemblyzer import VoiceEncoder, preprocess_wav
from pathlib import Path
from resemblyzer import VoiceEncoder, preprocess_wav

 
load_dotenv()
 
shutdown_event = threading.Event()
 
def handle_sigint(sig, frame):
    print("🛑 SIGINT received. Setting shutdown event...")
    shutdown_event.set()
 
signal.signal(signal.SIGINT, handle_sigint)
 
 
 
# ---------------------- Configuration ----------------------
speech_key = os.getenv("SPEECH_KEY")
service_region = "eastus"
keyword_model_path = "rooee.table"
wake_word = "Ruyi"
 
# ---------------------- Command List ----------------------
command_phrases = {
    "stop following me", "stop follow me", "stop following", "pose",
    "come closer", "come close", "come here", "go away",
    "follow me", "move close", "stop", "break", "home", "go to home", "go home"
}
 
# ---------------------- Check Model File ----------------------
if not os.path.exists(keyword_model_path):
    print(f"❌ ERROR: Model file not found at: {keyword_model_path}")
    exit(1)
 
# ---------------------- Setup ----------------------
speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)
keyword_model = speechsdk.KeywordRecognitionModel(keyword_model_path)
 
AUDIO = speechsdk.AudioConfig(use_default_microphone=True, filename= None, stream= None, device_name = None)
 
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
    play_saved_audio_without_interrupt(reply,speaker_audio,volume)
    print(f"Control Published: {text}")
 
 
# ---------------------- FSM States ----------------------
states = [
    'IDLE',              # Initial startup state
    'WAKEWORD',          # Listening for wake word
    'SPEECH_RECOG',      # Speech-to-text
    'BOT_PROCESS',       # Send input to chatbot
    'BOT_RESPONSE',      # Play bot response via TTS
    # 'LISTEN_WAIT',       # 7-second listening window
    'FALL_INTERRUPT',    # Fall detected interrupt
    'FALL_CONVO',        # Fall-related conversation
   # 'RESUME_PREVIOUS'    # Resume the state before interrupt
]
 
 
 
# ---------------------- FSM Class ----------------------
class ChatbotFSM:
    def __init__(self):
        self.machine = Machine(model=self, states=states, initial='IDLE')
        self.state_stack = []  # To  save/restore previous state on interrupt
 
        # using for speech:
        self.last_transcript = ""
        self.last_response = ""
        self.speech_retry_count = 0
        self.fall_info_message = ""  # fallback if no API data yet
        self.max_speech_retries=5

        self.user_name = "" #to store the name of the recognized speaker
        self.encoder = VoiceEncoder()
        # Initialize recognizer once
        try:
            self.recognizer = speechsdk.SpeechRecognizer(
                speech_config=speech_config,
                audio_config=AUDIO
            )
        except Exception as e:
            print(f"❌ Error initializing recognizer: {e}")
            self.recognizer = None
 
 
        # Normal conversation transitions
        self.machine.add_transition('start', 'IDLE', 'WAKEWORD')
        self.machine.add_transition('wakeword_detected', 'WAKEWORD', 'SPEECH_RECOG')
        self.machine.add_transition('speech_done', 'SPEECH_RECOG', 'BOT_PROCESS')
        self.machine.add_transition('bot_replied', 'BOT_PROCESS', 'BOT_RESPONSE')
        self.machine.add_transition('tts_finished', 'BOT_RESPONSE', 'SPEECH_RECOG')
 
        # Fall interrupt flowc
        self.machine.add_transition('fall_detected', '*', 'FALL_INTERRUPT', before='save_current_state')
        self.machine.add_transition('fall_alert_spoken', 'FALL_INTERRUPT', 'FALL_CONVO')
        # Fall conversation continues into chatbot mode
        self.machine.add_transition('fall_convo_done', 'FALL_CONVO', 'SPEECH_RECOG')
 
        # Fallback universal transition to WAKEWORD
        self.machine.add_transition('to_WAKEWORD', '*', 'WAKEWORD')
            
    def _handle_result(self, result):
        print("📤 Handling recognition result...")

        if result.reason == speechsdk.ResultReason.RecognizedSpeech:
            transcript = result.text.strip().lower()
            if transcript:
                print(f"[🗣] Heard: {transcript}")
                self.last_transcript = transcript
                self.speech_retry_count = 0
                self.speech_done()
            else:
                print("⚠ Empty transcript despite RecognizedSpeech.")
                self.speech_retry_count += 1
                self.trigger_speech_recog()

        elif result.reason == speechsdk.ResultReason.NoMatch:
            print(f"⚠ NoMatch — Recognizer heard: '{result.text.strip()}'")
            if result.no_match_details:
                print(f"No match reason: {result.no_match_details.reason}")

            self.speech_retry_count += 1
            print(f"🔁 Retry attempt: {self.speech_retry_count}")

            if self.speech_retry_count >= self.max_speech_retries:
                print("🛑 Max retries reached — Returning to WAKEWORD state.")
                play_saved_audio_without_interrupt("Max retries reached — Returning to WAKEWORD state.")
                self.speech_retry_count = 0
                self.to_WAKEWORD()
            else:
                self.trigger_speech_recog()

        elif result.reason == speechsdk.ResultReason.Canceled:
            cancellation = result.cancellation_details
            print(f"❌ Recognition canceled: {cancellation.reason}")

            if cancellation.reason == speechsdk.CancellationReason.Error:
                print(f"   Error details: {cancellation.error_details}")
                if hasattr(cancellation, 'error_code'):
                    print(f"   Error code: {cancellation.error_code}")

            self.speech_retry_count += 1
            print(f"🔁 Retry attempt (cancel): {self.speech_retry_count}")
            self.trigger_speech_recog()

        else:
            print(f"⚠ Unhandled recognition result reason: {result.reason}")
            self.trigger_speech_recog()

    

    def recognize_speech_once(self):
        if self.recognizer is None:
            print("⚠ Recognizer is not initialized.")
            self.trigger_speech_recog()
            return

        def get_persona_name(speaker_name: str) -> str:
            mapping = {"hasnain": "jane_thompson","kabeer": "mike_reynolds","shahzaib": "steve_harris"}
            x=mapping.get(speaker_name.lower(), speaker_name)
            print("check:",x)
            return x

        def safe_recognize():
            try:
                # subprocess.run(["rosparam", "set", param_name, "sttrunning"])
                print("🎙️ Listening --- Library Invoked")
                
                ######## my changes ##############
                speaker_result = {"name": None} 
                def run_speaker_identification():
                    try:
                        input_wav = preprocess_wav(Path("temp_recorded.wav"))
                        input_embedding = self.encoder.embed_utterance(input_wav)
                        speaker = identify_speaker(input_emb = input_embedding)  #knownembeddings are aready in the funtion in the speaker_detc.py
                        speaker_result["name"] = speaker
                        self.user_name = get_persona_name(speaker_result["name"])

                        print(f"🧠 Identified speaker: {speaker}")
                    except Exception as e:
                        print(f"❌ Error in speaker identification: {e}") 
                
                record("temp_recorded.wav")
                speaker_thread = threading.Thread(target=run_speaker_identification)
                speaker_thread.start()

                audio_config = speechsdk.AudioConfig(filename="temp_recorded.wav")
                recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)
                result = recognizer.recognize_once_async().get()
                
                speaker_thread.join()
                ############# my changes ended ###############

                print("📥 Recognition returned.")

                # subprocess.run(["rosparam", "set", param_name, "smiling"])
                
                try:
                    self._handle_result(result)
                except Exception as e:
                    print(f"❌ Exception while handling result: {e}")
                    self.trigger_speech_recog()

            except Exception as e:
                print(f"❌ Recognizer crashed or timed out.")
                print(f"   ➤ Exception type: {type(e).__name__}")
                print(f"   ➤ Details: {e}")
                print(f"   ➤ Current state: {self.state}")
                self.speech_retry_count += 1
                print(f"🔁 Retry attempt (exception): {self.speech_retry_count}")
                self.trigger_speech_recog()


        threading.Thread(target=safe_recognize, daemon=True).start()

    
    # def recognize_speech_once(self):
    #     if self.recognizer is None:
    #         print("⚠ Recognizer is not initialized.")
    #         self.trigger_speech_recog()  # retry safely
    #         return

    #     try:
    #         subprocess.run(["rosparam", "set", param_name, "sttrunning"])
    #         result = self.recognizer.recognize_once_async().get()
    #         subprocess.run(["rosparam", "set", param_name, "smiling"])
    #     except Exception as e:
    #         print(f"❌ Recognizer crashed or timed out.")
    #         print(f"   ➤ Exception type: {type(e).__name__}")
    #         print(f"   ➤ Details: {e}")
    #         print(f"   ➤ Current state: {self.state}")
    #         self.speech_retry_count += 1
    #         print(f"🔁 Retry attempt (exception): {self.speech_retry_count}")
    #         self.trigger_speech_recog()
    #         return



    #     if result.reason == speechsdk.ResultReason.RecognizedSpeech:
    #         transcript = result.text.strip().lower()
    #         if transcript:
    #             print(f"[🗣] Heard: {transcript}")
    #             self.last_transcript = transcript
    #             self.speech_retry_count = 0  # reset on success
    #             self.speech_done()
    #             return

    #     # elif result.reason == speechsdk.ResultReason.NoMatch:
    #     #     print(f"⚠ NoMatch — Recognizer heard: '{result.text.strip()}'")
    #     #     print("❌ No usable input. Returning to speech recognition.")
    #     #     self.speech_retry_count += 1
    #     #     print(f"🔁 Retry attempt: {self.speech_retry_count}")
    #     #     self.trigger_speech_recog()
    #     #     return
        
    #     elif result.reason == speechsdk.ResultReason.NoMatch:
    #         print(f"⚠ NoMatch — Recognizer heard: '{result.text.strip()}'")
    #         self.speech_retry_count += 1
    #         print(f"🔁 Retry attempt: {self.speech_retry_count}")

    #         if self.speech_retry_count >= self.max_speech_retries:
    #             print("🛑 Max retries reached — Returning to WAKEWORD state.")
    #             play_saved_audio_without_interrupt("Max retries reached — Returning to WAKEWORD state.")
    #             self.speech_retry_count = 0
    #             self.to_WAKEWORD()
    #         else:
    #             self.trigger_speech_recog()  # Safe re-entry in a thread

                    
 
    def trigger_speech_recog(self):
        """Re-enters speech recognition safely in a new thread."""
        def retry():
            retry_delay = 0.5 + self.speech_retry_count * 0.5
            time.sleep(retry_delay)  # Progressive delay

            if self.state == 'SPEECH_RECOG':
                self.on_enter_SPEECH_RECOG()

        threading.Thread(target=retry, daemon=True).start()

    
    def save_current_state(self):
        print(f"[🧠] Saving state: {self.state}")
        self.state_stack.append(self.state)
 
    def restore_previous_state(self):
        if self.state_stack:
            previous = self.state_stack.pop()
            print(f"[🔁] Resuming state: {previous}")
            if previous == 'WAKEWORD':
                self.speech_retry_count = 0
                self.to_WAKEWORD()
            elif previous == 'SPEECH_RECOG':
                self.wakeword_detected()
            elif previous == 'BOT_PROCESS':
                self.speech_done()
            elif previous == 'BOT_RESPONSE':
                self.bot_replied()
            # elif previous == 'LISTEN_WAIT':
            #     self.tts_finished()
            else:
                print(f"[⚠] Unknown state '{previous}', falling back to WAKEWORD")
                self.speech_retry_count = 0
                self.to_WAKEWORD()
        else:
            print("[⚠] No previous state saved, going to WAKEWORD")
            self.speech_retry_count = 0
            self.to_WAKEWORD()
 
 
    def on_enter_SPEECH_RECOG(self):
        print(f"[🎙] Entered SPEECH_RECOG — streaming for command")
        self.recognize_speech_once()
 
 
 
    def on_enter_BOT_PROCESS(self):
        text = self.last_transcript.strip().lower()
        text = remove_punctuation(text)
         
       
        if text in command_phrases:
            print("✅ Detected as a robot command.")
 
            publish_command(text)
            self.speech_retry_count = 0
            self.to_WAKEWORD()
 
        else:
            print(f"[🤖] Processing through chatbot: {text}")
            self.last_response = chatbot_with_memory(text, self.user_name, verbose=False)
 
            #publish_convo(self.last_response)
            self.bot_replied()
 
       
 
 
    def on_enter_BOT_RESPONSE(self):
        print(f"[🔊] Speaking: {self.last_response}")
 
        # subprocess.run(["rosparam", "set", param_name, "conversation"])
 
        play_saved_audio_without_interrupt(self.last_response,speaker_audio,volume)
 
        # subprocess.run(["rosparam", "set", param_name, "smiling"])
 
        if self.state == "BOT_RESPONSE":  # Only trigger if still in expected state
            self.tts_finished()
        else:
            print(f"[⚠] Skipping tts_finished — current state is {self.state}")
 
 
 
    def on_enter_FALL_INTERRUPT(self):
        print("[🗣] Starting fall-related conversation...")
 
        try:
            response = chatbot_with_memory(
                self.last_transcript + "Note : As the User Fall! Ask Him about his health as a Healthcare-Companion. Ensure is User Okay need any help. Give as Much as Human Behaviour as you can. Make it concise response not a long paragraphic ones",
                verbose=False
            )
            self.last_response = response
            print("Fall Detected: ", response)
            play_saved_audio_without_interrupt(response,speaker_audio,volume)
            # ✅ Only trigger transition if state is still valid
            if self.state == "FALL_INTERRUPT":
                self.fall_alert_spoken()
            else:
                print(f"[⚠] Skipping fall_alert_spoken — current state is {self.state}")
        except Exception as e:
            print(f"❌ Error in fall conversation: {e}")
 
 
 
    def on_enter_FALL_CONVO(self):
        print("[🩹] Fall Convo started — transitioning to speech recog")
        self.fall_convo_done()  # Continue to chatbot mode
 
def listen_for_wake_word():
    print("🎤 Wake word listener started...")
 
    def on_recognized(evt):
        if evt.result.reason == speechsdk.ResultReason.RecognizedKeyword:
            print(f"\n✅ Wake word '{wake_word}' detected!")
            #play_saved_audio_without_interrupt("Hello! Mister Chen I am here to keep you company.",speaker_audio,volume)
            fsm.wakeword_detected()  # FSM transition
 
    def on_canceled(evt):
        print(f"⚠ Wakeword recognition canceled: {evt.reason}")
 
    while not shutdown_event.is_set():
        if fsm.state != "WAKEWORD":
            time.sleep(0.2)
            continue
 
        try:
            keyword_recognizer = speechsdk.SpeechRecognizer(
                speech_config=speech_config,
                audio_config=AUDIO
            )
 
            keyword_recognizer.recognized.connect(on_recognized)
            keyword_recognizer.canceled.connect(on_canceled)
 
            keyword_recognizer.start_keyword_recognition_async(keyword_model).get()
 
            while fsm.state == "WAKEWORD" and not shutdown_event.is_set():
                time.sleep(0.1)
 
            # On shutdown, make sure recognizer is stopped to avoid dangling thread
            if shutdown_event.is_set():
                try:
                    print("🛑 Shutting down wake word recognizer gracefully...")
                    keyword_recognizer.stop_keyword_recognition_async().get()
                except Exception as e:
                    print(f"⚠ Failed to stop wake word recognizer cleanly: {e}")

 
        except Exception as e:
            print(f"❌ Wake word loop error: {e}")
            time.sleep(1)
 
 
        # Let FSM handle the rest — loop will restart automatically
 
 
def remove_punctuation(text):
    """Removes punctuation from the input text."""
    return text.translate(str.maketrans('', '', string.punctuation))
 
 
 
def handle_new_csv_data(is_new, new_data):
    if is_new:
        for _, row in new_data.iterrows():
            status = row.get("event", "Unknown")
            timestamp = row.get("ts", "Unknown")
            location = row.get("place", "Unknown")
 
            print(f"[✔ CSV ALERT] Status: {status}, Time: {timestamp}, Location: {location}")
            fsm.fall_info_message = (
                f"Fall detected in the {location} at {timestamp}. "
                f"Status: {status}. Please respond calmly and provide instructions."
            )
            fsm.last_transcript = fsm.fall_info_message
            print("[🚨] Triggering fall FSM transition...")
            fsm.fall_detected()
    else:
        print("[📄] No new fall events.")
 
 
## start the State
fsm = ChatbotFSM()
fsm.start()  # goes from IDLE → WAKEWORD
 

# def speech_watchdog():
#     while not shutdown_event.is_set():
#         if fsm.state == "SPEECH_RECOG":
#             time.sleep(30)  # Wait 30 seconds to see if it's stuck
#             if fsm.state == "SPEECH_RECOG":
#                 print("🕒 Watchdog timeout — resetting to WAKEWORD.")
#                 fsm.speech_retry_count = 0
#                 fsm.to_WAKEWORD()


if __name__ == "__main__":
    print("🟢 Starting FSM Chatbot + Fall Detection via CSV")
 
    # # ✅ Initialize ROS node
    # try:
    #     rospy.init_node('fsm_keyword_node', anonymous=True)
    #     print("✅ ROS node initialized.")
    # except rospy.exceptions.ROSException as e:
    #     print(f"❌ Failed to initialize ROS node: {e}")
    #     exit(1)
   
    # subprocess.run(["rosparam", "set", param_name, param_value])
 
 
    # Start CSV watcher
    # threading.Thread(
    #     target=start_watching,
    #     args=(r"/home/nvidia/AI_Results/FALL/fall_results.csv", handle_new_csv_data, shutdown_event),
    #     daemon=True
    # ).start()
 
    # Start wake word listener thread
    threading.Thread(target=listen_for_wake_word, daemon=True).start()
    # # Start watchdog thread
    # threading.Thread(target=speech_watchdog, daemon=True).start()

 
    try:
        while not shutdown_event.is_set():
            time.sleep(1)
        print("👋 Exiting main loop. Cleaning up...")
        shutdown_event.set()
        time.sleep(1)

    except KeyboardInterrupt:
        print("🛑 Ctrl+C detected. Shutting down...")
        shutdown_event.set()
        time.sleep(2)
        
 