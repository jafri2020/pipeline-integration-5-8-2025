from resemblyzer import VoiceEncoder, preprocess_wav
from pathlib import Path
import sounddevice as sd
from scipy.io.wavfile import write
from sklearn.metrics.pairwise import cosine_similarity
import numpy as np
from record_audio import record #self built function
import pickle



# Initialize encoder
encoder = VoiceEncoder()

# Speaker reference files
speakers = {
    "farhan1": "new_recordings/farhan1.wav",
    "farhan2": "new_recordings/farhan2.wav",
    "farhan3": "new_recordings/farhan3.wav",
    "farhan4": "new_recordings/farhan4.wav",
    # "farhan5": "new_recordings/farhan5.wav",
    "farhan6": "new_recordings/farhan6.wav",
    "farhan7": "new_recordings/farhan7.wav",
    "farhan8": "new_recordings/farhan8.wav",
    "farhan9": "new_recordings/farhan9.wav",
    "farhan10": "new_recordings/farhan10.wav",
    
    "kabeer1": "new_recordings/kabeer1.wav",
    "kabeer2": "new_recordings/kabeer2.wav",
    "kabeer3": "new_recordings/kabeer3.wav",
    "kabeer4": "new_recordings/kabeer4.wav",
    "kabeer5": "new_recordings/kabeer5.wav",
    "kabeer6": "new_recordings/kabeer6.wav",
    "kabeer7": "new_recordings/kabeer7.wav",
    "kabeer8": "new_recordings/kabeer8.wav",
    "kabeer9": "new_recordings/kabeer9.wav",
    "kabeer10": "new_recordings/kabeer10.wav",
    "kabeer11": "new_recordings/kabeer11.wav",
    
    "hasnain1": "new_recordings/hasnain1.wav",
    "hasnain2": "new_recordings/hasnain2.wav",
    "hasnain3": "new_recordings/hasnain3.wav",
    "hasnain4": "new_recordings/hasnain4.wav",
    "hasnain5": "new_recordings/hasnain5.wav",
    "hasnain6": "new_recordings/hasnain6.wav",
    "hasnain7": "new_recordings/hasnain7.wav",
    "hasnain8": "new_recordings/hasnain8.wav",
    "hasnain9": "new_recordings/hasnain9.wav",
    "hasnain10": "new_recordings/hasnain10.wav",
    "hasnain11": "new_recordings/hasnain11.wav",
    "hasnain12": "new_recordings/hasnain12.wav",
    "hasnain13": "new_recordings/hasnain13.wav",
    "hasnain14": "new_recordings/hasnain14.wav",
    "hasnain15": "new_recordings/hasnain15.wav", 
    "shahzaib1": "new_recordings/shahzaib1.wav",
    "shahzaib2": "new_recordings/shahzaib2.wav",
    "shahzaib3": "new_recordings/shahzaib3.wav",
    "shahzaib4": "new_recordings/shahzaib4.wav",
    "shahzaib5": "new_recordings/shahzaib5.wav",
    "shahzaib6": "new_recordings/shahzaib6.wav",
    "shahzaib7": "new_recordings/shahzaib7.wav",
    "shahzaib8": "new_recordings/shahzaib8.wav",
    "shahzaib9": "new_recordings/shahzaib9.wav",
    "shahzaib10": "new_recordings/shahzaib10.wav",
    "shaiza1": "new_recordings/shaiza1.wav",
    "shaiza2": "new_recordings/shaiza2.wav",
    "shaiza3": "new_recordings/shaiza3.wav",
    "shaiza4": "new_recordings/shaiza4.wav",
    "shaiza5": "new_recordings/shaiza5.wav",
    "shaiza6": "new_recordings/shaiza6.wav",
    "shaiza7": "new_recordings/shaiza7.wav",
    "shaiza8": "new_recordings/shaiza8.wav",
    "shaiza9": "new_recordings/shaiza9.wav",
    "shaiza10": "new_recordings/shaiza10.wav",
    "sheharbano1": "new_recordings/sheharbano1.wav",
    "sheharbano2": "new_recordings/sheharbano2.wav",
    "sheharbano3": "new_recordings/sheharbano3.wav",
    "sheharbano4": "new_recordings/sheharbano4.wav",
    "sheharbano5": "new_recordings/sheharbano5.wav",
    "sheharbano6": "new_recordings/sheharbano6.wav",
    "sheharbano7": "new_recordings/sheharbano7.wav",
    "sheharbano8": "new_recordings/sheharbano8.wav",
    "sheharbano9": "new_recordings/sheharbano9.wav",
    "sheharbano10": "new_recordings/sheharbano10.wav",
    "aazma1": "new_recordings/aazma1.wav",
    "aazma2": "new_recordings/aazma2.wav",
    "aazma3": "new_recordings/aazma3.wav",
    "aazma4": "new_recordings/aazma4.wav",
    "aazma5": "new_recordings/aazma5.wav",
    "aazma6": "new_recordings/aazma6.wav",
    "aazma7": "new_recordings/aazma7.wav",
    "aazma8": "new_recordings/aazma8.wav",
    "aazma9": "new_recordings/aazma9.wav",
    "aazma10": "new_recordings/aazma10.wav"

}



# Compute embeddings for known speakers
known_embeddings = {}
for name, file in speakers.items():
    wav = preprocess_wav(Path(file))
    emb = encoder.embed_utterance(wav)
    known_embeddings[name] = emb


# Save the embeddings
with open("known_embeddings.pkl", "wb") as f:
    pickle.dump(known_embeddings, f)