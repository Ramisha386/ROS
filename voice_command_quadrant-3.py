#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import whisper
import difflib
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)

# === WHISPER SETUP ===
print("🧠 Loading Whisper model...")
try:
    model = whisper.load_model("base")
    print("✅ Whisper model loaded.")
except Exception as e:
    print("❌ Failed to load Whisper:", e)
    exit(1)

# === AUDIO UTILITIES ===
def record_audio(duration=5, samplerate=16000):
    try:
        rospy.loginfo("🎤 Listening for command...")
        audio = sd.rec(int(samplerate * duration), samplerate=samplerate, channels=1, dtype='float32')
        sd.wait()
        return np.squeeze(audio)
    except Exception as e:
        rospy.logwarn(f"❌ Recording error: {e}")
        return None

def transcribe_audio(audio):
    try:
        rospy.loginfo("🧠 Transcribing...")
        result = model.transcribe(audio, fp16=False)
        return result["text"].strip().lower()
    except Exception as e:
        rospy.logwarn(f"❌ Transcription error: {e}")
        return None

# === NATURAL LANGUAGE INTENT PARSING ===
def get_intent(text):
    text = text.lower()
    if any(kw in text for kw in ["pick", "grab", "take", "lift"]):
        return "pick"
    elif any(kw in text for kw in ["one q","one quadrant","first quadrant"]):
        return "place1q"
    elif any(kw in text for kw in ["two q","two quadrant","second quadrant"]):
        return "place2q"
    elif any(kw in text for kw in ["three q","three quadrant","third quadrant"]):
        return "place3q"
    elif any(kw in text for kw in ["four q","four quadrant","forth quadrant","4q","4 cue","four cue"]):
        return "place4q"
    elif any(kw in text for kw in ["reset", "home", "initial", "start", "back"]):
        return "reset"

    # Fallback fuzzy match
    choices = ["pick", "place", "reset"]
    match = difflib.get_close_matches(text, choices, n=1, cutoff=0.6)
    if match:
        return match[0]

    for word in text.split():
        match = difflib.get_close_matches(word, choices, n=1, cutoff=0.8)
        if match:
            return match[0]

    return None

# === ROS VOICE NODE ===
def voice_command_quadrant_node():
    rospy.init_node('voice_command_quadrant_node', anonymous=True)
    pub = rospy.Publisher('/voice_command', String, queue_size=10)
    rate = rospy.Rate(0.2)  # one command every 5 seconds

    rospy.loginfo("🤖 Voice control ready. Say commands like 'pick', 'place in quadrant', or 'reset'...")

    while not rospy.is_shutdown():
        audio = record_audio()
        if audio is None:
            continue

        transcript = transcribe_audio(audio)
        if transcript is None:
            continue

        rospy.loginfo(f"🗣️ You said: '{transcript}'")
        intent = get_intent(transcript)
        if intent:
            pub.publish(intent)
            rospy.loginfo(f"📤 Published intent: {intent}")
        else:
            rospy.logwarn("⚠️ Command not recognized.")

        rate.sleep()

if __name__ == "__main__":
    try:
        voice_command_quadrant_node()
    except rospy.ROSInterruptException:
        pass
