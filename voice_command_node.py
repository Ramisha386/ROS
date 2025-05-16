#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np

model = whisper.load_model("base")

def record_and_transcribe(duration=5, samplerate=16000):
    rospy.loginfo("🎤 Listening for command...")
    audio = sd.rec(int(samplerate * duration), samplerate=samplerate, channels=1, dtype='float32')
    sd.wait()
    result = model.transcribe(np.squeeze(audio), fp16=False)
    return result["text"].strip().lower()

def voice_node():
    pub = rospy.Publisher('/voice_command', String, queue_size=10)
    rospy.init_node('voice_command_node', anonymous=True)
    rate = rospy.Rate(0.2)  # One command every 5 seconds

    while not rospy.is_shutdown():
        try:
            transcript = record_and_transcribe()
            rospy.loginfo(f"Heard: {transcript}")
            pub.publish(transcript)
        except Exception as e:
            rospy.logwarn(f"Error: {e}")
        rate.sleep()

if __name__ == '__main__':
    voice_node()
