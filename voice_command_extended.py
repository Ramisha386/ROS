import socket
import time
import minimalmodbus
import serial
import struct
import sounddevice as sd
import numpy as np
import whisper
import difflib
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)

# === CONFIGURATION ===
host = '192.168.10.190'
port = 30001
gripper_port = '/dev/ttyUSB0'

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
        print("\n🎤 Listening for command...")
        audio = sd.rec(int(samplerate * duration), samplerate=samplerate, channels=1, dtype='float32')
        sd.wait()
        return np.squeeze(audio)
    except Exception as e:
        print("❌ Recording error:", e)
        return None

def transcribe_audio(audio):
    try:
        print("🧠 Transcribing...")
        result = model.transcribe(audio, fp16=False)
        return result["text"].strip().lower()
    except Exception as e:
        print("❌ Transcription error:", e)
        return None

# === NATURAL LANGUAGE INTENT PARSING ===
def get_intent(text):
    text = text.lower()

    if any(kw in text for kw in ["pick", "grab", "take", "lift"]):
        return "pick"
    elif any(kw in text for kw in ["place", "put", "drop", "bin"]):
        return "place"
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

# === COMMUNICATION ===
def send_script(script):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        sock.send((script + "\n").encode())
        sock.close()
        print("✅ Script sent to robot.")
    except Exception as e:
        print("❌ Script send error:", e)

# === GRIPPER SETUP ===
def setup_gripper():
    try:
        gripper = minimalmodbus.Instrument(gripper_port, 9)
        gripper.serial.baudrate = 115200
        gripper.serial.bytesize = 8
        gripper.serial.parity = serial.PARITY_NONE
        gripper.serial.stopbits = 1
        gripper.serial.timeout = 1
        gripper.write_register(0x03E8, 0x0001, 0)  # Activate
        return gripper
    except Exception as e:
        print("❌ Gripper setup error:", e)
        exit(1)

def close_gripper(gripper):
    gripper.write_register(0x03E8, 0x0900, 0)
    gripper.write_register(0x03E9, 0x00FF, 0)
    gripper.write_register(0x03EA, 0xFFFF, 0)
    time.sleep(2)

def open_gripper(gripper):
    gripper.write_register(0x03E8, 0x0900, 0)
    gripper.write_register(0x03E9, 0x0000, 0)
    gripper.write_register(0x03EA, 0xFFFF, 0)
    time.sleep(2)

# === ROBOT ACTIONS ===
def pick(gripper):
    print("📦 Performing PICK...")
    script_full = """def test_move():
    global P_start_p=p[0.3206, -0.15, 0.2809, 2.1919, -2.1463, -0.0555]
    global P_second_p=p[0.3206, -0.15, 0.2209, 2.1919, -2.1463, -0.0555]
    global P_third_p=p[0.3206, -0.15, 0.0525, 2.1919, -2.1463, -0.0555]
    movel(P_start_p, a=0.7, v=0.125)
    stopj(1)
    movel(P_second_p, a=0.7, v=0.125)
    stopj(1)
    movel(P_third_p, a=0.7, v=0.125)
    stopj(1)
end

test_move()"""
    send_script(script_full)
    time.sleep(5)
    close_gripper(gripper)

def place(gripper):
    print("📤 Performing PLACE...")
    script_full = """def place_move():
    global P1=p[0.3206, -0.15, 0.2209, 2.1919, -2.1463, -0.0555]
    global P2=p[0.3206, -0.15, 0.2809, 2.1919, -2.1463, -0.0555]
    global P3=p[0.3206, 0.2, 0.2809, 2.1919, -2.1463, -0.0555]
    movel(P1, a=0.7, v=0.125)
    stopj(1)
    movel(P2, a=0.7, v=0.125)
    stopj(1)
    movel(P3, a=0.7, v=0.125)
    stopj(1)
end

place_move()"""
    send_script(script_full)
    time.sleep(5)
    open_gripper(gripper)

def reset_position():
    print("🏠 Returning to initial home position...")
    script = """def go_home():
    global P_start_p=p[0.3206, -0.15, 0.2809, 2.1919, -2.1463, -0.0555]
    movel(P_start_p, a=0.7, v=0.125)
    stopj(1)
end

go_home()"""
    send_script(script)
    time.sleep(4)

# === MAIN LOOP ===
if __name__ == "__main__":
    print("🔧 Initializing gripper...")
    gripper = setup_gripper()
    print("🤖 Voice control ready. Say things like:")
    print(" - 'Pick the object'")
    print(" - 'Place it in the bin'")
    print(" - 'Go back to your initial position'")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            audio = record_audio()
            if audio is None:
                continue

            transcript = transcribe_audio(audio)
            if transcript is None:
                continue

            print(f"🗣️ You said: '{transcript}'")
            command = get_intent(transcript)

            if command == "pick":
                pick(gripper)
            elif command == "place":
                place(gripper)
            elif command == "reset":
                reset_position()
            else:
                print("⚠️ Command not recognized. Try using 'pick', 'place', or 'reset' keywords.")

    except KeyboardInterrupt:
        print("\n🛑 Voice control stopped.")

