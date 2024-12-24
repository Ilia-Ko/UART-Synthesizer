import sys
import serial
from time import time, sleep
import subprocess as proc
import wave
from math import ceil
from tqdm import tqdm

portName = '/dev/ttyUSB0'
baudRate = 1200000
CHUNK = 512

# 0. Get audio file path
if len(sys.argv) != 2:
    print("Please, specify single argument - path to audio file")
    sys.exit()
fPath = sys.argv[1]

# I. Check that file is an audio
res, err = proc.Popen(["file", "-L", fPath], stdout=proc.PIPE, stderr=proc.PIPE, text=True).communicate()
if len(err) > 0:
    print(f"ERROR, cannot touch file '{fPath}'")
    sys.exit()
if res.lower().find("audio") < 0:
    print(f"ERROR, file '{fPath}' does not look like an audio")
    sys.exit()

# II. Convert file into temporary .wav: mono 8-bit @ 48 kHz
res, err = proc.Popen(["ffmpeg", "-hide_banner", "-y", "-v", "error",
        "-i", fPath, "-acodec", "pcm_s16le", "-ar", "48000", "-ac", "1", 'tmp.wav'],
        stdout=proc.PIPE, stderr=proc.PIPE, text=True).communicate()
if len(err) > 0:
    print(f"ERROR, failed to transcode file '{fPath}' into 16-bit mono 48 kHz WAV, error message:\n{err}")
    sys.exit()

# III. Read 8-bit binary samples from 'tmp.wav'
wr = wave.open('tmp.wav', 'rb')
nSamp = wr.getnframes()
fRate = wr.getframerate()
sampWd = wr.getsampwidth()
nChan = wr.getnchannels()
strCh = "mono"
if 2 == nChan:
    strCh = "stereo"
elif nChan > 2:
    strCh = f"{nChan}-channel"
print(f"\nAudio resampled: {strCh} {sampWd*8}-bit {fRate} Hz, bitrate = {nChan*8*sampWd*fRate/1000:.3f} kbit/s")
data = wr.readframes(nSamp)
wr.close()

def ResetBufs(p):
    p.reset_output_buffer()
    p.reset_input_buffer()
def Pings(p):
    for i in range(2):
        p.write(b'ping\r')
        sleep(0.01)
        ResetBufs(p)
# IV. Open serial port
p = serial.Serial(portName, baudRate, timeout=None)
Pings(p)

# V. Enter play mode
p.write(b'play\r')

# VI. Stream data
totChunks = ceil(nSamp * sampWd / CHUNK)
data = data.ljust(totChunks * CHUNK, b'\x00')
totDur = totChunks * CHUNK / sampWd / fRate
nBytes = totChunks * CHUNK
print(f"Original length: {nSamp/fRate:.3f} seconds, chunk-aligned: {totChunks} x {CHUNK} bytes, thus {totDur:.3f} seconds")
portion = bytearray(CHUNK)
ts0 = time()
realBytes = 0
retCode = 0
for ic in tqdm(range(totChunks), unit="kbit", unit_scale=CHUNK*8/1000,
    bar_format='{l_bar}{bar}| [{elapsed} < {remaining}, {rate_fmt}]'):
    # Prepare data chunk
    portion = data[ic*CHUNK:(ic+1)*CHUNK]
    # Wait until this chunk is requested
    recv = p.read(1)
    if b'+' == recv:
        p.write(portion)
        realBytes += CHUNK
    elif b'0' == recv:
        retCode = 1
        break
    else:
        retCode = 2
        break
ts1 = time()
# Finalize audio stream
p.write(b'\x6C\xC5\xB8\x29\x8D\xCA\xED\x1F')    # Two 4-byte stop codes
sleep(0.01)
Pings(p)
p.close()

if 0 == retCode or 1 == retCode:
    if 1 == retCode:
        print("MCU stopped audio streaming")
    elaps = ts1 - ts0
    print(f"Sent {realBytes/1024:.3f} KiB in {elaps:.3f} seconds, bitrate = {realBytes*8/1000/elaps:.3f} kbps\n")
elif 2 == retCode:
    print("ERROR: failed to communicate with MCU")