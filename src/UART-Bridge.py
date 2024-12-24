import serial
import readline
import os
import sys
from time import time, sleep
import subprocess as proc
import wave
from math import ceil
from tqdm import tqdm
from multiprocessing import RawArray, Pool, Lock

class MyCompleter(object):  # Custom completer

    def __init__(self, options, scales, tones, octaves):
        self.options = sorted(options)
        self.scales = sorted(scales)
        self.tones = sorted(tones)
        self.octaves = sorted(octaves)
        self.areFiles = False
        self.basePathLen = 0

    def complete(self, text, state):
        if state == 0:  # on first trigger, build possible matches
            self.areFiles = False
            if not text:
                self.matches = self.options[:]
            else:
                if text.startswith('play '):
                    strPath = text[5:]
                    if len(strPath) > 1:
                        if '"' == strPath[0]:
                            strPath = strPath[1:]
                    curDir = "/"
                    strItem = ""
                    if len(strPath) > 1:
                        if '"' == strPath[-1]:
                            strPath = strPath[:-1]
                        if len(strPath) > 0:
                            pos = strPath.rfind('/')
                            if pos >= 0:
                                curDir = strPath[:pos+1]
                                if pos+1 < len(strPath):
                                    strItem = strPath[pos+1:]
                    # List of elements in current directory
                    if os.path.isfile(strPath):
                        self.matches = ['play "'+strPath+'"']
                    else:
                        self.matches = []
                        if len(curDir) > 1:
                            if '/' != curDir[-1]:
                                curDir += '/'
                        elems = os.listdir(curDir)
                        for e in elems:
                            if len(strItem) == 0 or e.startswith(strItem):
                                path = curDir + e
                                if os.path.isdir(path):
                                    path += '/'
                                else:
                                    path += '"'
                                self.matches.append('play "'+path)
                        if len(self.matches) > 0:
                            self.areFiles = True
                            self.basePathLen = 6+len(curDir)
                        else:
                            self.matches = [None]
                else:
                    pos = text.rfind('/')
                    if pos > 0:
                        pre = text[:pos]
                        post = text[pos:]
                        self.matches = [pre+s for s in self.scales
                                    if s and s.startswith(post)]
                    elif text.startswith("impr scale ") or text.startswith("fire "):
                        pre = "fire "
                        post = text[5:]
                        app = " "
                        if text.startswith("impr scale "):
                            pre = "impr scale "
                            post = text[11:]
                            app = "/"
                        if len(post) == 0:
                            self.matches = [pre+s for s in self.tones]
                            if "fire " == pre:
                                self.matches += [pre+"noise"]
                        else:
                            if post[0] in self.tones:
                                self.matches = [pre+post[0]+s+app for s in self.octaves
                                            if s and s.startswith(post[1:])]
                                if len(post) == 1:
                                    self.matches += [pre+post[0]+app]
                            elif "fire " == pre:
                                if "noise".startswith(post[:min(len(post),5)]):
                                    self.matches = [pre+"noise "+ns+" " for ns in noises if ("noise "+ns).startswith(post)]
                                else:
                                    self.matches = [None]
                    else:
                        self.matches = [s for s in self.options
                                    if s and s.startswith(text)]

        # return match indexed by state
        try:
            return self.matches[state]
        except IndexError:
            return None

    def display_matches(self, substitution, matches, longest_match_length):
        line_buffer = readline.get_line_buffer()
        columns = environ.get("COLUMNS", 80)

        print()

        longest = max(map(len, matches))
        if self.areFiles:
            longest -= self.basePathLen
        # tpl = "{:<" + str(int(max(map(len, matches)) * 1.2)) + "}"
        tpl = "{:<" + str(int(longest * 1.2)) + "}"

        buffer = ""
        for match in matches:
            if self.areFiles:
                match = match[self.basePathLen:]
            match = tpl.format(match[len(substitution):])
            if len(buffer + match) > columns:
                print(buffer)
                buffer = ""
            buffer += match

        if buffer:
            print(buffer)

        print(" PC: "+line_buffer, end="")
        sys.stdout.flush()

commands = [
    'fire ',
    'play "', 'status', 'resbuf', 'ping', 'stop', 'help', 'exit', 'quit',
    'impr chaos', 'impr board', 'impr scale ', 'impr ladder',
    'set beat ', 'set filter ', 'set filter on', 'set filter off', 'set equalizer ',
    'set pattern ', 'set pattern stable', 'set pattern mutability ',
    'set entropy RNG', 'set entropy ADC',
    'set wave sin', 'set wave sin3', 'set wave sin5',
    'set wave rect', 'set wave saw', 'set wave 3ang',
    'get beat', 'get filter', 'get pattern', 'get entropy', 'get wave', 'get equalizer'
]
scales = [
    '/chrom', '/maj', '/nat.min', '/mel.min', '/har.min', '/pent.maj', '/pent.min', '/japanese',
    '/lydian', '/mixolydian', '/dorian', '/phrygian', '/locrian', '/blues', '/blues-7', '/blues-9'
]
tones = [
    'A', 'B', 'C', 'D', 'E', 'F', 'G',
    'a', 'b', 'c', 'd', 'e', 'f', 'g'
]
octaves = [
    '-4', '-3', '-2', '-1', '-0', '+0', '+1', '+2', '+3', '+4', '+5'
]
noises = [
    'white', 'pink', 'red', 'blue', 'violet', 'uv'
]
tonocts = [t+o for t in tones for o in octaves]
compl = MyCompleter(commands, scales, tones, octaves)
readline.set_completer_delims('\n')
readline.set_completer(compl.complete)
readline.parse_and_bind("tab: complete")
# readline.set_completion_display_matches_hook(compl.display_matches)
if os.path.isfile("./bridge.history"):
    readline.read_history_file("./bridge.history")
readline.set_history_length(1000)

portName = '/dev/ttyUSB0'
if not os.path.exists(portName):
    portName = '/dev/ttyUSB1'
if not os.path.exists(portName):
    portName = input("Please, enter valid port name: ")
print("Using port '"+portName+"'")
baudRate = 2000*1000
CHUNK = 128*3
miniDelay = 0.01
N_PROC = 10     # Count of processes for parallel compression 16 bit -> 12 bit

# Routine that reads and shows a line from MCU if available
def TryLine(p):
    if p.in_waiting > 0:
        recv = p.read(p.in_waiting)
        msg = ""
        try:
            msg = recv.decode("utf-8")
        except:
            msg = recv
        print(f"MCU: {msg}")

# Reset UART buffers on PC side
def ResetBufs(p):
    p.reset_output_buffer()
    p.reset_input_buffer()

# Double ping-pong with MCU
def PingPong(p):
    for i in range(2):
        p.write(b'ping\r')
        sleep(miniDelay/2)
        p.read(p.in_waiting)

# Compress array 'arrSrc' of 16-bit samples into an array 'arrDst' of 12-bit ones
arrSrc = None
arrDst = None
def Compress12bit(args):
    iBeg = args[0]
    nSamp = args[1]
    showExample = args[2]
    if showExample:
        print(f"   -- --  -- --     -- -- --")
        print(f"   R1 R0, L1 L0 --> R1 RL L0")
        print(f"   -- --  -- --     -- -- --")
        checkPoint = iBeg + nSamp // 2
    for i in range(iBeg, iBeg+nSamp):
        sL = arrSrc[4*i+0:4*i+2]
        sR = arrSrc[4*i+2:4*i+4]
        arrDst[3*i+0] = (15 & (sL[0] >> 4)) | (240 & (sL[1] << 4))
        arrDst[3*i+1] = (15 & (sL[1] >> 4)) | (240 & sR[0])
        arrDst[3*i+2] = (255 & sR[1])
        if showExample:
            if i >= checkPoint and i < checkPoint+6:
                d0 = arrDst[3*i+0]
                d1 = arrDst[3*i+1]
                d2 = arrDst[3*i+2]
                print(f"   {sR[1]:02x} {sR[0]:02x}, {sL[1]:02x} {sL[0]:02x} --> {d2:02x} {d1:02x} {d0:02x}")
    if showExample:
        print(f"   -- --  -- --     -- -- --")

# Playing audio file
def PlayAudio(p, fPath):
    # I. Check that given path is an audio file
    if not os.path.isfile(fPath):
        print(f"  ERROR, path '{fPath}' is not a file")
        return
    res, err = proc.Popen(["file", "-L", fPath], stdout=proc.PIPE, stderr=proc.PIPE, text=True).communicate()
    if len(err) > 0:
        print(f"  ERROR, cannot access file '{fPath}'")
        return
    if res.lower().find("audio") < 0:
        print(f"  ERROR, file '{fPath}' does not look like an audio")
        return

    # II. Convert file into temporary .wav: mono 8-bit @ 48 kHz
    t0 = time()
    res, err = proc.Popen(["ffmpeg", "-hide_banner", "-y", "-v", "error",
            "-i", fPath, "-acodec", "pcm_s16le", "-ar", "48000", "-ac", "2", 'tmp.wav'],
            stdout=proc.PIPE, stderr=proc.PIPE, text=True).communicate()
    if len(err) > 0:
        print(f"  ERROR, failed to transcode file '{fPath}' into 16-bit stereo 48 kHz WAV, error message:\n{err}")
        return
    t0 = time() - t0

    # III. Read 16-bit binary samples from 'tmp.wav'
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
    print(f"  Audio resampled in {t0:.2f} sec: {strCh} {sampWd*8}-bit {fRate} Hz, bitrate = {nChan*8*sampWd*fRate/1000:.3f} kbit/s")
    data1 = wr.readframes(nSamp)
    wr.close()

    # IIIa. Convert 16-bit stereo frames into 12-bit stereo 24-bit packed frames
    global arrSrc, arrDst
    arrSrc = RawArray('i', data1)
    arrDst = RawArray('i', 3*nSamp)
    # Prepare for parallel compression
    myTasks = []
    for i in range(N_PROC):
        nWork = nSamp // N_PROC
        iBeg = nSamp - (N_PROC-i) * nWork
        if i < nSamp % N_PROC:
            nWork += 1
            iBeg = i * nWork
        myTasks.append((iBeg, nWork, N_PROC//2==i))
    # Do parallel compression
    t0 = time()
    pool = Pool(N_PROC)
    pool.map(Compress12bit, myTasks)
    data = bytearray(3*nSamp)
    data[:] = arrDst[:]
    # data = bytearray(arrDst)
    t0 = time() - t0
    sampWd = 1.5    # Effective sample width in bytes (12 bits = 1.5 bytes)
    print(f"  Audio compressed in {t0:.2f} sec: {strCh} {ceil(sampWd*8)}-bit {fRate} Hz, bitrate = {nChan*8*sampWd*fRate/1000:.3f} kbit/s")

    # IV. Check serial port
    PingPong(p)
    p.timeout = 1.0 # Larger timeout

    # V. Prepare for data streaming
    totChunks = ceil(nChan * nSamp * sampWd / CHUNK)
    data = data.ljust(totChunks * CHUNK, b'\x00')
    totDur = totChunks * CHUNK / sampWd / nChan / fRate
    nBytes = totChunks * CHUNK
    print(f"  Original length: {nSamp/fRate:.3f} seconds, chunk-aligned: {totChunks} x {CHUNK} bytes, thus {totDur:.3f} seconds")
    portion = bytearray(CHUNK)
    realBytes = 0
    retCode = 0

    # VI. Enter play mode and stream data
    print("\n  Please, press USER BUTTON to stop audio streaming.")
    p.write(b'play\r')
    ts0 = time()
    for ic in tqdm(range(totChunks), unit="kbit", unit_scale=CHUNK*8/1000,
        bar_format='{l_bar}{bar}| [{elapsed} < {remaining}, {rate_fmt}]'):
        # Prepare data chunk
        portion = data[ic*CHUNK:(ic+1)*CHUNK]
        # portion = ('\x00'*CHUNK).encode()
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
    # Finalize audio stream
    p.write(b'\x6C\xC5\xB8\x29\x8D\xCA\xED\x1F')    # Two 4-byte stop codes
    ts1 = time()

    # Report results
    if 0 == retCode or 1 == retCode:
        if 1 == retCode:
            print("  Synthesizer has stopped audio streaming")
        elaps = ts1 - ts0
        print(f"  Sent {realBytes/1024:.3f} KiB in {elaps:.3f} seconds, bitrate = {realBytes*8/1000/elaps:.3f} kbps\n")
    elif 2 == retCode:
        print("  ERROR: failed to communicate with synthesizer")

    # VII. Restore UART port configuration
    p.timeout = 0
    sleep(miniDelay)
    PingPong(p)



# Open serial port
p = serial.Serial(portName, baudRate, timeout=0)
ResetBufs(p)
PingPong(p)



# Main loop for parsing user commands
while True:
    # Send command to PC
    cmd = input(" PC: ")
    if len(cmd) == 0:
        TryLine(p)  # Empty command is a request to read data
    elif "exit" == cmd or "quit" == cmd:
        # Exit commands
        break
    elif "help" == cmd:
        # Print help
        print()
        print("Command line interface reference.")
        print("1. Playing:")
        print("   fire <freq,Hz> <dur,ms>       / plays frequency <freq,Hz> for <dur,ms> milliseconds")
        print("   fire <tone> <dur,ms>          / plays tone <tone> for <dur,ms> milliseconds, see <tone> spec. below")
        print("      <tone> has format <key><octave> where <key> is from [A-G|a-g] and <octave> ranges from -4 to +5")
        print("      Examples of <tone> are:  F  G+1  d  B-2")
        print("   fire noise <type> <dur,ms>    / plays <type> noise for <dur,ms> milliseconds")
        print("      <type> is one of:  red  pink  white  blue  violet  uv.  White noise can be IIR-filtered additionally.")
        print("   impr chaos [<dur,ms>]         / plays randomly chosen frequencies from 110 to 880 Hz and")
        print("                                   optionally sets beat = <dur,ms>")
        print("   impr board [<dur,ms>]         / plays randomly chosen keyboard tones from 110 to 880 Hz and")
        print("                                   optionally sets beat = <dur,ms>")
        print("   impr ladder [<dur,ms>]        / plays ascending/descending keyboard tones from 110 to 880 Hz and")
        print("                                   optionally sets beat = <dur,ms>")
        print("   impr scale <tone>/<scale>     / plays randomly chosen tones from <scale> allocated around <tone> and")
        print("                                   optionally sets beat = <dur,ms>")
        print("      <scale> is one of:  maj  nat.min  har.min  mel.min  chrom  pent.maj  pent.min  japanese  lydian")
        print("                          mixolydian  dorian  phrygian  locrian  blues  blues-7  blues-9")
        print("   play <audiofile>              / stream <audiofile> via the synthesizer")
        print("   stop                          / stops anything being played")
        print()
        print("2. Playback settings:")
        print("   set wave <wav> [<amp>]        / specify waveform <wav> to be synthesized with amplitude <amp>")
        print("                                   <wav> is one of:  sin  sin3  sin5  rect  saw  3ang")
        print("                                   optional amplitude <amp> is a positive number")
        print("                                   NB! Amplitude < 1 causes degradation of resolution")
        print("                                   NB! Amplitude > 1 causes signal clamping")
        print("   get wave                      / request current waveform")
        print("   set beat <dur,ms>             / specify beat duration in milliseconds")
        print("   get beat                      / request current beat duration")
        print("   set pattern <patt>            / specify rythmic pattern for improvisations, see <patt> spec. below")
        print("      <patt> is a sequence of up to 16 rational positive numbers")
        print("      Example of <patt> is:  2 1 3/2 1/2")
        print("   set pattern stable            / prevents pattern mutation")
        print("   set pattern mutability <mut>  / specify chance <mut> of pattern mutation, on every pattern repetition")
        print("                                   a mutation takes place with probability from [0,1] compared to <mut>")
        print("   get pattern                   / request current pattern settings")
        print("   set entropy <src>             / specify source of randomness, <src> is one of:  RNG  ADC")
        print("   get entropy                   / request current source of randomness")
        print("   set filter <cfs_B> / <cfs_A>  / specify up to 6/6 coefficients for IIR filter (Infinite-Impulse Response).")
        print("                                   The IIR filter can be applied to streamed audio or noise samples as well")
        print("                                   as an IIR with non-sequential lags. When processing audio or noise samples,")
        print("                                   the long-range memory allows for filtering specific frequencies as")
        print("                                   defined by 'set equalizer <freqs,Hz>' command (see below)")
        print("   set filter on                 / enable IIR filtering of played sequence")
        print("   set filter off                / disable IIR filtering of played sequence")
        print("   get filter                    / request current filter settings")
        print("   set equalizer <freqs,Hz>      / specify up to 8 <freqs,Hz> for IIR filter-equalizer. Note that")
        print("                                   each frequency will be rounded to the nearest value as [48 kHz / n]")
        print("   get equalizer                 / request current equalizer frequencies")
        print()
        print("3. General commands:")
        print("   status                        / request current status of synthesizer")
        print("   resbuf                        / reset input/output UART buffers on PC side")
        print("   ping                          / check connection with synthesizer")
        print("   help                          / display this help")
        print("   exit                          / finish this program")
        print("   quit                          / finish this program")
        print()
        print("4. Some examples:")
        print("   fire A-1 6000                 / Play 220 Hz tone (A-1) for 6 seconds")
        print("   fire 3.14159e2 50             / Play 314.159 Hz frequency for 0.05 seconds")
        print("   impr scale F/maj 250          / Improvise in F-major with beat of 0.25 seconds")
        print("   set pattern 1 1 1/2 3/2       / Set beat pattern 4/4 with syncopation")
        print("   set pattern 1 3/7 2/5         / Set non-standard beat pattern 31/35")
        print("   set filter 5 0 -1.1 3         / Specify moving-average filter with 4-order polynomial")
        print()
        print("Please, use TAB completions in this shell")
        print()
    elif "resbuf" == cmd:
        # Reset UART buffers
        ResetBufs(p)
    elif cmd.startswith("play "):
        # Stream audio file
        fPath = cmd[5:]
        if len(fPath) > 2:
            if '"' == fPath[0]:
                fPath = fPath[1:]
            if '"' == fPath[-1]:
                fPath = fPath[:-1]
            PlayAudio(p, fPath)
    else:
        # Send command
        cmd += '\r'
        if len(cmd) % 2 != 0:   # Ensure that command contains even count of bytes
            cmd += '\n'         # This saves fragile audio streams that might go through UART as well
        rawCmd = cmd.encode('utf-8')
        p.write(rawCmd)
        p.flush()
        sleep(miniDelay)
        # Try reading MCU response
        TryLine(p)

# Finalize
sleep(miniDelay)
TryLine(p)  # Final check for MCU output
ResetBufs(p)
p.close()
readline.write_history_file("./bridge.history")