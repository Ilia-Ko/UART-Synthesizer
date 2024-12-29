This branch `main-f429` is dedicated to STM32F429ZI Disco board. Please, find branch `main-h743` for STM32H743VIT6 board.

# UART-Synthesizer on STM32F429ZI
A firmware for STM32F429ZI Discovery board to be an UART audio synthesizer and UART audio player.

## Hardware components
You will need the following hardware components to launch this synthesizer:
* STM32F429ZI evaluation board 'Discovery'
* Personal computer (for Python frontend utility and audio processing with FFmpeg)
* USB-to-UART adapter with support for 1.2 MHz baudrate
* USB Micro cable 
* Audio speakers
* Spare cable for audio speakers with Left, Right and GND wires
* 200-500 Ohm trimmer and breadboard to plug it at (for low-impedance speakers only)

## Software components
* STM32 CubeF4 distribution v1.12.1 https://github.com/STMicroelectronics/STM32CubeF4
* STlink utilities v1.8.0 https://www.st.com/en/development-tools/st-link-v2.html
* arm-none-eabi-gcc v14.2.0 https://gcc.gnu.org/
* make v4.4.1 https://www.gnu.org/software/make/
* (optional) Visual Studio Code or similar IDE
* (optional) Cortex-Debug plugin for VSCode
* python v3.12.7 https://www.python.org/
    * Modules: os, sys, time, math, readline, wave, subprocess, multiprocessing, pyserial (as `serial`), tqdm
* pyserial v3.4 https://github.com/pyserial/pyserial
* tqdm v4.67.1 https://github.com/tqdm/tqdm
* FFmpeg v7.1 https://ffmpeg.org/

## Assembling
1. Connect USB-to-UART adapter to the 'F429. RX (adapter) -- pin PB6, TX (adapter) -- pin PB7
2. Connect 'F429 pins to audio speakers: PA5 -- Left and/or Right (Mono), GND -- GND. In case of low-impedance speakers insert 200-500 Ohm trimmer inbetween (PA5 -- 200 Ohm -- Left and/or Right, GND -- GND)
3. Connect 'F429 with your PC via USB Micro cable
4. Plug USB-to-UART adapter into your PC

## Running
1. Update `Makefile` with your `CubeF4` installation paths (as well as `.vscode/*.json` files for proper syntax highlighting when using VSCode)
2. Run `make` in the project's root directory to compile the firmware
3. Flash the firmware to 'F429 with `st-flash` utility or VSCode's graphical interface
4. Reset 'F429 by pressing reset button
5. Launch Python frontend with `python src/UART-Bridge-F429.py`
6. Enter `help` to view available commands and their descriptions
7. Use tab-completions when typing commands
8. Type `exit` or `quit` to terminate Python frontend

## Further details

Additional information can be found in file 'Презентация.pdf' (not translated yet...)