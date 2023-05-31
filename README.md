# Mastermind Game On RaspberryPi
This is a coursework project aimed at developing a simple, systems-level application in C and ARM Assembler for the Raspberry Pi. The objective is to create a MasterMind board game implementation, focusing on the interaction between embedded hardware and external devices. The project requires a detailed understanding of low-level code, resource management, and time-sensitive operations.

# Lab Environment
Hardware environment: Raspberry Pi 2 or 3 with the starter kit.

Software environment:SD card 16Gb

# Embedded Systems Programming: MasterMind Application
Developing the MasterMind board game using C and ARM Assembly as the implementation language. The game will run on a Raspberry Pi 2 or 3 with the following attached devices: two LEDs, a button, and an LCD with a potentiometer for contrast control.

# Application Description
MasterMind is a two-player game between a codekeeper and a codebreaker. The codekeeper selects a sequence of colored pegs, and the codebreaker tries to guess the sequence. The codekeeper provides feedback on each guess, indicating the number of pegs that are the right color and in the right position, and the number of pegs that are the right color but in the wrong position. The game continues until the codebreaker successfully guesses the code or reaches a fixed number of turns.
