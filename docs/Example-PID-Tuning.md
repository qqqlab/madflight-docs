# PID Tuning 

## Channel 7 Setup

This channel mixes 4 toggle switches SA-SD to create the following PWM output. Instead of 4 toggle switches, other combinations of switches/toggles/trims can be used to send the PID Tuning PWM commands.

|PWM|Switch|PWM%|
|-|-|-|
1020|SC+SD first variable|-96
1180|SD next variable|-64
1340|SC previous variable|-32
1500|None|0
1660|SA decrease value|32
1820|SB increase value|64
1980|SA+SB reset value|96

EdgeTx mixer setup:

|Source|Weight|Offset|Switch|Multiplex|
|-|-|-|-|-|
MAX|16|16|SA|Replace
SA|16|80|SB|Replace
MAX|-16|-16|SC|Replace
SC|-16|-80|SD|Replace
