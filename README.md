# PulsePatch
Code for PulsePatches

Interfacing the MAX30102 sensor. This code targets a Simblee radio.
Outputs a data stream via Serial port or BLE
go here
        https://github.com/biomurph/PulsePatch-js
for code to receive BLE data

Getting-Started Docs for Simblee
        https://www.simblee.com/Simblee_Quickstart_Guide_v1.0.pdf

On-board heart rate and SpO2 calculated by Maxim algorithms (algorithm.cpp and algorithm.h).  Depending on the architecture, this should be done with one of two versions of the Maxim algorithms:

* The Arduino version (github repo: https://github.com/MaximIntegratedRefDesTeam/RD117_ARDUINO/)

* The mbed version (github repo: https://github.com/MaximIntegratedRefDesTeam/RD117_FLORA_BLE).  
	* Make sure to change `include "mbed.h"` to `include <arduino.h>` in the algorithm.h and algorithm.cpp files. 