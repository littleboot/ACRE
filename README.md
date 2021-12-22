# ACRE
Absolute Capacitive Rotary Encoder (ACRE) repository.
This repository contains all design files used to create the ACRE.

## Introduction
This readme describes the design and build of a Absolute Capacitive Rotary Encoder (ACRE) that consists out of two simple 2-layer pcb's and a handful of components.
It can detect and measure the absolute angle of an axis from 0 to 360 degrees. The design is based uppon the working principle of a digital caliper, currently it has an accuracy of approximately 1 degrees. 
Both hardware and software could be improved if you have any suggestions please let me know.

**Images**  
<img src="./docs/images/Test_photo01.png" alt="Test_photo01" width="500"/>
<img src="./docs/images/Reflector_Transceiver_photo01.png" alt="Reflector Transceiver photo" width="500"/>
<img src="./docs/images/Hotplate_soldering_photo01.png" alt="Hotplate_soldering_photo01" width="500"/>
<img src="./docs/images/Tranceiver_PCB_3D_render.png" alt="Tranceiver PCB 3D Render" width="500"/>
<img src="./docs/images/Scope_signals02.png" alt="Scope_signals02" width="500"/>
<img src="./docs/images/debug_dsp_screenshot.png" alt="debug dps" width="500"/>

**Videos**  
These videos's were made during the assembly and testing stages.

[Capacitive absolute encoder demo oled display](https://www.youtube.com/watch?v=4YnHbx9MLcY)
Capacitive_absolute_encoder_demo_oled_display


[Capacitive absolute encoder signal processing debugging](https://www.youtube.com/watch?v=FkHQsDzOtvA)
[Capacitive absolute encoder demo 1](https://www.youtube.com/watch?v=fJ5A3xmAYgY)
[Capacitive absolute encoder demo 3](https://www.youtube.com/watch?v=_CGLx_bb51s)
[Capacitive absolute encoder solder paste application](https://www.youtube.com/watch?v=jppr43Z4cH8)
[Capacitive absolute encoder manual assembly](https://www.youtube.com/watch?v=WzHAo4lWglU)
[Capacitive absolute encoder solder paste application (single pcb)](https://www.youtube.com/watch?v=BiA69r2c_mw)
[capacitive absolute encoder hardware  development and testing](https://www.youtube.com/watch?v=zR5dzq9aDFo)
[Capacitive absolute encoder serial output testing](https://www.youtube.com/watch?v=iJKz_ekD_Zw)

## Working principle
 If multiple phase shifted sine waves of the same frequency are combined together the resulting waveform is a sine wave with the same frequency but with a phase shift that depends on phases of the combined signals.  
<img src="./docs/images/sine_addition_math_plot.png" alt="sine_addition_math_plot" width="500"/>  
*8 45 degrees shifted sine waves, addition of 4 adjacent waves.*  

A capacitive displacement sensor exploits this concept by creating a mechanical sensor structure consisting out of parallel plate capacitors. These capacitors combine half of the transmission signals depending on the mechanical displacement of the sensor structure. By measuring the phase shift of the combined wave the mechanical displacement can be calculated.

**Sensor construction**  
The sensor consists out of two PCB's a transceiver PCB that holds the transmit and receiving electrodes. And a reflector PCB that rotates relative to the transceiver PCB and couples/combines 50% of the transmit signals to the receiving electrode. The other 50% is coupled to ground.
<img src="./docs/images/Tranceiver_bottom_PCB_3D_render.png" alt="Tranceiver_bottom_PCB_3D_render" width="600"/>  
*Transceiver PCB*  

<img src="./docs/images/Reflector_top_PCB_3D_render.png" alt="Reflector_top_PCB_3D_render" width="500"/>  

*Reflector PCB*  

<img src="./docs/images/Reflector_Transceiver_pcb_overlay.png" alt="Reflector_Transceiver_pcb_overlay" width="500"/>  

*Transceiver and Reflector electrodes drawn on top of each other (to better see alignment)*  

## Electronics design
Generating eight 45 degrees apart phase shifted pure analog sine waves it quite difficult so instead the sine is modulated on a PWM signal which makes it easy and cheap to implement.
 
However the signals that are used in this project aren't sine modulated PWM signals but something else instead. At the moment it is still a mistery for me how these signals came to be. After many hours trying to figure out how to generate these signals from scratch I decided to clone them in stead.  
<img src="./docs/images/Transmit_signals.png" alt="Transmit_signals" width="1000"/> 

**Replicating the transmission signals**  
First the caliper transmission signals were measured using a logic analyser (*\ACRE\Other\Caliper Transmission signals.logicdata*). Using the Saleae Logic software the HIGH and LOW periods of a single transmit signal for a full period (when the full signal repeats) were measured and put into a excel sheet (*\ACRE\Other\Caliper Signal sample calculation.xlsx*). Next a time period T was choosen in which the signal could be expressed. 
Using this information and a matlab script a lookup table that can be used on the MCU was created (*\ACRE\Other\signalLutGeneration.m*).

**Bias signal**  
While reverse engineering a caliper another signal was discovered on the receiving electrode. This signal was pulling the receiver electrode to ground at a fixed interval in sync with the transmit signals. I believe the purpose of this signal is to discharge the receiver electrode in between samples. This signal was deconstructed and converted to a lookup table so it could be used to replicate the signal on a MCU.


