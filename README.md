# Arduino-Nano-VU-Meter-and-Patterns
# video: https://www.youtube.com/watch?v=OavrHqIQ5Vc

Finally 7 Vus and 11 patterns working on Arduino Nano and Arduino Pro Mini 3.3 volts.

I say finally working because on the Arduino Nano when using Adafruit's Electret Microphone Amplifier - MAX4466 with Adjustable Gain,  the leds were showing a lot of noise. All this was caused because the Arduino Nano doesn't have an onboard 3.3 volts regulator. So the work around was to add one LD1117 3.3 volts regulator to power the microphone and the AREF input. And that fixed the problem.
You can also use an Arduino Pro Mini 3.3 volts and there is no need to use the //analogReference(EXTERNAL); 
