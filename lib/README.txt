SSI APRS library v1.0
01/08/2017

Description
This library implements the AX.25 protocol for sending APRS packets using the DRA818 and Teensy 3.2. 
Pin A14, Teensy 3.2's DAC, is used to generate the AFSK1200 waveform.
Class support allows for possible extension to other radio modules

Functionality
Allows packets to be sent with GPS, timestamp, heading, speed, and altitude data, or simply as a custom format string.

Usage (Arduino):
Create a DRA818 object, and an APRS object
Call radio.init() to configure the DRA818 (optional)
Call sendPacketGPS() or sendPacketNoGPS() to send a packet
see aprs_lib in the examples folder.

Attributions:
Big thanks to rvnash for the code which this is based from, as well as the methods for converting lat/lon to string form and calculating the FCS sequence.

TODO:
clean up comments, add documentation, debug prints
