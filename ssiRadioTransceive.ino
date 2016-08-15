Skip to content
This repository
Search
Pull requests
Issues
Gist
 @invictus1123
 Unwatch 1
  Star 0
 Fork 0 invictus1123/ssiRadio
 Code  Issues 0  Pull requests 0  Wiki  Pulse  Graphs  Settings
ssiRadio/ 
ssiRadioTransceive.ino
   or cancel
    
 Edit file    Preview changes


11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
 *  
 *  Overview of program:
 *  Foreground Code (non-interrupt operations):
 *  -Setup pins and radio, and make sure that radio is working. (atomic)
 *  -Assemble data into a string for transmission (TODO: integrate with avionics code, implementing a ClearToSend flag)
 *  -Convert the data into an APRS packet using the AX.25 Protocol.
 *      *AX.25 Protocol: A protocol used by APRS; it is a packet format which consists of:
 *                       *HDLC Flag*DestinationCallsign(s)*OwnCallsign*DigipeaterPath*DataIDBit*Data*FCS*HDLC Flag*
 *      *FCS: Frame Check Sequence, a 16 bit sequence calculated using some bit math which is sent with the transmission. The receiving end does the same math
 *            and determines whether any of the packet is corrupted: The results will be the same for a clean packet. (see calculateFCS() and checkIntegrity())
 *      **For a more in-depth explanation see http://www.aprs.org/doc/APRS101.PDF p.22      
 *  -Put the packet into a queue which will be read and transmitted with an interrupt function (atomic)    
 *  -Verify the integrity of and print out any received packets
 *  Background Code (interrupt operations, running at SAMPLE_RATE times per second inbetween foreground operations)
 *  -Take the bytes of the packet and split them into bits
 *  -Convert those bits into an analog waveform using Audio Frequency Shift Keying (AFSK) and a sine table using Non-Return to Zero (NRZ) encoding and Bit-Stuffing
 *      *AFSK: Send 1s and 0s by changing an audio tone between two frequencies: MARK (1200Hz) and SPACE (2200Hz).
 *             See NRZ: These frequencies don't represent 1 and 0 in digital data, but an unchanging signal "mark" and a changing signal "space"
 *      *NRZ:  A method for encoding digital data common in packet radio: The first digital bit equals a Mark ("1") and changes to Space ("0")
 *             if the next digital bit is different. Example: 01100101 is transmitted as MSMSMSSS, or 10101000. This, in combination with Bit-Stuffing, helps   
 *             make sure the transmitter and receiver are synchronized, that is, the receiver does not lose track of which part of a byte it is on or what part
 *             of a transmission it is on.
 *      *Bit-Stuffing: If at any point too many marks are transmitted (which would result in one tone being played for many samples),
 *             the receiver may lose track of where it is in the transmission ('bit slip'). Thus in transmissions whenever the data results in 5 or more marks being
 *             transmitted, a space will be put in (which must be parsed out later) to cause the tone to change and prevent bit slip.
 *  -Read samples coming in from the audioPin and discern what frequency they are    
 *  -Convert the samples into NRZ bits, looking for an APRS flag character (HDLC_FLAG) which will indicate the beginning of a transmission
 *  -Convert the NRZ bits into data once we have a transmission, filtering out any stuffed bits.
@invictus1123
Commit changes


Update 

Add an optional extended description…
  Commit directly to the master branch.
  Create a new branch for this commit and start a pull request. Learn more about pull requests.
Commit changes  Cancel
Contact GitHub API Training Shop Blog About
© 2016 GitHub, Inc. Terms Privacy Security Status Help
