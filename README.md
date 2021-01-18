# ****RC Car shenanigans****
The purpose of this repo is to document various resources in reagrds to my private RC car project.

## Overview
The project consists of modding a 1:10 RC car to be capable of having an FPV feed.  
This is achieved using standard FPV gear that is mainly used with FPV quads.

## Hardware
###### Control unit
The car will be controlled using a BF compatible FC.
The reasons for this are as follows:
 - Easy interfacing with additional sensors.
   - Mainly a GPS.
 - Possibility of exchanging telemetry data with a transmitter. 
 - Creating a MAVLink interface.

###### External MCUs

An Arduino will be used to parse the MAVLink data of the FC.
Based on this telemetry data, the arduino will execute certain actions that the FC cannot do.

###### Misc extra hardware
An addressable light strip will be used to interface with the arduino.  
The idea is to have the arduino control the LEDs based on MAVLink channel telemetry.
