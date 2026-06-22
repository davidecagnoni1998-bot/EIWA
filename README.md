# EIWA 
Drone almost-real time telemetry system with virtual cockpit

## Overview
This project is designed to provide a customizable and modular system installable on diverse drone and general RC models
to have almost-real time telemetry data on a ground station and perform BVLOS flight

## Hardware
- Heltec WifiKit 32 as central core and USB radio link
- Arduino Pro Mini on each unit

## Software
Developed using:
- Arduino IDE
- Visual Code 

## Project Structure
The "SW airborne" folder contains the code for the central core and all the units present on the module or the drone
The "SW ground" folder contains the MatLab applications files and the code running on the USB radio link for the ground station
