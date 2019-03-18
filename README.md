# ECS279 Final Project - Simbicon Study
This is the final project for ECS 279 at UC Davis. The main focus of the project is about Simbicon. There are two parts included in this project.  

The report is included as **report.pdf**. We have a screen record (**ScreenRecord.mp4**) for C++ re-implementation, and Demo for SimbiconJS extension. [Demo](https://ltzx.github.io.)

## C++ Re-implementation
All the codes in folder **RI-C++** is for the re-implementation of the functions that compute the torques applied at every stage of the simulation to drive the character to achieve desired pose, which involves deriving the torque using PD controller, as well as calibrating the torque on two hips.  

1. Download the code from Simbicon project [webpage](https://www.cs.ubc.ca/~van/simbicon_cef/index.html).  
2. Replace the following files in /release/src/Core.  
  - PoseController.h
  - PoseController.cpp
  - SimBiController.h
  - SimBiController.cpp
3. Set the AppGui as the start project in visual studio
4. Build the solution and run.  

We also have a screen record (ScreenRecord.mp4) that compare the original one and re-implemented one side by side with 2 different styles.

## SimbiconJS Extension
All the other files in this repo is for the extension of [SimbiconJS](https://github.com/mfirmin/SimbiconJS).  
Again, here is the [demo](https://ltzx.github.io.) for the extension.
