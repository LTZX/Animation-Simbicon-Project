# ECS279 Final Project - Simbicon Study
This is the final project for ECS 279 at UC Davis. The main focus of the project is about Simbicon. There are two parts included in this project.  

## C++ Re-implementation
All the codes in folder **RI-C++** is for the re-implementation of the functions that compute the torques applied at every stage of the simulation to drive the character to achieve desired pose, which involves deriving the torque using PD controller, as well as calibrating the torque on two hips.  

1. Download the code from Simbicon project [webpage](https://www.cs.ubc.ca/~van/simbicon_cef/index.html).  
2. Replace the following files.  
  - PoseController.h
  - PoseController.cpp
  - SimBiController.h
  - SimBiController.cpp
3. Set the AppGui as the start project in visual studio
4. Build the solution and run.

## SimbiconJS Extension
All the other files in this repo is for the extension of [SimbiconJS](https://github.com/mfirmin/SimbiconJS).  
[Demo](https://ltzx.github.io.)
