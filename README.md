## Description

2026 FRC code for DiscoBots, team 2587. 
Drivetrain: REV MAXSwerve Modules, SPARKS MAX, a NEO as the driving motor, a NEO 550 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder. 
  (Sparx Flex is not supported without modifications to code) 

IMU: NavX 1.0 
Vision: April Tags tracking using two cameras 
Path Planning Code: based on PathPlannerLib 

## Prerequisites

* SPARK MAX Firmware v26.1.0
* REVLib v2026.0.0
* update the other pre-reqs when vision and pathplanning are added 

## DriveTrain Configuration

If cloning this repo for another team, Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!
These values can be adjusted in the `Configs.java` and `Constants.java` files.

Verify calibration to zero offsets for the absolute encoders in Hardware Client 2 using the `Absolute Encoder` utility under the associated turning SPARK devices.

# PathPlanner Configuration
Paths will be generated and stored in deploy/pathplanner 

# Vision Configuration 
Cameras will need to be connected and configured 


# Based on the MAXSwerve Java Template v2026.0
See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/CHANGELOG.md) for information about updates to the REV template.

