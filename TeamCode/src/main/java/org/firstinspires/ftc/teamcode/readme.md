# Configuration: "Config 2025"

## Control Hub Portal:

### Control Hub:

#### Motors
|Port|Type|Name|
|---:|---|---|
|0 | GoBilda 5202/3/4 series | frontRight |
|1 | GoBilda 5202/3/4 series | frontLeft |
|2 | GoBilda 5202/3/4 series | backRight |
|3 | GoBilda 5202/3/4 series | backLeft |

#### Servos
|Port| Type                     |Name|
|---:|--------------------------|---|
|0 | Servo                    | wrist |
|1 | Continous Rotation Servo | intake |
|2 |                          | |
|3 |                          | |

#### Digital Devices
|Port|Type|Name|
|---:|---|---|
|0 | | |
|1 | | |
|2 | | |
|3 | | |
|4 | | |
|5 | | |
|6 | | |
|7 | | |

#### I2C Bus 0
|Port|Type|Name|
|---:|---|---|
|0 | REV internal IMU (BHI260AP) | imu |
|1 | | |
|2 | | |
|3 | | |

### Expansion Hub

#### Motors
|Port| Type                           | Name             |
|---:|--------------------------------|------------------|
|0 | REV Robotics 40:1 HD Hex Motor | armMotor         |
|1 | Gobilda 5202/3/4 series        | linearSlideMotor |
|2 |                                |                  |
|3 |                                |                  |

#### Servos
|Port|Type|Name|
|---:|---|---|
|0 | | |
|1 | | |
|2 | | |
|3 | | |

#### Digital Devices
|Port|Type|Name|
|---:|---|---|
|0 | | |
|1 | | |
|2 | | |
|3 | | |
|4 | | |
|5 | | |
|6 | | |
|7 | | |


#### I2C Bus 0
|Port|Type|Name|
|---:|---|---|
|0 | | |
|1 | | |
|2 | | |
|3 | | |
#### Webcam 1
4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""

# Button Mappings
## Gamepad 1
*Joystick Left* - Driving
*Joystick Right* - Turning
*DPAD* - 90 and 180 turns
*A* -
*B* -
*X* - Reset Global Positioning
*Y* - Global Positioning
*Trigger Right* - Precision Mode
*Trigger Left* - Not-so Precision Mode
*Bumper Right* -
*Bumper Left* -

## Gamepad 2

*Joystick Left* - Arm
*Joystick Right* - Linear Slide
*DPAD* - Default Linear Slide Positions
*A* - Intake
*B* -
*X* - Outtake
*Y* -
*Trigger Right* -
*Trigger Left* - 
*Bumper Right* - Wrist right
*Bumper Left* - Wrist left