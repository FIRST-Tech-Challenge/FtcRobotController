Configuration
==============
This discusses the configuration files (in ``/teamcode/org/ftc/firstinspires/teamcode/Config/``) that you might have to edit, the others do not have to be edited.

Main Configuration
___________________

``name`` - The robot's name

``version`` - The robot's version

``allianceColor`` - The alliance color, it's either AllianceColor.BLUE or AllianceColor.RED.

``debug`` - Either true or false, at the current moment it does not do anything

``debugTarget`` - "none" means no target enter a file path to debug a file enter a folder path to debug a folder, "*" means
debug all

``logLevel`` - 0 is quiet, the maximum is 5.

``initSubsystems`` - Whether to initialize the subsystems, it might break stuff if it is false.

``initSubsystemControl`` - Whether to initialize control (It might crash).

``initSubsystemDrive`` - Whether to initialize drive.

``initSubsystemVision`` - Whether to initialize vision.

``initMechanical`` - Set to false if you don't want to init Mechanical stuff

``initGetGamePadInputs`` - Set to false if you are not using the game pad.

``initHardwareMap`` - Set to false if you do not want to init the hardware map.

Vision Configuration
_____________________
``CAMERA_WIDTH`` - the width of the wanted camera resolution

``CAMERA_HEIGHT`` - the height of the wanted camera resolution

``HORIZON`` - the horizon value to tune

``WEBCAM_NAME`` - the name of the webcam

``VUFORIA_KEY`` - The Vuforia Key

``finalMarkerLocation`` - Where the marker is
