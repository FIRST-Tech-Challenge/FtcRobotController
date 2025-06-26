package org.firstinspires.ftc.teamcode.OpenCV;

public class AT_Notes {

/*

- commonly use the 36h11 family of april tags
- SDK also provides pose data (position, orientation) from cameras point of view
-> requires flat AprilTag ->  (was previously not possible in power play but possible with the side walls)


- offset from center of the camera to the center of the april tag calculateed in 3
components
-> X -> distance horizontal from center to the right
-> Y -> distance from lens center, outwards
-> Z -> distance from the center, upwards

**SDK doesn't provide measurements based on pixels but on IRL measurements (inches or cm)

- SDK evaluates AprilTag image and performs pose estimation
-> pose estimation is providing an estimate of XYZ distances between tag and
camera along with estimate angle of rotation


-> calibration data for webcam required for pose estimation
***already inputted the data for gobilda usb webcam

NAVIGATION

-OpModes use AT, evaluating inputs + driving to destination
-> opmode drives to position relative to tag or to tag

examples: RobotAutoDriveToAprilTagOmni.java
RobotAutoDriveToAprilTagTank.java
-> look at Advanced Use section for more navigation possibility

**SDK supports multiple camera
-> camera can be used for multiple image detection (detecting an object and AT)



**SDK provides rotation data for April Tags in the following:
pitch -> measure of rotation about X axis
Roll -> measure of rotation about Y axis
Yaw -> measure of rotation about Z axis




APRIL TAG DETECTION VALUES


PRY -> pitch roll and yaw (deg)

RBE -> range, bearing and elevation (inch, deg, deg)

XYZ -> right, forward, up, (inches)


pointing towards a target -> tank drive
-> when the AT is marked as a target to shoot towards
main measurements are tag range and tag bearing

 -> look at SDK sample: RobotAutoDriveToAprilTagTank.java


approaching a target squarely -> omni drive
-> if AT used to mark location of something to approach from front
 -> use tag Yaw -> indication ofo how far off camera is from tag image
 centerline

 -> eventually have to use Range bearing and Yaw to move towards target
 and end up directly in front of it

 -> can be performed with holonomic (omnidirectional) drive


 -> approach:

 1) target bearing used to turn bot towards target
 2) Yaw used to strafe sideways -> in turn rotating around the target to
 get directly in front of it
 3) range can be used to drive forward/backward to obtain correct standoff distance



 *** axis motions can be controlled by proportional control loop where turning
 towards tag is given highest priority, followed by strafing sideways,
 followed by approaching tag

  -> look at SDK sample: RobotAutoDriveToAprilTagOmni.java



LOCALIZATION



 */


}
