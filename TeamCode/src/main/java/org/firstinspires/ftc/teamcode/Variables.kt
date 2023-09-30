package org.firstinspires.ftc.teamcode

object Variables {
    // Redundant thanks to odometry.
    var rotationsPerMeter = 3.3
    var encoders = 537.6

    // Farmers Market Servo Values
    var servoRelease = 0.0
    var servoClamp = 0.0

    var t = 0.0;
    var slideToBoard = 0.0;
    var clawToBoard = 0.0;
    var x =  slideToBoard - clawToBoard + .5*t;
    var y = Math.sqrt(3.0)/2 * t;
    var slideLength = Math.sqrt(Math.pow(x, 2.0) + Math.pow(y, 2.0));
    var slideAngle = Math.atan(y/x);
    var clawAngle = 60 - slideAngle;

    enum class VisionProcessors {
        TFOD, APRILTAG, BOTH
    }
}