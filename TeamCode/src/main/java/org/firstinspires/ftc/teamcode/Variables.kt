package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection


object Variables {
    // Redundant thanks to odometry.
    var rotationsPerMeter = 3.3
    var encoders = 537.6

    // Farmers Market Servo Values
    var servoRelease = 0.0
    var servoClamp = 0.0

    var motorFL: DcMotor? = null
    var motorBL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorBR: DcMotor? = null

    var blinkinLedDriver: RevBlinkinLedDriver? = null
    var pattern: BlinkinPattern? = null

    var desiredTag: AprilTagDetection? = null
    var targetFound = false

    var DESIRED_DISTANCE = 12.0 //how close the camera should get to the target (inches)
    var SPEED_GAIN = 0.02 //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    var STRAFE_GAIN = 0.015 //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    var TURN_GAIN = 0.01 //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    var MAX_AUTO_SPEED = 0.5 //  Clip the approach speed to this max value (adjust for your robot)
    var MAX_AUTO_STRAFE = 0.5 //  Clip the approach speed to this max value (adjust for your robot)
    var MAX_AUTO_TURN = 0.3  //  Clip the turn speed to this max value (adjust for your robot)

    var drive = 0.0 //x
    var strafe = 0.0 //y
    var turn = 0.0 //yaw

    enum class Direction {
        FORWARD, BACKWARD, RIGHT, LEFT, ROTATE_LEFT, ROTATE_RIGHT, ROTATE
    }

    enum class BlinkinColor {
        RAINBOW, RED, RED_PULSE, ORANGE, ORANGE_PULSE, BLUE, GREEN, GREEN_PULSE, YELLOW, PURPLE, PINK
    }
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