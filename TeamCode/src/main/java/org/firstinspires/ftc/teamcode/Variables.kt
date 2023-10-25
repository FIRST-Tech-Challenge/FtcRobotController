package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
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
    var slideGate: Servo? = null
    var rMotorR: DcMotor? = null;
    var rMotorL: DcMotor? = null;
    var touchyR: TouchSensor? = null
    var touchyL: TouchSensor? = null
    var slideRotationMotor: DcMotor? = null
    var motorSlideLeft: DcMotor? = null
    var clawRotation: Servo? = null
    var clawMotor: Servo? = null

    var closedClaw = 0.87
    var openClaw = 0.8
    var bottom = 0
    var low = 0 //unknown
    var mid = 0 //unknown
    var high = 1600
    var speed = 400
    var lPower = 0.2
    var rPower = -0.22280685416
    var lMax = -10500
    var rMax = 10000
    var lMin = 0
    var rMin = 0
    
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
    var clawToBoard = 0.1;
    var x =  0.0;
    var y = 0.0;
    var slideLength = 0.0;
    var slideAngle = 0.0;
    var clawAngle = 0.0;

    enum class VisionProcessors {
        TFOD, APRILTAG, BOTH
    }
}