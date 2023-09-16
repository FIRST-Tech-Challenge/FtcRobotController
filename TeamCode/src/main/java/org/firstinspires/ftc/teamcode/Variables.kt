package org.firstinspires.ftc.teamcode

object Variables {
    // Redundant thanks to odometry.
    var rotationsPerMeter = 3.3
    var encoders = 537.6

    // Farmers Market Servo Values
    var servoRelease = 0.0
    var servoClamp = 0.0

    enum class VisionProcessors {
        TFOD, APRILTAG, BOTH
    }
}