package org.firstinspires.ftc.teamcode.testing

import com.asiankoala.koawalib.hardware.motor.KEncoder
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.odometry.KThreeWheelOdometry
import org.firstinspires.ftc.teamcode.koawalib.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.koawalib.vision.Webcam

class Hardware(startPose: Pose) {
    val fl = MotorFactory("fl")
        .reverse
        .brake
        .build()

    val bl = MotorFactory("bl")
        .reverse
        .brake
        .build()

    val br = MotorFactory("br")
        .forward
        .brake
        .build()

    val fr = MotorFactory("fr")
        .forward
        .brake
        .build()

//    val arm = MotorFactory("arm")
//        .forward
//        .float
//        .createEncoder(672.0/90.0, false)
//        .zero(Arm.initPos)
//        .withPositionControl(
//            PIDGains(0.13, 0.0, 0.0033),
////            PIDGains(0.0, 0.0, 0.0)
//            FFGains(kCos = 0.1),
//            allowedPositionError = 2.0,
//            disabledPosition = DisabledPosition(Arm.initPos)
//        )
//        .build()

//    val arm = MotorFactory("arm")
//        .forward
//        .float
//        .createEncoder(672.0/90.0, false)
//        .zero(Arm.initPos)
//        .withMotionProfileControl(
//            PIDGains(0.0, 0.0, 0.0),
//            FFGains(
//                kCos = 0.1,
//                kS = 0.06,
//                kV = 0.003
//            ),
//            MotionConstraints(
//                60.0,
//                60.0
//            ),
//            allowedPositionError = 2.0,
//            disabledPosition = DisabledPosition(Arm.initPos)
//        )
//        .build()

    val webcam = Webcam("Webcam", AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506))

    private val leftEncoder = KEncoder(fr, ticksPerUnit, true).reverse.zero()
    private val rightEncoder = KEncoder(fl, ticksPerUnit, true).zero()
    private val auxEncoder = KEncoder(br, ticksPerUnit, true).zero()

    val odometry = KThreeWheelOdometry(
        leftEncoder,
        rightEncoder,
        auxEncoder,
        9.86,
        8.325,
        startPose
    )

    companion object {
        private const val ticksPerUnit = 1892.3724
    }
}