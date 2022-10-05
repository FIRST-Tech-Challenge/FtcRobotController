package org.firstinspires.ftc.teamcode.koawalib

import com.asiankoala.koawalib.hardware.motor.KEncoder
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.hardware.servo.KServo
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.odometry.KThreeWheelOdometry
import org.firstinspires.ftc.teamcode.koawalib.vision.Webcam

class Hardware(startPose: Pose) {
    val fl = MotorFactory("fl")
        .forward
        .brake
        .build()

    val bl = MotorFactory("bl")
        .forward
        .brake
        .build()

    val br = MotorFactory("br")
        .reverse
        .brake
        .build()

    val fr = MotorFactory("fr")
        .reverse
        .brake
        .build()

//    val slides = MotorFactory("Slides")
//        .reverse
//        .brake
//        .build()


//    val arm = KServo("Arm")
//    val claw = KServo("Claw").startAt(Claw.openPos)

    val webcam = Webcam("Webcam")

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