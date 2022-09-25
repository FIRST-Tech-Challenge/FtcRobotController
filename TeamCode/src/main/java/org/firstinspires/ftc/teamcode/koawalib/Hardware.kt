package org.firstinspires.ftc.teamcode.koawalib

import com.asiankoala.koawalib.control.controller.PIDGains
import com.asiankoala.koawalib.control.motor.FFGains
import com.asiankoala.koawalib.hardware.motor.KEncoder
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.odometry.KThreeWheelOdometry

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

    val arm = MotorFactory("arm")
        .forward
        .float
        .createEncoder(672.0/90.0, false)
        .zero(-55.0)
        .withPositionControl(
            PIDGains(0.09, 0.0, 0.0009),
            FFGains(kCos = 0.1),
            allowedPositionError = 2.0,
        )
        .build()

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