package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.config.Config
import com.asiankoala.koawalib.hardware.motor.KEncoder
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.odometry.KThreeWheelOdometry
import org.firstinspires.ftc.teamcode.koawalib.constants.OdoConstants

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

    val liftMotor = MotorFactory("liftLead")
        .float
        .build()

    val liftSecondMotor = MotorFactory("lift2")
        .float
        .build()

    private val leftEncoder = KEncoder(fr, ticksPerUnit, true).reverse.zero()
    private val rightEncoder = KEncoder(fl, ticksPerUnit, true).zero()
    private val auxEncoder = KEncoder(br, ticksPerUnit, true).zero()

    val odometry = KThreeWheelOdometry(
        leftEncoder,
        rightEncoder,
        auxEncoder,
        OdoConstants.TRACK_WIDTH,
        OdoConstants.PERP_TRACKER,
        startPose
    )

    @Config
    companion object {
        private const val ticksPerUnit = 1892.3724
    }
}