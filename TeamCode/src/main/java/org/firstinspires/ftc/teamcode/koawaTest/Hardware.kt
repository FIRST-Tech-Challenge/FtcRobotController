package org.firstinspires.ftc.teamcode.koawaTest

import com.asiankoala.koawalib.hardware.motor.EncoderFactory
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.odometry.KThreeWheelOdometry
import com.asiankoala.koawalib.subsystem.vision.KWebcam
import org.firstinspires.ftc.teamcode.koawalib.constants.OdoConstants
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline

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
        .build()

    val fr = MotorFactory("fr")
        .forward
        .build()

    private val leftEncoder = EncoderFactory(ticksPerUnit)
        .revEncoder
        .build(fl)
    private val rightEncoder = EncoderFactory(ticksPerUnit)
        .reverse
        .revEncoder
        .build(bl)
    private val auxEncoder = EncoderFactory(ticksPerUnit)
        .reverse
        .revEncoder
        .build(fr)

    val liftMotor = MotorFactory("liftLead")
        .float
        .build()

    val liftSecondMotor = MotorFactory("lift2")
        .reverse
        .float
        .build()

    val arm = MotorFactory("Arm")
        .reverse
        .float
        .build()

    val odometry = KThreeWheelOdometry(
        leftEncoder,
        rightEncoder,
        auxEncoder,
        OdoConstants.TRACK_WIDTH,
        OdoConstants.PERP_TRACKER,
        startPose
    )

    companion object {
        private const val ticksPerUnit = 1892.3724
    }
}