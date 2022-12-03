package org.firstinspires.ftc.teamcode.koawalib.auto

import com.acmerobotics.dashboard.config.Config
import com.asiankoala.koawalib.control.controller.PIDGains
import com.asiankoala.koawalib.control.motor.FFGains
import com.asiankoala.koawalib.hardware.motor.EncoderFactory
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.hardware.servo.KServo
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.odometry.KThreeWheelOdometry
import org.firstinspires.ftc.teamcode.koawalib.constants.ArmConstants
import org.firstinspires.ftc.teamcode.koawalib.constants.ClawConstants
import org.firstinspires.ftc.teamcode.koawalib.constants.LiftConstants
import org.firstinspires.ftc.teamcode.koawalib.constants.OdoConstants

class AutoHardware(startPose: Pose) {
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

    val liftLeadMotor = MotorFactory("liftLead")
        .float
        .forward
        .createEncoder(
            EncoderFactory(LiftConstants.ticksPerUnit)
            .zero(LiftConstants.homePos)
        )
        .withPositionControl(
            PIDGains(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD),
            FFGains(kS = LiftConstants.kS, kV = LiftConstants.kV, kA = LiftConstants.kA, kG = LiftConstants.kG),
            allowedPositionError = LiftConstants.allowedPositionError,
            disabledPosition = LiftConstants.disabledPosition
        )
        .build()

    val liftSecondMotor = MotorFactory("lift2")
        .reverse
        .float
        .build()

    val armMotor = MotorFactory("Arm")
        .reverse
        .float
        .createEncoder(
            EncoderFactory(ArmConstants.ticksPerUnit)
            .reverse
            .zero(ArmConstants.autoHomePos)
        )
        .withPositionControl(
            PIDGains(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD),
            FFGains(kS = ArmConstants.kS, kV = ArmConstants.kV, kA = ArmConstants.kA, kCos = ArmConstants.kCos),
            allowedPositionError = ArmConstants.allowedPositionError
        )
        .build()

    val clawServo = KServo("Claw")
        .startAt(ClawConstants.closePos)

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