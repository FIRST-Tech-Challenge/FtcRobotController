package org.firstinspires.ftc.teamcode.koawalib

import com.acmerobotics.dashboard.config.Config
import com.asiankoala.koawalib.control.controller.PIDGains
import com.asiankoala.koawalib.control.motor.FFGains
import com.asiankoala.koawalib.control.profile.MotionConstraints
import com.asiankoala.koawalib.hardware.motor.KEncoder
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.hardware.sensor.KDistanceSensor
import com.asiankoala.koawalib.hardware.servo.KServo
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.odometry.KThreeWheelOdometry
import org.firstinspires.ftc.teamcode.koawalib.constants.ArmConstants
import org.firstinspires.ftc.teamcode.koawalib.constants.ClawConstants
import org.firstinspires.ftc.teamcode.koawalib.constants.LiftConstants
import org.firstinspires.ftc.teamcode.koawalib.constants.OdoConstants
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline
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

    val liftLeadMotor = MotorFactory("liftLead")
        .float
        .forward
        .createEncoder(LiftConstants.ticksPerUnit, false)
        .zero(LiftConstants.homePos)
        .withMotionProfileControl(
            PIDGains(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD),
            FFGains(LiftConstants.kS, LiftConstants.kV, LiftConstants.kA, kG = LiftConstants.kG),
            MotionConstraints(LiftConstants.maxVel, LiftConstants.maxAccel),
            allowedPositionError = LiftConstants.allowedPositionError,
            disabledPosition = LiftConstants.disabledPosition
        )
        .build()

    val liftSecondMotor = MotorFactory("lift2")
        .float
        .build()

    val liftThirdMotor = MotorFactory("lift3")
        .float
        .build()

    val armMotor = MotorFactory("Arm")
        .float
        .createEncoder(ArmConstants.ticksPerUnit, false)
        .zero(ArmConstants.homePos)
        .withMotionProfileControl(
            PIDGains(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD),
            FFGains(ArmConstants.kS, ArmConstants.kV, ArmConstants.kA, ArmConstants.kCos),
            MotionConstraints(ArmConstants.maxVel, ArmConstants.maxAccel),
            allowedPositionError = ArmConstants.allowedPositionError,
        )
        .build()

    val clawServo = KServo("Claw")
        .startAt(ClawConstants.closePos)

    val distanceSensor = KDistanceSensor("distanceSensor")

    val lights = KServo("Lights")

    val webcam = Webcam("Webcam", SleevePipeline(0.166, 578.272, 578.272, 402.145, 221.506))

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