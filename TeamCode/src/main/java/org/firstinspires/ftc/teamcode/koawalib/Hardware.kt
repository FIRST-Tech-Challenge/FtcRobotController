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
        .createEncoder(Lift.ticksPerUnit, false)
        .zero(Lift.homePos)
        .withMotionProfileControl(
            PIDGains(Lift.kP, Lift.kI, Lift.kD),
            FFGains(Lift.kS, Lift.kV, Lift.kA, kG = Lift.kG),
            MotionConstraints(Lift.maxVel, Lift.maxAccel),
            allowedPositionError = Lift.allowedPositionError,
            disabledPosition = Lift.disabledPosition
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
        .createEncoder(Arm.ticksPerUnit, false)
        .zero(Arm.homePos)
        .withMotionProfileControl(
            PIDGains(Arm.kP, Arm.kI, Arm.kD),
            FFGains(Arm.kS, Arm.kV, Arm.kA, Arm.kCos),
            MotionConstraints(Arm.maxVel, Arm.maxAccel),
            allowedPositionError = Arm.allowedPositionError,
        )
        .build()

    val clawServo = KServo("Claw")
        .startAt(Claw.closePos)

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
        TRACK_WIDTH,
        PERP_TRACKER,
        startPose
    )

    @Config
    companion object {
        private const val ticksPerUnit = 1892.3724
        @JvmField var TRACK_WIDTH = 0.0
        @JvmField var PERP_TRACKER = 0.0
    }
}