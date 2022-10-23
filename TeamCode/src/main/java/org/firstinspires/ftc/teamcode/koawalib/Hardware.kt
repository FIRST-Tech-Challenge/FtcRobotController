package org.firstinspires.ftc.teamcode.koawalib

import com.asiankoala.koawalib.control.controller.PIDGains
import com.asiankoala.koawalib.control.motor.DisabledPosition
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
import org.firstinspires.ftc.teamcode.koawalib.vision.AprilTagDetectionPipeline
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

    val liftMotor = MotorFactory("Lift")
        .float
        .forward
        .createEncoder(Lift.ticksPerUnit, false)
        .zero(Lift.homePos)
        .withMotionProfileControl(
            PIDGains(TODO(), TODO(), TODO()),
            FFGains(kG = TODO(), kS = TODO(), kV = TODO(), kA = TODO()),
            MotionConstraints(TODO(), TODO()),
            allowedPositionError = TODO(),
            disabledPosition = DisabledPosition(TODO())
        )
        .build()

    val armMotor = MotorFactory("Arm")
        .float
        .createEncoder(Arm.ticksPerUnit, false)
        .zero(Arm.homePos)
        .withMotionProfileControl(
            PIDGains(TODO(), TODO(), TODO()),
            FFGains(kCos = TODO()),
            MotionConstraints(TODO(), TODO()),
            allowedPositionError = TODO(),
            disabledPosition = DisabledPosition(TODO())
        )
        .build()

    val clawServo = KServo("Claw")
        .startAt(Claw.closePos)

    val distanceSensor = KDistanceSensor("distanceSensor")

    val lights = KServo("Lights")

    val webcam = Webcam("Webcam", AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506))

    private val leftEncoder = KEncoder(fr, ticksPerUnit, true).reverse.zero()
    private val rightEncoder = KEncoder(fl, ticksPerUnit, true).zero()
    private val auxEncoder = KEncoder(br, ticksPerUnit, true).zero()

    val odometry = KThreeWheelOdometry(
        leftEncoder,
        rightEncoder,
        auxEncoder,
        TODO(),
        TODO(),
        startPose
    )

    companion object {
        private const val ticksPerUnit = 1892.3724
    }
}