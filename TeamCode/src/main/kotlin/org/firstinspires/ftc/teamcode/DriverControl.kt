package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt

/*
    CONTROL IDEAS:

    1. Driver-relative control *by default*
        a. Hold some button down to use bot-relative (for precise movements)
        b. Use the onboard gyroscope for this
    2.
 */
@TeleOp(name = "STANDALONE Kotlin Driver Control", group = "Kt")
class StandaloneDriverControl : DriverControlBase(Pose2d(0.0, 0.0, 0.0))
@TeleOp(name = "Kotlin Driver Control", group = "Kt")
class DriverControl : DriverControlBase(BotShared.storedPose)

open class DriverControlBase(private val initialPose: Pose2d) : OpMode() {

    private lateinit var shared: BotShared

    override fun init() {
//        val setter = DriverControl::tagCamera.setter
        shared = BotShared(this)
        shared.drive = MecanumDrive(hardwareMap, initialPose)
    }

    override fun start() {
        super.start()
//        shared.intake.raise()
    }

    /** true for lowered, false for raised */
    private var lastIntakeStatus = false
    private var hasToggledManual = false
    private var useManual = false

    override fun loop() {

        val drive = shared.drive!!
//        val lsd = shared.lsd!!

        // TODO: test driver relative, check servos, add it to setDrivePowers()

        // counter-clockwise
        val gyroYaw = shared.imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

        // +X = forward
        // +Y = left
        val inputVector = Vector2d(
            // up
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
        )

        // angle of the stick
        val inputTheta = atan2(inputVector.y, inputVector.x)
        // evaluated theta
        val driveTheta = inputTheta - gyroYaw // + PI
        // magnitude of inputVector clamped to [0, 1]
        val inputPower = clamp(sqrt(
            (inputVector.x * inputVector.x) +
            (inputVector.y * inputVector.y)
        ), 0.0, 1.0)

        val driveRelativeX = cos(driveTheta) * inputPower
        val driveRelativeY = sin(driveTheta) * inputPower

        if (gamepad1.x) {
            if (!hasToggledManual) {
                useManual = !useManual
                hasToggledManual = true
            }
        } else hasToggledManual = false

        // Most values are [-1.0, 1.0]

        val control = object {
//            val movement = outputVector
            val rotation = -gamepad1.right_stick_x.toDouble()
            val useManual = !gamepad1.x
            val intake = gamepad1.a
        }

        // +X = forward, +Y = left
        drive.setDrivePowers(PoseVelocity2d(
            if (useManual) inputVector
            else Vector2d(
                driveRelativeX,
                driveRelativeY
            ),
            control.rotation
        ))

        telemetry.addLine("Gyro Yaw: " + shared.imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES))
        telemetry.addLine("Input Yaw: " + inputTheta * 180.0 / PI)
//        telemetry.addLine("Yaw Difference (bot - input): " + )

        telemetry.addLine("Left Stick X: " + gamepad1.left_stick_x)
        telemetry.addLine("Left Stick Y: " + gamepad1.left_stick_y)
        telemetry.addLine("Manual Controls: " + (if (useManual) "EN" else "DIS") + "ABLED")
        telemetry.update()

        // Intake controls
        if (control.intake != lastIntakeStatus) {
            lastIntakeStatus = if (control.intake) {
                shared.intake?.lower()
                true
            } else {
                shared.intake?.raise()
                false
            }
        }
        shared.intake?.active = control.intake

        shared.update()
    }
}