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

    // these variables can be deleted when Gamepadyn is finished (state transitions cause headaches)
    /** true for lowered, false for raised */
    private var lastIntakeStatus = false
    private var hasToggledDriveRelativity = false
    private var hasToggledIntakeLift = false
    private var useBotRelative = false
    private var isIntakeLiftRaised = true

    override fun loop() {

        // nullables
        val drive = shared.drive!!
        val intake = shared.intake!!
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

        // \frac{1}{1+\sqrt{2\left(1-\frac{\operatorname{abs}\left(\operatorname{mod}\left(a,90\right)-45\right)}{45}\right)\ }}
        val powerModifier = 1.0 / (1.0 + sqrt(2.0 * (1.0 - (((abs(gyroYaw) % (PI / 2)) - (PI / 4)) / (PI / 4)))))

        // Most values are [-1.0, 1.0]

        val control = object {
//            val movement = outputVector
            val rotation = -gamepad1.right_stick_x.toDouble()
            val useManual = !gamepad1.x
            val relativityToggle = gamepad1.x
            val intakeHeightToggle = gamepad1.a
        }

        // toggle driver-relative controls
        if (control.relativityToggle) {
            if (!hasToggledDriveRelativity) {
                useBotRelative = !useBotRelative
                hasToggledDriveRelativity = true
            }
        } else hasToggledDriveRelativity = false

        // (intake) hold right bumper to rotate forward, left bumper to rotate backward
        if (gamepad1.right_bumper) intake.active = Ternary.B; else if (gamepad1.left_bumper) Ternary.C else intake.active = Ternary.A

        // toggle intake lift
        if (control.intakeHeightToggle) {
            if (!hasToggledIntakeLift) {
                isIntakeLiftRaised = !isIntakeLiftRaised
                // kotlin noticed I forgot to finish this line BECAUSE OF ITS INDENTATION!!! HOW?!?! thanks compiler
                if (isIntakeLiftRaised) intake.raise() else intake.lower()
                hasToggledIntakeLift = true
            }
        } else hasToggledIntakeLift = false

        shared.motorTruss?.power = if (gamepad1.dpad_up) 1.0 else if (gamepad1.dpad_down) -1.0 else 0.0

        // +X = forward, +Y = left
        drive.setDrivePowers(PoseVelocity2d(
            if (useBotRelative) inputVector
            else Vector2d(
                driveRelativeX,
                driveRelativeY
            ) * powerModifier,
            control.rotation
        ))

        telemetry.addLine("Gyro Yaw: " + shared.imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES))
        telemetry.addLine("Input Yaw: " + inputTheta * 180.0 / PI)
//        telemetry.addLine("Yaw Difference (bot - input): " + )

        telemetry.addLine("Left Stick X: " + gamepad1.left_stick_x)
        telemetry.addLine("Left Stick Y: " + gamepad1.left_stick_y)
        telemetry.addLine("Manual Controls: " + (if (useBotRelative) "EN" else "DIS") + "ABLED")
        telemetry.update()

//        // Intake controls
//        if (control.intakeHeightToggle != lastIntakeStatus) {
//            lastIntakeStatus = if (control.intakeHeightToggle) {
//                shared.intake?.lower()
//                true
//            } else {
//                shared.intake?.raise()
//                false
//            }
//        }
//        shared.intake?.active = control.intakeHeightToggle

        shared.update()
    }
}