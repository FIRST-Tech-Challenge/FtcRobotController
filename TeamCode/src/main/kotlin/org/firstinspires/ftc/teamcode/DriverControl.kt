package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * CURRENT CONTROLS:
 *
 * Gamepad 1: Movement
 *  - Left Stick X/Y: Movement
 *  - Right Stick X: Rotation
 *  - X (face left): Toggle driver-relative controls (ON by default)
 * Gamepad 2: Objective
 *  - Left Stick Y: Manual Slide
 *  - Left Trigger: Close(?) Arm
 *  - Right Trigger: Open(?) Arm
 *  - Right Bumper: Retract(?) truss pulley
 *  - Left Bumper: Extend(?) truss pulley
 *  - D-Pad Up: Spin intake inwards
 *  - D-Pad Down: Spin intake outwards
 *  - A (face down): Toggle intake height
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

    // TODO: replace this function
    private fun prototypeInput() {
        val drive = shared.drive!!
//        val spinner = shared.motorIntakeSpin
        val intake = shared.intake
        // TODO: replace with Linear Slide Driver
        val slide = shared.motorSlide!!
        val hang = shared.motorTruss!!
        val arm = shared.servoArm!!

        // arm
        if (gamepad2.left_trigger > 0.1) arm.position += 0.001
        else if (gamepad2.right_trigger > 0.1) arm.position -= 0.001

        // TODO: should this be locked until endgame?
        //       we could use (timer > xx.xx) or something
        // truss hang
        shared.motorTruss?.power = if (gamepad2.right_bumper) 1.0 else if (gamepad2.left_bumper) -1.0 else 0.0

        // lift
        slide.mode = RUN_WITHOUT_ENCODER
        slide.power = if (abs(gamepad2.left_stick_y) > 0.1) -gamepad2.left_stick_y.toDouble() else 0.0

        // (intake) hold right bumper to rotate forward, left bumper to rotate backward
        if (gamepad2.dpad_up) intake?.active = Ternary.B; else if (gamepad2.dpad_down) Ternary.C else intake?.active = Ternary.A
    }


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
//        val powerModifier = 1.0 / (1.0 + sqrt(2.0 * (1.0 - abs((gyroYaw % (PI / 2)) - (PI / 4)) / (PI / 4))))
        val powerModifier = 1.0

        // Most values are [-1.0, 1.0]

        val control = object {
//            val movement = outputVector
            val rotation = -gamepad1.right_stick_x.toDouble()
            val relativityToggle = gamepad1.x
            val intakeHeightToggle = gamepad2.a
        }

        // toggle driver-relative controls
        if (control.relativityToggle) {
            if (!hasToggledDriveRelativity) {
                useBotRelative = !useBotRelative
                hasToggledDriveRelativity = true
            }
        } else hasToggledDriveRelativity = false

        // toggle intake lift
        if (control.intakeHeightToggle) {
            if (!hasToggledIntakeLift) {
                isIntakeLiftRaised = !isIntakeLiftRaised
                // kotlin noticed I forgot to finish this line BECAUSE OF ITS INDENTATION!!! HOW?!?! thanks compiler
                if (isIntakeLiftRaised) intake.raise() else intake.lower()
                hasToggledIntakeLift = true
            }
        } else hasToggledIntakeLift = false


        // +X = forward, +Y = left
        drive.setDrivePowers(PoseVelocity2d(
            if (useBotRelative) inputVector
            else Vector2d(
                driveRelativeX,
                driveRelativeY
            ) * powerModifier,
            control.rotation
        ))

        prototypeInput();

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

        telemetry.addLine("Gyro Yaw: " + shared.imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES))
        telemetry.addLine("Input Yaw: " + inputTheta * 180.0 / PI)
//        telemetry.addLine("Yaw Difference (bot - input): " + )

        telemetry.addLine("Left Stick X: " + gamepad1.left_stick_x)
        telemetry.addLine("Left Stick Y: " + gamepad1.left_stick_y)
        telemetry.addLine("Manual Controls: " + (if (useBotRelative) "EN" else "DIS") + "ABLED")
        telemetry.update()

        shared.update()
    }
}