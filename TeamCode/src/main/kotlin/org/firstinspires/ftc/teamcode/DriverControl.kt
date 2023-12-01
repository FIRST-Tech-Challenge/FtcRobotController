package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
import com.qualcomm.robotcore.hardware.Servo
import computer.living.gamepadyn.ActionBind
import computer.living.gamepadyn.Configuration
import computer.living.gamepadyn.GDesc
import computer.living.gamepadyn.Gamepadyn
import computer.living.gamepadyn.InputType.ANALOG
import computer.living.gamepadyn.InputType.DIGITAL
import computer.living.gamepadyn.RawInput
import computer.living.gamepadyn.ftc.InputSystemFtc
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.DriverControlBase.Action.CLAW
import org.firstinspires.ftc.teamcode.DriverControlBase.Action.DEBUG_ACTION
import org.firstinspires.ftc.teamcode.DriverControlBase.Action.MOVEMENT
import org.firstinspires.ftc.teamcode.DriverControlBase.Action.ROTATION
import org.firstinspires.ftc.teamcode.DriverControlBase.Action.SPIN_INTAKE
import org.firstinspires.ftc.teamcode.DriverControlBase.Action.TOGGLE_DRIVER_RELATIVITY
import org.firstinspires.ftc.teamcode.DriverControlBase.Action.TOGGLE_INTAKE_HEIGHT
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
 *
 * Gamepad 1: Movement
 *  - Left Stick X/Y: Movement
 *  - Right Stick X: Rotation
 *  - X (face left): Toggle driver-relative controls (ON by default)
 *  - D-Pad Up: Spin intake outwards
 *  - D-Pad Down: Spin intake inwards
 *
 * Gamepad 2: Objective
 *  - Left Stick Y: Manual slide
 *  - Left Trigger: Close(?) claw
 *  - Right Trigger: Open(?) claw
 *  - Right Bumper: Retract truss pulley
 *  - Left Bumper: Extend truss pulley
 *  - A (face down): Toggle intake height
 */
@TeleOp(name = "Standalone Driver Control", group = "Tele Sub-Mode")
class StandaloneDriverControl : DriverControlBase(Pose2d(0.0, 0.0, 0.0))
@TeleOp(name = "# Driver Control", group = "Tele Sub-Mode")
class DriverControl : DriverControlBase(BotShared.storedPose)

open class DriverControlBase(private val initialPose: Pose2d) : OpMode() {

    private lateinit var shared: BotShared

    /**
     * The time (in ms, relative to the OpMode's start time) that the robot last ran the `loop()` function.
     */
    private var lastLoopTime = 0.0
//    TODO: fix these delta time calculations!
    private var deltaTime = 60.0 / 1000.0

    // these variables can be deleted when Gamepadyn is finished (state transitions cause headaches)
    /** true for lowered, false for raised */
    private var lastIntakeStatus = false
    private var useBotRelative = false
    private var isIntakeLiftRaised = true

    enum class Action {
        MOVEMENT,
        ROTATION,
        SPIN_INTAKE,
        CLAW,
        TOGGLE_DRIVER_RELATIVITY,
        TOGGLE_INTAKE_HEIGHT,
        DEBUG_ACTION
    }

    private lateinit var gamepadyn: Gamepadyn<Action>

    /**
     * Set up the robot
     */
    override fun init() {
//        val setter = DriverControl::tagCamera.setter
        shared = BotShared(this)
        shared.drive = MecanumDrive(hardwareMap, initialPose)
        gamepadyn = Gamepadyn(InputSystemFtc(this), strict = true, useInputThread = false,
            MOVEMENT                    to GDesc(ANALOG, 2),
            ROTATION                    to GDesc(ANALOG, 1),
            SPIN_INTAKE                 to GDesc(ANALOG, 1),
            CLAW                        to GDesc(ANALOG, 1),
            TOGGLE_DRIVER_RELATIVITY    to GDesc(DIGITAL),
            TOGGLE_INTAKE_HEIGHT        to GDesc(DIGITAL),
            DEBUG_ACTION                to GDesc(DIGITAL)
        )

        // Configuration
        gamepadyn.players[0].configuration = Configuration(
            ActionBind(RawInput.FACE_X, TOGGLE_DRIVER_RELATIVITY),
            ActionBind(RawInput.FACE_A, TOGGLE_INTAKE_HEIGHT),
        )

        val intake = shared.intake!!

        // toggle driver-relative controls
        gamepadyn.players[0].getEventDigital(TOGGLE_DRIVER_RELATIVITY)!!.addListener { useBotRelative = !useBotRelative }
        // toggle intake height
        gamepadyn.players[0].getEventDigital(TOGGLE_INTAKE_HEIGHT)!!.addListener { if (intake.raised) intake.lower() else intake.raise() }
    }

    override fun start() {
        lastLoopTime = time
    }

    /**
     * The robot's game loop. Handles input, updates the motors, basically calls everything.
     */
    override fun loop() {
        deltaTime = (time - lastLoopTime)
        lastLoopTime = time


        /**
         * Run the various update functions
         */
        updateDrive()
        updateSlide()
        updateIntake()
        updateTrussHang()

        // Most input values are [-1.0, 1.0]

        telemetry.addLine("Left Stick X: ${gamepad1.left_stick_x}")
        telemetry.addLine("Left Stick Y: ${gamepad1.left_stick_y}")
        telemetry.addLine("Manual Controls: ${if (useBotRelative) "EN" else "DIS"}ABLED")
        telemetry.addLine("Delta Time: $deltaTime")
        telemetry.update()

        shared.update()
    }

    /**
     * Handle controls for the truss pulley
     */
    private fun updateTrussHang() {
        // TODO: should this be locked until endgame?
        //       we could use (timer > xx.xx) or something
        shared.motorTruss?.power =
            if (gamepad2.right_bumper) -1.0     // pull it in
            else if (gamepad2.left_bumper) 1.0  // let it loose
            else 0.0
    }

    /**
     * Update bot movement (drive motors)
     */
    private fun updateDrive() {
        val drive = shared.drive!!

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

        // +X = forward, +Y = left
        drive.setDrivePowers(PoseVelocity2d(
            if (useBotRelative) inputVector else Vector2d(
                driveRelativeX,
                driveRelativeY
            ) * powerModifier,
            -gamepad1.right_stick_x.toDouble()
        ))

        telemetry.addLine("Gyro Yaw: " + shared.imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES))
        telemetry.addLine("Input Yaw: " + if (inputVector.x > 0.05 && inputVector.y > 0.05) inputTheta * 180.0 / PI else 0.0)
//        telemetry.addLine("Yaw Difference (bot - input): " + )
    }

    /**
     * Update the linear slide
     */
    private fun updateSlide() {
        // TODO: replace with Linear Slide Driver
        val slide = shared.motorSlide!!
//        val lsd = shared.lsd!!

        // lift/slide
        slide.mode = RUN_WITHOUT_ENCODER
        slide.power = if (abs(gamepad2.left_stick_y) > 0.1) -gamepad2.left_stick_y.toDouble() else 0.0
    }

    /**
     * Update the intake mechanisms (spinner, claw, arm)
     */
    private fun updateIntake() {
        val intake = shared.intake
        val arm = shared.servoArm!!
        val claw = shared.claw!!

        // Arm
        val armPos = (arm.position + Servo.MAX_POSITION * /*deltaTime*/ 0.01 * -gamepad2.right_stick_y)
        arm.position = armPos.clamp(Servo.MIN_POSITION, Servo.MAX_POSITION)

// TODO: need to make claw work
        claw.state += 0.5 * 0.01 * (gamepad2.right_trigger - gamepad2.left_trigger)

// Claw
//        claw.state +=
//        clawLeft.position +=    Servo.MAX_POSITION * 0.5 * deltaTime * (gamepad2.right_trigger - gamepad2.left_trigger)
//        clawRight.position +=   Servo.MAX_POSITION * 0.5 * deltaTime * (gamepad2.right_trigger - gamepad2.left_trigger)

        // spinner
        intake?.active =
            if (gamepad1.dpad_down) 1.0     // inwards
            else if (gamepad1.dpad_up) -1.0 // outwards
            else 0.0                        // off

        // Debug telemetry
        telemetry.addLine("Claw Positions: L=${shared.servoClawLeft?.position} R=${shared.servoClawRight?.position}")
        telemetry.addLine("Claw Module: ${claw.state}")
        telemetry.addLine("Arm Position: ${arm.position}")
    }
}