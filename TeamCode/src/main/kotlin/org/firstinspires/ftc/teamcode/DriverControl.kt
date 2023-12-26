package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time
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
import org.firstinspires.ftc.teamcode.botmodule.Intake
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
    private var useBotRelative = true
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
        val pv = PoseVelocity2d(
                if (useBotRelative) Vector2d(
                        driveRelativeX,
                        driveRelativeY
                ) else inputVector,
                -gamepad1.right_stick_x.toDouble()
        )
        // +X = forward, +Y = left
//        drive.setDrivePowers(pv)
        val wheelVels = MecanumKinematics(1.0).inverse<Time>(PoseVelocity2dDual.constant(pv, 1));

        shared.motorLeftFront.power = wheelVels.leftFront[0] / powerModifier
        shared.motorLeftBack.power = wheelVels.leftBack[0] / powerModifier
        shared.motorRightBack.power = wheelVels.rightBack[0] / powerModifier
        shared.motorRightFront.power = wheelVels.rightFront[0] / powerModifier

//        Actions.run

        telemetry.addLine("Gyro Yaw: " + shared.imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES))
        telemetry.addLine("Input Yaw: " + if (inputVector.x > 0.05 && inputVector.y > 0.05) inputTheta * 180.0 / PI else 0.0)
//        telemetry.addLine("Yaw Difference (bot - input): " + )
    }

    /**
     * Set up the robot
     */
    override fun init() {
        //set intake down


//        val setter = DriverControl2::tagCamera.setter
        shared = BotShared(this)
        shared.drive = MecanumDrive(hardwareMap, initialPose)
        gamepadyn = Gamepadyn(InputSystemFtc(this), true,
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

        // toggle driver-relative controls
        gamepadyn.players[0].getEventDigital(TOGGLE_DRIVER_RELATIVITY)!!.addListener { if (it.digitalData) useBotRelative = !useBotRelative }

        val intake = shared.intake
      /**  if (intake != null) {
            // toggle intake height
            gamepadyn.players[0].getEventDigital(TOGGLE_INTAKE_HEIGHT)!!.addListener { if (intake.raised) intake.lower() else intake.raise() }
        } else {
            telemetry.addLine("WARNING: Safeguard triggered (intake not present)");
        }**/
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
        updateOuttake()
        updateIntake()
        updateTrussHang()
        updateDroneLaunch()

        // Most input values are [-1.0, 1.0]

        telemetry.addLine("Left Stick X: ${gamepad1.left_stick_x}")
        telemetry.addLine("Left Stick Y: ${gamepad1.left_stick_y}")
        telemetry.addLine("Bot Relativity: ${if (useBotRelative) "EN" else "DIS"}ABLED")
        telemetry.addLine("Delta Time: $deltaTime")
        telemetry.update()

        gamepadyn.update()

        shared.update()
    }



    /**
     * Update the linear slide
     */
    private fun updateOuttake() {
        val outtake = shared.outtake

        //Close Claws
        if (gamepad1.y || gamepad2.left_bumper){
            outtake?.closeClaws()
        }

        //Place Left Pixel
        if (gamepad1.x){
            outtake?.openLeftClaw()
        }
        //Place Right Pixel
        if (gamepad1.b){
            outtake?.openRightClaw()
        }
        //Place Both and go down
        if (gamepad1.a ||gamepad2.right_bumper){
            outtake?.place()
        }
        //Increase Count by 1
        if (gamepad1.dpad_up || gamepad2.dpad_up){
            outtake?.increaseRow()
        }

        //Decrease Count by 1
        if (gamepad1.dpad_down|| gamepad2.dpad_down){
            outtake?.decreaseRow()
        }

        //Reset Count
        if (gamepad1.dpad_right||gamepad2.dpad_left){
            outtake?.resetRow()
        }

        //Execute Count
        if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger> 0.1){
            outtake?.runToRow()
        }
    }

    /**
     * Update the intake mechanisms (spinner, lift)
     */


    private fun updateIntake() {
        val intake = shared.intake
        //Lift Intake
        if (gamepad2.y){
            intake?.raise()
        }

        if (gamepad2.a){
            intake?.lower()
        }


        // spinner
        if (gamepad1.left_trigger>0.1 || gamepad2.left_trigger>0.1){
            if (gamepad1.left_trigger+gamepad2.left_trigger>1)
            intake?.spin(1.0.toDouble())
            else
                intake?.spin((gamepad1.left_trigger+gamepad2.left_trigger).toDouble())

        } else {
            intake?.spin(0.0.toDouble())
        }

    }

    /**
     * Handle controls for the truss pulley
     */
    private fun updateTrussHang() {
        val trussHang = shared.hang

        if(gamepad2.x){
            trussHang?.raiseArms()
        }

        if(abs(gamepad2.right_stick_y) > 0.1){
            trussHang?.lift(gamepad2.right_stick_y.toDouble())
        }


    }

    //Everything for drone launch

    private fun updateDroneLaunch() {
        val droneLaunch = shared.droneLauncher

        if (gamepad2.b){
            droneLaunch?.launch()
        }
    }

}