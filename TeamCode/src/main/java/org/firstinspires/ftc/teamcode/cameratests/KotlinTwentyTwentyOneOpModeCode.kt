package org.firstinspires.ftc.teamcode.cameratests
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Blinker
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.core.movement.old.ChassisMovementCode
import org.firstinspires.ftc.teamcode.DistanceSensorClass.RingClass
import org.firstinspires.ftc.teamcode.Grabber
import org.firstinspires.ftc.teamcode.LauncherCode.Launcher
import org.firstinspires.ftc.teamcode.LauncherCode.LauncherStates
import org.firstinspires.ftc.teamcode.LifterCode.Lifter
import kotlin.math.abs

class KotlinTwentyTwentyOneOpModeCode: LinearOpMode() {
    private var leftStickValue: Double = 0.0
    private var rightStickValue = 0.0
    var intakeMotor: DcMotor? = null
    var intakeMotor2: DcMotor? = null
    private var wiperServo: Servo? = null
    private var motorState1 = false //false = off, true = on.

    private var motorState2 = false
    private var intakeDirection = 1
    private var Control_Hub: Blinker? = null
    private var expansion_Hub_2: Blinker? = null
    private var mytimer = ElapsedTime()
    var debugTimer = ElapsedTime()
    private var launchpower = 1.0
    private enum class OperState {
        DEBUGSELECT
    }

    private enum class Intake {
        WaitingForPush, WaitingForRelease, ChangeValue, ChangeMotors, WaitingForDpadRelease, ChangeFrontValue, WaitingForDownRelease, SwitchIntakeDirection
    }

    private enum class RingWiper {
        WaitingForPushY, WaitingForReleaseY, ToggleValue, ChangeServo
    }
    override fun runOpMode() {
        val launcher = Launcher()
        var launchStates = LauncherStates.Start
        val lift = Lifter()
        val grabber = Grabber()
        val ring = RingClass()
        val chassis = ChassisMovementCode.Chassis()
        var driveOpState = ChassisMovementCode.OperState.NORMALDRIVE
        val debugOpState = OperState.DEBUGSELECT
        var intakeSwitch = Intake.WaitingForPush
        var ringWiperSwitch = RingWiper.WaitingForPushY
        Control_Hub = hardwareMap.get<Blinker>(Blinker::class.java, "Control Hub")
        expansion_Hub_2 = hardwareMap.get<Blinker>(Blinker::class.java, "Expansion Hub 2")
        lift.LiftMotor = hardwareMap.get<DcMotor>(DcMotor::class.java, "LiftMotor")
        launcher.LaunchMotor = hardwareMap.get<DcMotor>(DcMotor::class.java, "LaunchMotor")
        launcher.LaunchServo = hardwareMap.get<Servo>(Servo::class.java, "LaunchServo")
        ring.DistanceSensor = hardwareMap.get<DistanceSensor>(DistanceSensor::class.java, "Distance Sensor")
        wiperServo = hardwareMap.get<Servo>(Servo::class.java, "wiperServo")
        intakeMotor = hardwareMap.get<DcMotor>(DcMotor::class.java, "intakeMotor")
        intakeMotor2 = hardwareMap.get<DcMotor>(DcMotor::class.java, "intakeMotor2")
        grabber.GrabberLeft = hardwareMap.get<Servo>(Servo::class.java, "GrabberLeft")
        grabber.GrabberRight = hardwareMap.get<Servo>(Servo::class.java, "GrabberRight")
        chassis.imu = hardwareMap.get<BNO055IMU>(BNO055IMU::class.java, "imu")
        chassis.front_left_wheel = hardwareMap.get<DcMotor>(DcMotor::class.java, "front left wheel")
        chassis.front_right_wheel = hardwareMap.get<DcMotor>(DcMotor::class.java, "front right wheel")
        chassis.back_left_wheel = hardwareMap.get<DcMotor>(DcMotor::class.java, "back left wheel")
        chassis.back_right_wheel = hardwareMap.get<DcMotor>(DcMotor::class.java, "back right wheel")
        val parameters = BNO055IMU.Parameters() //in wrong spot--where is better?
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "BNO055IMUCalibration.json" // see the calibration sample opmode
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        chassis.imu.initialize(parameters)
        var drive: Double
        var strafe: Double
        var rotate: Double
        var movementLength = 0.0
        var increaseIntensity = 5.0
        var upWait = false
        var downWait = false
        var rightWait = false
        var leftWait = false
        var drivePreset = 0.0
        var increaseDecrease = 1.0
        var aWait = false
        var rotationGoal = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle.toDouble()
        var servoState = false

        //double timerStopTime = 0;
        waitForStart()
        launcher.Reload()
        while (opModeIsActive()) {
            this.leftStickValue = -gamepad2.left_stick_y.toDouble()
            this.rightStickValue = -gamepad2.right_stick_y.toDouble()
            lift.MoveLift(this.leftStickValue)
            telemetry.addData("drive", chassis.trueDrive)
            grabber.Toggle(gamepad2.right_bumper)
            launchpower = if (gamepad2.left_bumper) {
                0.9
            } else {
                1.0
            }
            telemetry.addData("zAngle", chassis.zAngle)
            telemetry.addData("distance sensor", ring.AveragedArray)
            //telemetry.addData("testing LauncherOn:", launcher.launcherOn);
            //telemetry.addData("Lift Power", lift.LiftPower);
            //telemetry.addData("Fork Power", lift.ForkPower);
            telemetry.update()
            chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble())
            val zAngle = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble()
            val yAngle = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle.toDouble()
            val xAngle = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle.toDouble()
            when (debugOpState) {
                OperState.DEBUGSELECT ->                     //telemetry.addData("Timer Stop Time: ", timerStopTime);
                    telemetry.update()
            }
            when (launchStates) {
                LauncherStates.Start -> {
                    if (this.gamepad2.a) {
                        launchStates = LauncherStates.ButtonPushed
                    }
                    if (this.gamepad2.b) {
                        launchStates = LauncherStates.Pressed
                    }
                }
                LauncherStates.Pressed -> if (!this.gamepad2.b) {
                    launchStates = LauncherStates.firsttimer
                }
                LauncherStates.firsttimer -> {
                    mytimer.reset()
                    launchStates = LauncherStates.Load
                }
                LauncherStates.Load -> {
                    launcher.Shoot()
                    if (mytimer.time() >= 0.25) {
                        launchStates = LauncherStates.secondtimer
                    }
                }
                LauncherStates.secondtimer -> {
                    mytimer.reset()
                    launchStates = LauncherStates.ResetPosition
                }
                LauncherStates.ResetPosition -> {
                    launcher.Reload()
                    if (mytimer.time() >= 0.25) {
                        launchStates = LauncherStates.Start
                    }
                }
                LauncherStates.ButtonPushed -> if (!this.gamepad2.a) {
                    launchStates = LauncherStates.ToggleLauncher
                }
                LauncherStates.ToggleLauncher -> {
                    launcher.LauncherToggle()
                    launchStates = LauncherStates.Start
                }
            }
            launcher.LauncherRun(launchpower)
            when (driveOpState) {
                ChassisMovementCode.OperState.NORMALDRIVE -> {
                    drive = -this.gamepad1.left_stick_y.toDouble()
                    strafe = this.gamepad1.left_stick_x.toDouble()
                    telemetry.addData("zAngle", chassis.zAngle)
                    telemetry.addData("rotate", chassis.trueRotate)
                    telemetry.addData("br", chassis.backRight)
                    telemetry.addData("bl", chassis.backLeft)
                    telemetry.addData("fr", chassis.frontRight)
                    rotate = 0.0


                    //chassis.SetMotors (drive, strafe, rotate);
                    //chassis.SetIMUMotors(drive,strafe,rotate,chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.RADIANS).firstAngle);
                    chassis.SetMotors(drive, strafe, rotate)
                    chassis.Drive()
                    chassis.Encoders()
                    chassis.SetAxisMovement()
                    if (this.gamepad1.left_trigger != 0f) {
                        chassis.ZeroEncoders()
                    }
                    if (this.gamepad1.right_trigger != 0f) {
                        driveOpState = ChassisMovementCode.OperState.NORMALROTATE
                    }
                    if (this.gamepad1.a) {
                        drivePreset = chassis.trueDrive + movementLength
                        rotationGoal = zAngle
                        driveOpState = ChassisMovementCode.OperState.FORWARD
                    }
                    if (this.gamepad1.b) {
                        drivePreset = chassis.trueStrafe + movementLength
                        rotationGoal = zAngle
                        driveOpState = ChassisMovementCode.OperState.LATERALMOVEMENT
                    }
                    if (this.gamepad1.y) {
                        driveOpState = ChassisMovementCode.OperState.SETMOVEMENTDISTANCE
                    }
                    if (this.gamepad1.x) {
                        chassis.ZeroEncoders()
                        driveOpState = ChassisMovementCode.OperState.FULLDRIVE
                    }
                }
                ChassisMovementCode.OperState.NORMALROTATE -> if (this.gamepad1.right_trigger != 0f) {
                    rotate = -this.gamepad1.right_stick_x.toDouble()
                    drive = 0.0
                    strafe = 0.0
                    chassis.SetMotors(drive, strafe, rotate)
                    chassis.Drive()
                    chassis.SetAxisMovement()
                    rotationGoal = zAngle
                    if (this.gamepad1.left_trigger != 0f) {
                        chassis.ZeroEncoders()
                    }
                    telemetry.addData("IMU rotation", chassis.imu.angularOrientation)
                    telemetry.update()
                } else {
                    driveOpState = ChassisMovementCode.OperState.NORMALDRIVE
                }
                ChassisMovementCode.OperState.FORWARD -> {
                    chassis.SetAxisMovement()
                    chassis.Encoders()
                    chassis.ForwardAndBackward(drivePreset)
                    if (this.gamepad1.right_trigger != 0f) {
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE
                    }
                    if (abs(zAngle - rotationGoal) >= 2) {
                        //chassis.SetMotors(0,0,chassis.CorrectRotation(zAngle,rotationGoal));
                        chassis.Drive()
                    }
                    if (abs(drivePreset - chassis.trueDrive) <= 0.2) {
                        chassis.front_left_wheel.power = 0.01
                        chassis.front_right_wheel.power = 0.01
                        chassis.back_right_wheel.power = 0.01
                        chassis.back_left_wheel.power = 0.01
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE
                    }
                }
                ChassisMovementCode.OperState.LATERALMOVEMENT -> {
                    chassis.SetAxisMovement()
                    chassis.Encoders()
                    chassis.LeftAndRight(drivePreset)
                    if (this.gamepad1.right_trigger != 0f) {
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE
                    }
                    if (abs(zAngle - rotationGoal) >= 2) {
                        //chassis.SetMotors(0,0,chassis.CorrectRotation(zAngle,rotationGoal));
                        chassis.Drive()
                    }
                    if (abs(drivePreset - chassis.trueStrafe) <= 0.2) {
                        chassis.front_left_wheel.power = 0.01
                        chassis.front_right_wheel.power = 0.01
                        chassis.back_right_wheel.power = 0.01
                        chassis.back_left_wheel.power = 0.01
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE
                    }
                }
                ChassisMovementCode.OperState.SETMOVEMENTDISTANCE -> {
                    telemetry.addLine("Press up and down on the d-pad to increase movement per press")
                    telemetry.addLine("Press right and left on the d-pad to increase or decrease the increased amount added")
                    telemetry.addData("Current movement per press", movementLength)
                    telemetry.addData("Amount increased per increase", increaseIntensity)
                    telemetry.update()
                    if (upWait and !this.gamepad1.dpad_up) {
                        movementLength += increaseIntensity
                        upWait = false
                    }
                    if (!this.gamepad1.dpad_down and downWait) {
                        movementLength -= increaseIntensity
                        downWait = false
                    }
                    if (!this.gamepad1.dpad_right and rightWait) {
                        increaseIntensity++
                        rightWait = false
                    }
                    if (!this.gamepad1.dpad_left and leftWait) {
                        increaseIntensity--
                        leftWait = false
                    }
                    if (this.gamepad1.dpad_up) {
                        upWait = true
                    }
                    if (this.gamepad1.dpad_down) {
                        downWait = true
                    }
                    if (this.gamepad1.dpad_right) {
                        rightWait = true
                    }
                    if (this.gamepad1.dpad_left) {
                        leftWait = true
                    }
                    if (this.gamepad1.left_trigger != 0f) {
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE
                    }
                }
                ChassisMovementCode.OperState.SETMOTORMULTIPLE -> {
                    telemetry.addLine("Press A to change increase/decrease")
                    if (increaseDecrease == 1.0) {
                        telemetry.addLine("INCREASING")
                    }
                    if (increaseDecrease == -1.0) {
                        telemetry.addLine("DECREASING")
                    }
                    telemetry.addLine("Press right dpad to change FR wheel multiplier")
                    telemetry.addData("FR wheel multiplier: ", chassis.frontRightMultiplier)
                    telemetry.addLine("Press right dpad to change FL wheel multiplier")
                    telemetry.addData("FL wheel multiplier: ", chassis.frontLeftMultiplier)
                    telemetry.addLine("Press right dpad to change BR wheel multiplier")
                    telemetry.addData("BR wheel multiplier: ", chassis.backRightMultiplier)
                    telemetry.addLine("Press right dpad to change BL wheel multiplier")
                    telemetry.addData("BL wheel multiplier: ", chassis.backLeftMultiplier)
                    telemetry.update()
                    if ((increaseDecrease == 1.0) and !this.gamepad1.a and aWait) {
                        increaseDecrease = -1.0
                        aWait = false
                    }
                    if ((increaseDecrease == -1.0) and !this.gamepad1.a and aWait) {
                        increaseDecrease = 1.0
                        aWait = false
                    }
                    if (upWait and !this.gamepad1.dpad_up) {
                        chassis.frontLeftMultiplier = chassis.frontLeftMultiplier + 0.01 * increaseDecrease
                        upWait = false
                    }
                    if (!this.gamepad1.dpad_down and downWait) {
                        chassis.backRightMultiplier = chassis.backRightMultiplier + 0.01 * increaseDecrease
                        downWait = false
                    }
                    if (!this.gamepad1.dpad_right and rightWait) {
                        chassis.frontRightMultiplier = chassis.frontRightMultiplier + 0.01 * increaseDecrease
                        rightWait = false
                    }
                    if (!this.gamepad1.dpad_left and leftWait) {
                        chassis.backLeftMultiplier = chassis.backLeftMultiplier + 0.01 * increaseDecrease
                        leftWait = false
                    }
                    if (this.gamepad1.dpad_up) {
                        upWait = true
                    }
                    if (this.gamepad1.dpad_down) {
                        downWait = true
                    }
                    if (this.gamepad1.dpad_right) {
                        rightWait = true
                    }
                    if (this.gamepad1.dpad_left) {
                        leftWait = true
                    }
                    if (this.gamepad1.a) {
                        aWait = true
                    }
                    if (this.gamepad1.left_trigger != 0f) {
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE
                    }
                }
                ChassisMovementCode.OperState.FULLDRIVE -> {
                    telemetry.addData("X", chassis.trueX)
                    telemetry.addData("Y", chassis.trueY)
                    telemetry.addData("Preset X", chassis.presetX)
                    telemetry.addData("Preset Y", chassis.presetY)
                    telemetry.addData("True Drive", chassis.trueDrive)
                    telemetry.addData("True Strafe", chassis.trueStrafe)
                    telemetry.addData("true rotate", chassis.trueRotate)
                    telemetry.addData("clear drive", chassis.clearDrive)
                    telemetry.addData("clear strafe", chassis.clearStrafe)
                    telemetry.addData("clear rotate", chassis.clearRotate / chassis.tau)
                    telemetry.addData("total rotate", chassis.trueRotate + chassis.clearRotate)
                    chassis.SetAxisMovement()
                    drive = -this.gamepad1.left_stick_y.toDouble()
                    strafe = this.gamepad1.left_stick_x.toDouble()
                    rotate = -this.gamepad1.right_stick_x.toDouble()
                    telemetry.addData("drive", drive)
                    telemetry.addData("strafe", strafe)
                    telemetry.addData("rotate", rotate)
                    chassis.SetMotors(drive, strafe, rotate)
                    chassis.Drive()
                    chassis.SetTrueAxis()
                }
                ChassisMovementCode.OperState.ABSOLUTEDRIVE -> {
                    drive = -this.gamepad1.left_stick_y.toDouble()
                    strafe = -this.gamepad1.left_stick_x.toDouble()
                    telemetry.addData("zAngle", chassis.zAngle)
                    telemetry.addData("rotate", chassis.trueRotate)
                    telemetry.addData("br", chassis.backRight)
                    telemetry.addData("bl", chassis.backLeft)
                    telemetry.addData("fr", chassis.frontRight)
                    rotate = 0.0
                    chassis.SetMotors(drive, strafe, rotate)
                    chassis.Drive()
                    chassis.Encoders()
                    chassis.SetAxisMovement()
                }
                else -> {
                }
            }
            when (intakeSwitch) {
                Intake.WaitingForPush -> intakeSwitch = when {
                    gamepad2.x -> {
                        Intake.WaitingForRelease
                    }
                    gamepad2.dpad_up -> {
                        Intake.WaitingForDpadRelease
                    }
                    gamepad2.dpad_down -> {
                        Intake.WaitingForDownRelease
                    }
                    else -> {
                        Intake.ChangeMotors
                    }
                }
                Intake.WaitingForRelease -> if (!gamepad2.x) {
                    intakeSwitch = Intake.ChangeValue
                }
                Intake.ChangeValue -> {
                    motorState2 = !motorState2
                    motorState1 = !motorState1
                    intakeSwitch = Intake.ChangeMotors
                }
                Intake.WaitingForDpadRelease -> if (!gamepad2.dpad_up) {
                    intakeSwitch = Intake.ChangeFrontValue
                }
                Intake.ChangeFrontValue -> {
                    motorState1 = !motorState1
                    intakeSwitch = Intake.ChangeMotors
                }
                Intake.WaitingForDownRelease -> if (!gamepad2.dpad_up) {
                    intakeSwitch = Intake.SwitchIntakeDirection
                }
                Intake.SwitchIntakeDirection -> {
                    intakeDirection *= -1
                    intakeSwitch = Intake.ChangeMotors
                }
                Intake.ChangeMotors -> {
                    if (motorState2) {
                        intakeMotor2?.power = 1.0
                    } else if (!motorState2) {
                        intakeMotor2?.power = 0.0
                    }
                    if (motorState1) {
                        intakeMotor?.power = (-1 * intakeDirection).toDouble()
                    } else if (!motorState1) {
                        intakeMotor?.power = 0.0
                    }
                    intakeSwitch = Intake.WaitingForPush
                }
            }
            when (ringWiperSwitch) {
                RingWiper.WaitingForPushY -> ringWiperSwitch = if (gamepad2.y) {
                    RingWiper.WaitingForReleaseY
                } else {
                    RingWiper.ChangeServo
                }
                RingWiper.WaitingForReleaseY -> if (!gamepad2.y) {
                    ringWiperSwitch = RingWiper.ToggleValue
                }
                RingWiper.ToggleValue -> {
                    servoState = !servoState
                    ringWiperSwitch = RingWiper.ChangeServo
                }
                RingWiper.ChangeServo -> {
                    if (servoState) {
                        wiperServo?.position = 0.75
                    } else if (!servoState) {
                        wiperServo?.position = .95
                    }
                    ringWiperSwitch = RingWiper.WaitingForPushY
                }
            }
            telemetry.addData("Wiper state: ", ringWiperSwitch)
            telemetry.addData("Wiper position", wiperServo?.position)
            telemetry.addData("is Y pressed", gamepad2.y)
            telemetry.addData("LeftGrabber", grabber.GrabberLeft.position)
            telemetry.addData("RightGrabber", grabber.GrabberRight.position)
        }
    }

}