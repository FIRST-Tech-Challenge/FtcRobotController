package org.firstinspires.ftc.teamcode.intothedeep

import androidx.core.math.MathUtils.clamp
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.TestingTeleop
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainConfig
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainTuningParameters
import org.firstinspires.ftc.teamcode.drivetrain.FeedForward
import org.firstinspires.ftc.teamcode.drivetrain.Gain
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrivetrain
import org.firstinspires.ftc.teamcode.drivetrain.driveMotors
import org.firstinspires.ftc.teamcode.drivetrain.imu
import org.firstinspires.ftc.teamcode.drivetrain.nextVoltageSensor
import org.firstinspires.ftc.teamcode.helpers.PowerVector
import org.firstinspires.ftc.teamcode.helpers.getMotor
import org.firstinspires.ftc.teamcode.helpers.getServo
import org.firstinspires.ftc.teamcode.helpers.mecanumDrive
import org.firstinspires.ftc.teamcode.robot.Robot
import kotlin.math.PI


@Config
class Dave(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    startingPose: Pose2d = Pose2d(Vector2d(0.0, 0.0), 0.0),
) : Robot {
    override val drivetrain by lazy {
        MecanumDrivetrain(
            DrivetrainConfig(
                motors = hardwareMap.driveMotors(
                    leftFront = "left_front_motor",
                    leftFrontDirection = FORWARD,
                    rightFront = "right_front_motor",
                    rightFrontDirection = REVERSE,
                    leftBack = "left_back_motor",
                    leftBackDirection = REVERSE,
                    rightBack = "right_back_motor",
                    rightBackDirection = FORWARD,
                ),
                params = drivetrainParams,
                imu = hardwareMap.imu(orientation = drivetrainParams.imuOrientation),
                voltageSensor = hardwareMap.nextVoltageSensor,
            ), pose = startingPose
        )
    }
    override var position = startingPose
    val omni by lazy { hardwareMap.mecanumDrive() }
    override var velocity
        get() = drivetrain.velocityVector
        set(value) {
            omni.powerVector = PowerVector(value.linearVel.x, value.linearVel.y, value.angVel)
        }
    override val positionHistory = mutableListOf(position.copy())
    override val velocityHistory = mutableListOf(velocity.copy())
    val arm by lazy { hardwareMap.getMotor("arm_motor") }
    val extension by lazy { hardwareMap.getMotor("extension_motor") }
    val pincer by lazy { hardwareMap.getServo("pincer") }
    var armPosition = 0
        set(value) {
            var target = clamp(value, minimumArmPosition, maximumArmPosition)
            arm.targetPosition = target
            arm.velocity = armVelocity
            arm.mode = RUN_TO_POSITION
            field = target
        }
    var armExtension = 0
        set(value) {
            var target = clamp(value, minimumArmExtension, maximumArmExtension)
            extension.targetPosition = target
            extension.velocity = extensionVelocity
            extension.mode = RUN_TO_POSITION
            field = target
        }
    var pincerPosition = 0.0
        set(value) {
            var target = clamp(value, -1.0, 1.0)
            pincer.position = target
            field = target
        }

    companion object {
        @JvmField
        var drivetrainParams: DrivetrainTuningParameters = DrivetrainTuningParameters(
            imuOrientation = RevHubOrientationOnRobot(
                LogoFacingDirection.UP, UsbFacingDirection.BACKWARD
            ),
            inchesPerTick = 1.0,
            lateralInchesPerTick = 1.0,
            trackWidthTicks = 0.0,
            maxAngularVelocity = PI,
            maxAngularAcceleration = PI,
            maxWheelVelocity = 50.0,
            minProfileAcceleration = -30.0,
            maxProfileAcceleration = 50.0,
            positionalGain = Gain(0.0, 0.0, 0.0),
            velocityGain = Gain(0.0, 0.0, 0.0),
            feedForward = FeedForward(),
        )

        @JvmField
        var lowerAscentBarHeight: Int = 100

        @JvmField
        var higherAscentBarHeight: Int = 200

        @JvmField
        var lowerSpecimenBarHeight: Int = 100

        @JvmField
        var higherSpecimenBarHeight: Int = 200

        @JvmField
        var lowerBasketHeight: Int = 100

        @JvmField
        var upperBasketHeight: Int = 200

        @JvmField
        var armAngleBasket: Double = 0.0

        @JvmField
        var armAngleSpecimen: Double = 0.0

        @JvmField
        var armAngleHang: Double = 0.0

        @JvmField
        var armGearRatio = 20

        @JvmField
        var armPositionModifier = 1

        @JvmField
        var maximumArmPosition = 700

        @JvmField
        var minimumArmPosition = 0

        @JvmField
        var maximumArmExtension = 600

        @JvmField
        var minimumArmExtension = 0

        @JvmField
        var extensionRate = 100

        @JvmField
        var armVelocity = 1000.0

        @JvmField
        var extensionVelocity = 1000.0

        @JvmField
        var pincerClosePosition = 0.0

        @JvmField
        var pincerOpenPosition = 1.0
    }

    fun update() {
        telemetry.motorPosition("Arm", arm)
        telemetry.motorPosition("Extension", extension)
        positionHistory.add(position)
        velocityHistory.add(velocity)
    }

    fun armUp() {
        armPosition += armGearRatio * armPositionModifier
    }

    fun armDown() {
        armPosition -= armGearRatio * armPositionModifier
    }

    fun extend() {
        armExtension += extensionRate
    }

    fun retract() {
        armExtension -= extensionRate
    }

    fun retractFullPower() {
        extension.mode = RUN_WITHOUT_ENCODER
        extension.power = -1.0
    }

    fun resetExtension() {
        extension.power = 0.0
        extension.mode = RUN_TO_POSITION
        extension.velocity = 2000.0
        extension.targetPosition = extension.currentPosition
    }

    fun closePincer() {
        pincerPosition = pincerClosePosition
    }

    fun openPincer() {
        pincerPosition = pincerOpenPosition
    }

    fun initialize() {
        arm.zeroPowerBehavior = BRAKE
        arm.mode = STOP_AND_RESET_ENCODER
        arm.targetPosition = arm.currentPosition
        arm.direction = FORWARD
        arm.velocity = 1000.0

        extension.mode = STOP_AND_RESET_ENCODER
        extension.zeroPowerBehavior = BRAKE
        extension.targetPosition = extension.currentPosition
        extension.direction = REVERSE
        extension.velocity = 1000.0

        arm.mode = RUN_TO_POSITION
        extension.mode = RUN_TO_POSITION
    }
}

fun Telemetry.motorPosition(name: String, motor: DcMotorEx) {
    this.addData("$name target", motor.targetPosition)
    this.addData("$name Position", motor.currentPosition)
    this.addData("$name velocity", motor.velocity)
    this.addData("$name power", motor.power)
}


data class Extension(val motor: DcMotorEx, var maximumExtension: Int, var minimumExtension: Int) {
    inner class FullExtension() : Action {
        override fun run(p: TelemetryPacket): Boolean {
            TODO("Not yet implemented")
        }
    }

    inner class FullRetraction() : Action {
        override fun run(p: TelemetryPacket): Boolean {
            TODO("Not yet implemented")
        }
    }

    inner class ExtendTo(val ticks: Int) : Action {
        override fun run(p: TelemetryPacket): Boolean {
            TODO("Not yet implemented")
        }
    }
}

data class ArmPivot(val motor: DcMotorEx, var maximumAngle: Int, var minimumAngle: Int) {
    inner class Down() : Action {
        override fun run(p: TelemetryPacket): Boolean {
            TODO("Not yet implemented")
        }
    }

    inner class Up() : Action {
        override fun run(p: TelemetryPacket): Boolean {
            TODO("Not yet implemented")
        }
    }

    inner class To(val radians: Double) : Action {
        override fun run(p: TelemetryPacket): Boolean {
            TODO("Not yet implemented")
        }
    }
}

data class Pincer(val servo: ServoImplEx) {
    inner class Open() : Action {
        override fun run(p: TelemetryPacket): Boolean {
            TODO("Not yet implemented")
        }
    }

    inner class Close() : Action {
        override fun run(p: TelemetryPacket): Boolean {
            TODO("Not yet implemented")
        }
    }
}