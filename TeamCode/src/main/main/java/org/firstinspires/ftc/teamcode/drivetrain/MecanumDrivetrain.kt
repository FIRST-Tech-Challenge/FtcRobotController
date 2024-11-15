package org.firstinspires.ftc.teamcode.drivetrain

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MecanumKinematics.WheelIncrements
import com.acmerobotics.roadrunner.MecanumKinematics.WheelVelocities
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryActionFactory
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnActionFactory
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.ftc.LazyImu
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.now
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive.Companion.PARAMS
import org.firstinspires.ftc.teamcode.drivetrain.messages.MecanumLocalizerInputsMessage
import org.firstinspires.ftc.teamcode.drivetrain.messages.PoseMessage
import org.firstinspires.ftc.teamcode.helpers.getMotor
import java.lang.Math.PI
import java.util.LinkedList
import kotlin.math.max

interface Drivetrain {
    var velocityVector: PoseVelocity2d
    fun updatePoseEstimate(): PoseVelocity2d
    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder
    val poseHistory: LinkedList<Pose2d>
}

class MecanumDrivetrain(val config: DrivetrainConfig, var pose: Pose2d = Pose2d(0.0, 0.0, 0.0)) :
    Drivetrain {
    private val estimatedPoseWriter = DownsampledWriter("Estimated Pose", 50000000)
    private val holonomics = with(config.params) {
        HolonomicController(
            axialPosGain = positionalGain.axial,
            lateralPosGain = positionalGain.lateral,
            headingGain = positionalGain.heading,
            axialVelGain = velocityGain.axial,
            lateralVelGain = velocityGain.lateral,
            headingVelGain = velocityGain.heading,
        )
    }
    val feedForward = with(config.params) {
        MotorFeedforward(kS = feedForward.kS, kV = scaledKV, kA = scaledKA)
    }
    override val poseHistory: LinkedList<Pose2d> = LinkedList<Pose2d>()
    override var velocityVector: PoseVelocity2d = PoseVelocity2d(
        linearVel = Vector2d(x = 0.0, y = 0.0), angVel = 0.0
    )
        set(value) {
            val wheelVels: WheelVelocities<Time> = MecanumKinematics(1.0).inverse<Time>(
                PoseVelocity2dDual.constant<Time>(value, 1)
            )

            var maxPowerMag = 1.0

            for (power in wheelVels.all()) {
                maxPowerMag = max(maxPowerMag, power.value())
            }

            with(config.motors) {
                leftFront.motor.power = wheelVels.leftFront[0] / maxPowerMag
                leftBack.motor.power = wheelVels.leftBack[0] / maxPowerMag
                rightBack.motor.power = wheelVels.rightBack[0] / maxPowerMag
                rightFront.motor.power = wheelVels.rightFront[0] / maxPowerMag
            }
            field = value
        }

    override fun updatePoseEstimate(): PoseVelocity2d {
        val twist = config.localizer.update()
        pose = pose.plus(twist.value())

        poseHistory.add(pose)
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(pose))

        return twist.velocity().value()
    }

    override fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return with(config.params) {
            TrajectoryActionBuilder(
                TurnActionFactory { turn: TimeTurn -> TurnAction(turn) },
                TrajectoryActionFactory { timeTrajectory: TimeTrajectory ->
                    FollowTrajectoryAction(timeTrajectory)
                },
                TrajectoryBuilderParams(
                    1e-6,
                    ProfileParams(
                        dispResolution = 0.25, angResolution = 0.1, angSamplingEps = 1e-2
                    ),
                ),
                beginPose = beginPose,
                beginEndVel = 0.0,
                baseTurnConstraints = turnConstraints,
                baseVelConstraint = velocityConstraint,
                baseAccelConstraint = accelerationConstraint,
            )
        }
    }

    inner class TurnAction(val turn: TimeTurn) : Action {
        private val turnStartTime by lazy { now() }
        override fun run(p: TelemetryPacket): Boolean {
            val timeTaken = now() - turnStartTime

            if (timeTaken >= turn.duration) {
                // stop turning, time is up!
                config.motors.stop()
                return false
            }

            val target = turn[timeTaken]

            val command = holonomics.compute(
                target, pose, this@MecanumDrivetrain.updatePoseEstimate()
            )

            val wheelVelocities = with(config.params) {
                kinematics.inverse<Time>(command)
            }

            val availableVoltage = with(config) {
                voltageSensor.voltage
            }

            with(config.motors) {
                leftFront.motor.power =
                    feedForward.compute(wheelVelocities.leftFront) / availableVoltage
                leftBack.motor.power =
                    feedForward.compute(wheelVelocities.leftBack) / availableVoltage
                rightFront.motor.power =
                    feedForward.compute(wheelVelocities.rightFront) / availableVoltage
                rightBack.motor.power =
                    feedForward.compute(wheelVelocities.rightBack) / availableVoltage
            }

            return true
        }

    }

    inner class FollowTrajectoryAction(val timeTrajectory: TimeTrajectory) : Action {
        private val followStartTime by lazy { now() }

        override fun run(p: TelemetryPacket): Boolean {
            val timeTaken = now() - followStartTime

            if (timeTaken >= timeTrajectory.duration) {
                config.motors.stop()
                return false
            }

            val target = timeTrajectory[timeTaken]

            val command = holonomics.compute(target, pose, updatePoseEstimate())

            val wheelVelocities = with(config.params) {
                kinematics.inverse<Time>(command)
            }

            val availableVoltage = config.voltageSensor.voltage

            with(config.motors) {
                leftFront.motor.power =
                    feedForward.compute(wheelVelocities.leftFront) / availableVoltage
                leftBack.motor.power =
                    feedForward.compute(wheelVelocities.leftBack) / availableVoltage
                rightFront.motor.power =
                    feedForward.compute(wheelVelocities.rightFront) / availableVoltage
                rightBack.motor.power =
                    feedForward.compute(wheelVelocities.rightBack) / availableVoltage
            }

            return true
        }
    }


    init {
        with(config.motors) {
            with(leftFront) { motor.zeroPowerBehavior = zeroPowerBehavior }
            with(leftBack) { motor.zeroPowerBehavior = zeroPowerBehavior }
            with(rightBack) { motor.zeroPowerBehavior = zeroPowerBehavior }
            with(rightFront) { motor.zeroPowerBehavior = zeroPowerBehavior }
        }
    }
}

data class DriveMotors(
    val leftFront: DriveMotor,
    val rightFront: DriveMotor,
    val leftBack: DriveMotor,
    val rightBack: DriveMotor,
)

fun DriveMotors.stop() {
    leftFront.stop()
    leftBack.stop()
    rightFront.stop()
    rightBack.stop()
}

fun DriveMotor.stop() {
    motor.power = 0.0
}

data class DriveMotor (
    val motor: DcMotorEx,
    val direction: Direction = FORWARD,
    val zeroPowerBehavior: ZeroPowerBehavior = BRAKE,
    val encoder: Encoder = OverflowEncoder(RawEncoder(motor))
)

data class DrivetrainTuningParameters(
    val imuOrientation: ImuOrientationOnRobot,
    var inchesPerTick: Double,
    var lateralInchesPerTick: Double,
    var trackWidthTicks: Double,
    var maxAngularVelocity: Double = PI,
    var maxAngularAcceleration: Double = PI,
    var maxWheelVelocity: Double = 50.0,
    var minProfileAcceleration: Double = -30.0,
    var maxProfileAcceleration: Double = 50.0,
    var positionalGain: Gain = Gain(),
    var velocityGain: Gain = Gain(),
    var feedForward: FeedForward = FeedForward(),
)

data class FeedForward(
    var kS: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0,
)

val DrivetrainTuningParameters.scaledKA: Double get() = feedForward.kA / inchesPerTick
val DrivetrainTuningParameters.scaledKV: Double get() = feedForward.kV / inchesPerTick

data class Gain(
    var axial: Double = 0.0,
    var lateral: Double = 0.0,
    var heading: Double = 0.0,
)

val DrivetrainTuningParameters.kinematics: MecanumKinematics
    get() {
        return MecanumKinematics(
            trackWidth = this.inchesPerTick * this.trackWidthTicks,
            lateralMultiplier = this.inchesPerTick / this.lateralInchesPerTick
        )
    }

val DrivetrainTuningParameters.turnConstraints: TurnConstraints
    get() = TurnConstraints(
        maxAngVel = maxAngularVelocity,
        minAngAccel = -maxAngularAcceleration,
        maxAngAccel = maxAngularAcceleration
    )

val DrivetrainTuningParameters.velocityConstraint: VelConstraint
    get() = MinVelConstraint(
        listOf(
            kinematics.WheelVelConstraint(maxWheelVelocity),
            AngularVelConstraint(maxAngularVelocity)
        )
    )

val DrivetrainTuningParameters.accelerationConstraint: AccelConstraint
    get() = ProfileAccelConstraint(
        minAccel = minProfileAcceleration, maxAccel = maxProfileAcceleration
    )

data class DrivetrainConfig(
    val motors: DriveMotors,
    val params: DrivetrainTuningParameters,
    val imu: IMU,
    val voltageSensor: VoltageSensor,
    val localizer: Localizer = DriveLocalizer(motors, imu, params),
)

data class BotDimensions(
    val trackWidth: Double = 16.0,
    val wheelBase: Double = 16.0,
    val height: Double = 16.0,
    val weight: Double = 5.0,
)

fun HardwareMap.imu(
    name: String = "imu", orientation: ImuOrientationOnRobot = RevHubOrientationOnRobot(
        LogoFacingDirection.UP, UsbFacingDirection.BACKWARD
    )
): IMU {
    return LazyImu(this, name, orientation).imu
}

val HardwareMap.nextVoltageSensor: VoltageSensor get() = voltageSensor.iterator().next()

fun HardwareMap.driveMotors(
    leftFront: String = "left_front",
    leftFrontDirection: Direction = REVERSE,
    rightFront: String = "right_front",
    rightFrontDirection: Direction = FORWARD,
    leftBack: String = "left_back",
    leftBackDirection: Direction = REVERSE,
    rightBack: String = "right_back",
    rightBackDirection: Direction = REVERSE,
): DriveMotors {
    return DriveMotors(
        leftFront = DriveMotor(this.getMotor(leftFront), leftFrontDirection),
        rightFront = DriveMotor(this.getMotor(rightFront), rightFrontDirection),
        leftBack = DriveMotor(this.getMotor(leftBack), leftBackDirection),
        rightBack = DriveMotor(this.getMotor(rightBack), rightBackDirection)
    )
}


val DriveMotor.positionAndVelocity: PositionVelocityPair get() = this.encoder.getPositionAndVelocity()

val LazyImu.imu: IMU
    get() = this.get()

val IMU.heading: Rotation2d
    get() = Rotation2d.exp(
        this.robotYawPitchRollAngles.getYaw(
            RADIANS
        )
    )

class DriveLocalizer(val motors: DriveMotors, val imu: IMU, val params: DrivetrainTuningParameters) :
    Localizer {
    private var lastLeftFrontPos = 0.0
    private var lastLeftBackPos = 0.0
    private var lastRightBackPos = 0.0
    private var lastRightFrontPos = 0.0
    private var lastHeading: Rotation2d
    private var initialized = false

    init {
        lastHeading = imu.heading
    }

    override fun update(): Twist2dDual<Time> {
        write(
            "MECANUM_LOCALIZER_INPUTS", MecanumLocalizerInputsMessage(
                leftFront = motors.leftFront.positionAndVelocity,
                leftBack = motors.leftBack.positionAndVelocity,
                rightBack = motors.rightBack.positionAndVelocity,
                rightFront = motors.rightFront.positionAndVelocity,
                angles = imu.robotYawPitchRollAngles
            )
        )

        val heading = imu.heading

        if (!initialized) {
            initialized = true

            updateLastPosition()

            return Twist2dDual<Time>(
                Vector2dDual.constant<Time>(Vector2d(0.0, 0.0), 2), DualNum.constant<Time>(0.0, 2)
            )
        }

        val headingDelta = heading.minus(lastHeading)
        val twist = params.kinematics.forward<Time>(
            WheelIncrements<Time>(
                DualNum<Time>(
                    doubleArrayOf(
                        (motors.leftFront.positionAndVelocity.position - lastLeftFrontPos),
                        motors.leftFront.positionAndVelocity.velocity.toDouble(),
                    )
                ).times(PARAMS.inPerTick), DualNum<Time>(
                    doubleArrayOf(
                        (motors.leftBack.positionAndVelocity.position - lastLeftBackPos),
                        motors.leftBack.positionAndVelocity.velocity.toDouble(),
                    )
                ).times(PARAMS.inPerTick), DualNum<Time>(
                    doubleArrayOf(
                        (motors.rightBack.positionAndVelocity.position - lastRightBackPos),
                        motors.rightBack.positionAndVelocity.velocity.toDouble(),
                    )
                ).times(PARAMS.inPerTick), DualNum<Time>(
                    doubleArrayOf(
                        (motors.rightFront.positionAndVelocity.position - lastRightFrontPos),
                        motors.rightFront.positionAndVelocity.velocity.toDouble(),
                    )
                ).times(PARAMS.inPerTick)
            )
        )

        updateLastPosition()

        return Twist2dDual<Time>(
            twist.line, DualNum.cons<Time>(headingDelta, twist.angle.drop(1))
        )
    }

    fun updateLastPosition() {
        lastLeftFrontPos = motors.leftFront.positionAndVelocity.position.toDouble()
        lastLeftBackPos = motors.leftBack.positionAndVelocity.position.toDouble()
        lastRightBackPos = motors.rightBack.positionAndVelocity.position.toDouble()
        lastRightFrontPos = motors.rightFront.positionAndVelocity.position.toDouble()

        lastHeading = imu.heading
    }
}
