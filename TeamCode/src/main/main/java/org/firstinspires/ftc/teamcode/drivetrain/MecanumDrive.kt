package org.firstinspires.ftc.teamcode.drivetrain

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MecanumKinematics.WheelIncrements
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.MecanumKinematics.WheelVelocities
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
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.ftc.throwIfModulesAreOutdated
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.drivetrain.Drawing.drawRobot
import org.firstinspires.ftc.teamcode.drivetrain.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.drivetrain.messages.MecanumCommandMessage
import org.firstinspires.ftc.teamcode.drivetrain.messages.MecanumLocalizerInputsMessage
import org.firstinspires.ftc.teamcode.drivetrain.messages.PoseMessage
import java.util.Arrays
import java.util.LinkedList
import kotlin.math.ceil
import kotlin.math.max

// copy from Road Runner docs
class MecanumDrive(hardwareMap: HardwareMap, pose: Pose2d) {
    class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        val logoFacingDirection: RevHubOrientationOnRobot.LogoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP
        val usbFacingDirection: UsbFacingDirection = UsbFacingDirection.FORWARD

        val inPerTick: Double = 1.0
        val lateralInPerTick: Double = inPerTick
        val trackWidthTicks: Double = 0.0

        val kS: Double = 0.0
        val kV: Double = 0.0
        val kA: Double = 0.0

        val maxWheelVel: Double = 50.0
        val minProfileAccel: Double = -30.0
        val maxProfileAccel: Double = 50.0

        val maxAngVel: Double = Math.PI // shared with path
        val maxAngAccel: Double = Math.PI

        val axialGain: Double = 0.0
        val lateralGain: Double = 0.0
        val headingGain: Double = 0.0

        val axialVelGain: Double = 0.0
        val lateralVelGain: Double = 0.0
        val headingVelGain: Double = 0.0
    }

    val kinematics: MecanumKinematics = MecanumKinematics(
        PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick
    )

    val defaultTurnConstraints: TurnConstraints = TurnConstraints(
        PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel
    )
    val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf<VelConstraint>(
            kinematics.WheelVelConstraint(PARAMS.maxWheelVel),
            AngularVelConstraint(PARAMS.maxAngVel)
        )
    )
    val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    val leftFront: DcMotorEx
    val leftBack: DcMotorEx
    val rightBack: DcMotorEx
    val rightFront: DcMotorEx

    val voltageSensor: VoltageSensor

    val lazyImu: LazyImu

    val localizer: Localizer
    var pose: Pose2d

    private val poseHistory = LinkedList<Pose2d>()

    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50000000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50000000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50000000)
    private val mecanumCommandWriter = DownsampledWriter("MECANUM_COMMAND", 50000000)

    inner class DriveLocalizer : Localizer {
        val leftFront: Encoder = OverflowEncoder(RawEncoder(this@MecanumDrive.leftFront))
        val leftBack: Encoder = OverflowEncoder(RawEncoder(this@MecanumDrive.leftBack))
        val rightBack: Encoder = OverflowEncoder(RawEncoder(this@MecanumDrive.rightBack))
        val rightFront: Encoder = OverflowEncoder(RawEncoder(this@MecanumDrive.rightFront))
        val imu: IMU = lazyImu.get()

        private var lastLeftFrontPos = 0.0
        private var lastLeftBackPos = 0.0
        private var lastRightBackPos = 0.0
        private var lastRightFrontPos = 0.0
        private var lastHeading: Rotation2d? = null
        private var initialized = false

        override fun update(): Twist2dDual<Time> {
            val leftFrontPosVel = leftFront.getPositionAndVelocity()
            val leftBackPosVel = leftBack.getPositionAndVelocity()
            val rightBackPosVel = rightBack.getPositionAndVelocity()
            val rightFrontPosVel = rightFront.getPositionAndVelocity()

            val angles = imu.robotYawPitchRollAngles

            write(
                "MECANUM_LOCALIZER_INPUTS", MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles
                )
            )

            val heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

            if (!initialized) {
                initialized = true

                lastLeftFrontPos = leftFrontPosVel.position.toDouble()
                lastLeftBackPos = leftBackPosVel.position.toDouble()
                lastRightBackPos = rightBackPosVel.position.toDouble()
                lastRightFrontPos = rightFrontPosVel.position.toDouble()

                lastHeading = heading

                return Twist2dDual<Time>(
                    Vector2dDual.constant<Time>(Vector2d(0.0, 0.0), 2),
                    DualNum.constant<Time>(0.0, 2)
                )
            }

            val headingDelta = heading.minus(lastHeading!!)
            val twist = kinematics.forward<Time>(
                WheelIncrements<Time>(
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity.toDouble(),
                        )
                    ).times(PARAMS.inPerTick)
                )
            )

            lastLeftFrontPos = leftFrontPosVel.position.toDouble()
            lastLeftBackPos = leftBackPosVel.position.toDouble()
            lastRightBackPos = rightBackPosVel.position.toDouble()
            lastRightFrontPos = rightFrontPosVel.position.toDouble()

            lastHeading = heading

            return Twist2dDual<Time>(
                twist.line,
                DualNum.cons<Time>(headingDelta, twist.angle.drop(1))
            )
        }
    }

    init {
        this.pose = pose

        throwIfModulesAreOutdated(hardwareMap)

        for (module in hardwareMap.getAll<LynxModule>(LynxModule::class.java)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "leftFront")
        leftBack = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "leftBack")
        rightBack = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "rightBack")
        rightFront = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, "rightFront")

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = LazyImu(
            hardwareMap, "imu", RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection
            )
        )

        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        localizer = DriveLocalizer()

        write("MECANUM_PARAMS", PARAMS)
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels: WheelVelocities<Time> = MecanumKinematics(1.0).inverse<Time>(
            PoseVelocity2dDual.constant<Time>(powers, 1)
        )

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        leftFront.power = wheelVels.leftFront[0] / maxPowerMag
        leftBack.power = wheelVels.leftBack[0] / maxPowerMag
        rightBack.power = wheelVels.rightBack[0] / maxPowerMag
        rightFront.power = wheelVels.rightFront[0] / maxPowerMag
    }

    inner class FollowTrajectoryAction(val timeTrajectory: TimeTrajectory) : Action {
        private var beginTs = -1.0

        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {

            val disps: List<Double> = range(
                0.0, timeTrajectory.path.length(),
                max(2, ceil(timeTrajectory.path.length() / 2).toInt())
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val p = timeTrajectory.path[disps[i], 1].value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            var t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = timeTrajectory[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                .compute(txWorldTarget, pose, robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: WheelVelocities<Time> = kinematics.inverse<Time>(command)
            val voltage = voltageSensor.voltage

            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                )
            )

            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower

            p.put("x", pose.position.x)
            p.put("y", pose.position.y)
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()))

            val error = txWorldTarget.value().minusExp(pose)
            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            drawRobot(c, pose)

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(c: Canvas) {
            c.setStroke("#4CAF507A")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(val turn: TimeTurn) : Action {

        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            var t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = turn[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                .compute(txWorldTarget, pose, robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels: WheelVelocities<Time> = kinematics.inverse<Time>(command)
            val voltage = voltageSensor.voltage
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                )
            )

            leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
            leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
            rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
            rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage

            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            drawRobot(c, pose)

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

            return true
        }

        override fun preview(c: Canvas) {
            c.setStroke("#7C4DFF7A")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }
    }

    fun updatePoseEstimate(): PoseVelocity2d {
        val twist = localizer.update()
        pose = pose.plus(twist.value())

        poseHistory.add(pose)
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(pose))

        return twist.velocity().value()
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        var i = 0
        for (t in poseHistory) {
            xPoints[i] = t.position.x
            yPoints[i] = t.position.y

            i++
        }

        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            TurnActionFactory { turn: TimeTurn -> TurnAction(turn) },
            TrajectoryActionFactory { t: TimeTrajectory -> FollowTrajectoryAction(t) },
            TrajectoryBuilderParams(
                1e-6,
                ProfileParams(
                    0.25, 0.1, 1e-2
                )
            ),
            beginPose, 0.0,
            defaultTurnConstraints,
            defaultVelConstraint, defaultAccelConstraint
        )
    }

    companion object {
        var PARAMS: Params = Params()
    }
}
