package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID
import org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER
import org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches
import org.firstinspires.ftc.teamcode.drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic
import org.firstinspires.ftc.teamcode.drive.DriveConstants.kV
import kotlin.math.abs

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
class SampleMecanumDrive(hardwareMap: HardwareMap) :
    MecanumDrive(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER) {
    private val trajectorySequenceRunner: TrajectorySequenceRunner

    private val follower: TrajectoryFollower = HolonomicPIDVAFollower(
        TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
        Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5
    )

    private val leftFront: DcMotorEx
    private val leftRear: DcMotorEx
    private val rightRear: DcMotorEx
    private val rightFront: DcMotorEx
    private val motors: List<DcMotorEx>

    private val imu: IMU
    private val batteryVoltageSensor: VoltageSensor

    private val lastEncPositions: MutableList<Int> = java.util.ArrayList<Int>()
    private val lastEncVels: MutableList<Int> = java.util.ArrayList<Int>()

    init {

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU::class.java, "imu")
        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR
            )
        )
        imu.initialize(parameters)

        leftFront = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        leftRear = hardwareMap.get(DcMotorEx::class.java, "leftRear")
        rightRear = hardwareMap.get(DcMotorEx::class.java, "rightRear")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rightFront")

        motors = listOf(leftFront, leftRear, rightRear, rightFront)

        for (motor in motors) {
            val motorConfigurationType: MotorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        if (RUN_USING_ENCODER) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID)
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        val lastTrackingEncPositions: List<Int> = java.util.ArrayList<Int>()
        val lastTrackingEncVels: List<Int> = java.util.ArrayList<Int>()

        // TODO: if desired, use setLocalizer() to change the localization method
        // setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));
        trajectorySequenceRunner = TrajectorySequenceRunner(
            follower, HEADING_PID, batteryVoltageSensor,
            lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        )
    }

    fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, false, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectoryBuilder(startPose: Pose2d, startHeading: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectorySequenceBuilder(startPose: Pose2d): TrajectorySequenceBuilder {
        return TrajectorySequenceBuilder(
            startPose,
            VEL_CONSTRAINT, ACCEL_CONSTRAINT,
            MAX_ANG_VEL, MAX_ANG_ACCEL
        )
    }

    fun turnAsync(angle: Double) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(poseEstimate)
                .turn(angle)
                .build()
        )
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    private fun followTrajectoryAsync(trajectory: Trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start())
                .addTrajectory(trajectory)
                .build()
        )
    }

    fun followTrajectory(trajectory: Trajectory) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    private fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    val lastError: Pose2d
        get() = trajectorySequenceRunner.getLastPoseError()

    fun update() {
        updatePoseEstimate()
        val signal: DriveSignal? = trajectorySequenceRunner.update(poseEstimate, poseVelocity)
        if (signal != null) setDriveSignal(signal)
    }

    private fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy) update()
    }

    val isBusy: Boolean
        get() = trajectorySequenceRunner.isBusy

    fun setMode(runMode: DcMotor.RunMode?) {
        for (motor in motors) {
            motor.mode = runMode
        }
    }

    private fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        for (motor in motors) {
            motor.zeroPowerBehavior = zeroPowerBehavior
        }
    }

    fun setPIDFCoefficients(runMode: DcMotor.RunMode?, coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
            coefficients.p, coefficients.i, coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.voltage
        )

        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients)
        }
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        var vel: Pose2d = drivePower

        if ((abs(drivePower.x) + abs(drivePower.y) + abs(drivePower.heading)) > 1) {
            // re-normalize the powers according to the weights
            val denom: Double =
                VX_WEIGHT * abs(
                    drivePower.x
                ) + VY_WEIGHT * abs(
                    drivePower.y
                ) + OMEGA_WEIGHT * abs(
                    drivePower.heading
                )

            vel = Pose2d(
                VX_WEIGHT * drivePower.x,
                VY_WEIGHT * drivePower.y,
                OMEGA_WEIGHT * drivePower.heading
            ).div(denom)
        }

        setDrivePower(vel)
    }

    override fun getWheelPositions(): List<Double> {
        lastEncPositions.clear()

        val wheelPositions: MutableList<Double> = java.util.ArrayList<Double>()
        for (motor in motors) {
            val position: Int = motor.currentPosition
            lastEncPositions.add(position)
            wheelPositions.add(encoderTicksToInches(position.toDouble()))
        }
        return wheelPositions
    }

    override fun getWheelVelocities(): List<Double> {
        lastEncVels.clear()

        val wheelVelocities: MutableList<Double> = java.util.ArrayList<Double>()
        for (motor in motors) {
            val vel = motor.velocity.toInt()
            lastEncVels.add(vel)
            wheelVelocities.add(encoderTicksToInches(vel.toDouble()))
        }
        return wheelVelocities
    }

    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        leftFront.power = frontLeft
        leftRear.power = rearLeft
        rightRear.power = rearRight
        rightFront.power = frontRight
    }

    override val rawExternalHeading: Double
        get() = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

    val externalHeadingVelocity: Double
        get() = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate.toDouble()

    companion object {
        var TRANSLATIONAL_PID: PIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)
        var HEADING_PID: PIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)

        var LATERAL_MULTIPLIER: Double = 1.0

        var VX_WEIGHT: Double = 1.0
        var VY_WEIGHT: Double = 1.0
        var OMEGA_WEIGHT: Double = 1.0

        private val VEL_CONSTRAINT: TrajectoryVelocityConstraint =
            getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH)
        private val ACCEL_CONSTRAINT: TrajectoryAccelerationConstraint = getAccelerationConstraint(MAX_ACCEL)

        private fun getVelocityConstraint(
            maxVel: Double,
            maxAngularVel: Double,
            trackWidth: Double
        ): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(
                listOf(
                    AngularVelocityConstraint(maxAngularVel),
                    MecanumVelocityConstraint(maxVel, trackWidth)
                )
            )
        }

        private fun getAccelerationConstraint(maxAccel: Double): TrajectoryAccelerationConstraint {
            return ProfileAccelerationConstraint(maxAccel)
        }
    }
}