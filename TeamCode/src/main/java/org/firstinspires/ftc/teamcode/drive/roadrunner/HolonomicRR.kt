package org.firstinspires.ftc.teamcode.drive.roadrunner

//import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsNew.*
import androidx.annotation.NonNull
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.drive.roadrunner.RobotConstantsAccessor.baseConstraints
import org.firstinspires.ftc.teamcode.drive.roadrunner.RobotConstantsAccessor.encoderTicksToInches
import org.firstinspires.ftc.teamcode.drive.roadrunner.RobotConstantsAccessor.headingPID
import org.firstinspires.ftc.teamcode.drive.roadrunner.RobotConstantsAccessor.kA
import org.firstinspires.ftc.teamcode.drive.roadrunner.RobotConstantsAccessor.kStatic
import org.firstinspires.ftc.teamcode.drive.roadrunner.RobotConstantsAccessor.kV
import org.firstinspires.ftc.teamcode.drive.roadrunner.RobotConstantsAccessor.motorVelocityPID
import org.firstinspires.ftc.teamcode.drive.roadrunner.RobotConstantsAccessor.trackWidth
import org.firstinspires.ftc.teamcode.drive.roadrunner.RobotConstantsAccessor.translationalXPID
import org.firstinspires.ftc.teamcode.drive.roadrunner.RobotConstantsAccessor.translationalYPID
import org.firstinspires.ftc.teamcode.drive.roadrunner.constants.DriveConstantsThinBot.globalPoseEstimate
import org.firstinspires.ftc.teamcode.library.functions.toDegrees
import org.firstinspires.ftc.teamcode.drive.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.Holonomic
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.HolonomicImpl
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.encoderTicksToInches
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.headingPID
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.kA
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.kF
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.kStatic
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.kV
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.motorVelocityPID
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.runUsingEncoder
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsThinBot.globalPoseEstimate
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.support.DashboardUtil
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import kotlin.math.absoluteValue


class HolonomicRR

constructor (
        private val imuController: IMUController,
        private val frontLeftExt: DcMotorEx,
        private val backLeftExt:  DcMotorEx,
        private val backRightExt: DcMotorEx,
        private val frontRightExt: DcMotorEx,
        localizer: Localizer? = null)

    : MecanumDrive(kV, kA, kStatic, trackWidth), Holonomic
{
    /*


        RoadRunner holonomic functions


     */

    // List of motors for quick access
    private val motorsExt = listOf(frontLeftExt, backLeftExt, backRightExt, frontRightExt)

    // Variables to track current state of the drive system
    private enum class Mode { IDLE, TURN, FOLLOW_TRAJECTORY, LEGACY_MOVE, LEGACY_FOLLOW }
    private var mode = Mode.IDLE

    // Variable for obtaining current time
    private val clock = NanoClock.system()

    // Quick access to FtcDashboard instance
    private val dashboard = FtcDashboard.getInstance()

    // Turn controller and turn motion profile
    private val turnController = PIDFController(headingPID)
    private lateinit var turnProfile : MotionProfile

    // Constraints for the drivetrain and the holonomic motion profile follower
    private var driveConstraints = MecanumConstraints(baseConstraints, trackWidth)
    private var follower = HolonomicPIDVAFollower(translationalXPID, translationalYPID, headingPID)

    // For determining the hardware refresh rate
    private var lastReadTime : Long = 0

    // The start time for a trajectory or turn movement
    private var movementStart = 0.0
    private var movementStartPosition: Pose2d = poseEstimate
    var safeModeLastTriggered: Long? = null
        private set

    // List of waypoint actions for the given trajectory movement
    private var trajectoryWaypointActions = emptyList<Pair<Double, ()->Unit>>().toMutableList()

    var safeModeErrorThreshold = 10

    init {
        dashboard.telemetryTransmissionInterval = 25
        turnController.setInputBounds(0.0, 2.0 * Math.PI)

        // TODO: Fix global pose estimate to reference RobotConstantsAccessor
        if (globalPoseEstimate != null) poseEstimate = globalPoseEstimate

//        Thread { while (!Thread.interrupted()) updatePoseEstimate() }.start()

        super.localizer = localizer ?: MecanumLocalizer(this)
    }

//    val trajectoryBuilder : TrajectoryBuilder
//        get() = TrajectoryBuilder(poseEstimate, driveConstraints)

    /**
     * Returns the last known error from the current drive controller
     */
    val lastError: Pose2d
        get() = when(mode) {
            Mode.FOLLOW_TRAJECTORY -> follower.lastError
            Mode.TURN -> Pose2d(0.0, 0.0, turnController.lastError)
            else                    -> Pose2d()
        }

    public fun doMotorConfigForAutonomous() {
        motorsExt.forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.direction = DcMotorSimple.Direction.FORWARD
            if (runUsingEncoder) {
                it.mode = DcMotor.RunMode.RUN_USING_ENCODER
                if (motorVelocityPID != null) setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, motorVelocityPID as PIDCoefficients)
            }
        }
    }

    /**
     * Updates the current pose estimate and report to FtcDashboard
     */
    @JvmOverloads
    fun update(safeMode: Boolean = false) {
        val beforePoseUpdate = System.currentTimeMillis()
        updatePoseEstimate()
        val afterPoseUpdate = System.currentTimeMillis()

        val currentPose = poseEstimate

        val afterGetPose = System.currentTimeMillis()
        var beforeGetDriveSignal = Long.MIN_VALUE
        var afterGetDriveSignal = Long.MIN_VALUE
        val lastError = this.lastError

        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()

        packet.put("mode", mode)

        val currentHeading = currentPose.heading

        packet.put("x", currentPose.x)
        packet.put("y", currentPose.y)
        packet.put("heading", currentHeading)
        packet.put("heading (deg)", currentHeading.toDegrees())

        packet.put("xError", lastError.x)
        packet.put("yError", lastError.y)
        packet.put("headingError", lastError.heading)

        val imuHeading = rawExternalHeading

        packet.put("imu heading", imuHeading)
        packet.put("imu heading (deg)", imuHeading.toDegrees())

        packet.put("heading diff (deg)", currentHeading.toDegrees() - imuHeading.toDegrees())

        fieldOverlay.setStroke("#3F51B5")
        fieldOverlay.fillCircle(currentPose.x, currentPose.y, 3.0)

        when (mode) {
            Mode.TURN -> {
                val t = clock.seconds() - movementStart

                val targetState = turnProfile[t]

                turnController.targetPosition = targetState.x

                val targetOmega = targetState.v
                val targetAlpha = targetState.a
                val correction = turnController.update(currentPose.heading, targetOmega)

                setDriveSignal(
                        DriveSignal(
                                Pose2d(0.0, 0.0, targetOmega + correction),
                                Pose2d(0.0, 0.0, targetAlpha)
                        )
                )

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())

                }
            }
            Mode.FOLLOW_TRAJECTORY -> {
                val toSet = follower.update(currentPose)
                beforeGetDriveSignal = System.currentTimeMillis()
                setDriveSignal(toSet)
                afterGetDriveSignal = System.currentTimeMillis()

                val trajectory = follower.trajectory

                val currentTrajectoryDuration = clock.seconds() - movementStart
                val totalTrajectoryDuration = trajectory.duration()
                val trajectoryTimeRatio = currentTrajectoryDuration/totalTrajectoryDuration

//                if (trajectoryWaypointActions.isNotEmpty()) {
//                    val currentActionPair = trajectoryWaypointActions.first()
//                    if (trajectoryTimeRatio > currentActionPair.first) {
//                        currentActionPair.second.invoke()
//                        trajectoryWaypointActions.remove(currentActionPair)
//                    }
//                }

                trajectoryWaypointActions.firstOrNull()?.also {
                    if (trajectoryTimeRatio > it.first) {
                        it.second.invoke()
                        trajectoryWaypointActions.remove(it)
                    }
                }

                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke("#4CAF50")
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.path)

                fieldOverlay.setStroke("#F44336")
                val time = follower.elapsedTime()
                DashboardUtil.drawRobot(fieldOverlay, trajectory[time])

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }

                if (safeMode) {
//                    val currentPose = poseEstimate
//                    val deltaX = (currentPose.x - movementStartPosition.x).absoluteValue
//                    val deltaY = (currentPose.y - movementStartPosition.y).absoluteValue
//                    System.out.println("HolonomicRR CLOCK ${clock.seconds() - movementStart} deltaX $deltaX deltaY $deltaY")
                    if ((clock.seconds() - movementStart) > 0.5 && (lastError.x.absoluteValue + lastError.y.absoluteValue > this.safeModeErrorThreshold)) {
                        safeModeLastTriggered = System.currentTimeMillis()
                        stop()
                    }
                }
            }

        }

        val endReadTime = System.currentTimeMillis()
        println()
        print("%% HolonomicRR_updates\tREAD GAP = ${beforePoseUpdate - lastReadTime}")
        lastReadTime = endReadTime
        print("\tAFTER POSE UPDATE = ${afterPoseUpdate - beforePoseUpdate}")
        print("\tAFTER GET POSE = ${afterGetPose - beforePoseUpdate}")
        print("\tAFTER UPDATE FOLLOWER = ${beforeGetDriveSignal - beforePoseUpdate}")
        print("\tAFTER GET DRIVE SIGNAL = ${afterGetDriveSignal - beforePoseUpdate}")
        print("\tUPDATE TIME = ${endReadTime - beforePoseUpdate}\t")
        print("\t %% HolonomicRR_pose\t X=${poseEstimate.x}\t Y=${poseEstimate.y}\t HEADING=${currentHeading}\t IMU_HEADING=${imuHeading}\t   XERROR=${lastError.x}\t YERROR=${lastError.y}\t HEADINGERROR=${lastError.heading}")
        dashboard.sendTelemetryPacket(packet)

        globalPoseEstimate = poseEstimate
    }

    /**
     * Set drive system to follow given trajectory
     *
     * @param trajectory The trajectory to follow
     * @param waypointActions List of actions to accomplish at a certain percentage (from 0.0 to 1.0) of movement completion
     */
    @JvmOverloads fun followTrajectory(trajectory: Trajectory, waypointActions: List<Pair<Double, ()->Unit>> = emptyList()) {

//        doMotorConfigForAutonomous()

        follower.followTrajectory(trajectory)
        lastReadTime = System.currentTimeMillis()
//        driveSignalUpdateThread = Thread(driveSignalUpdateRunnable)
//        driveSignalUpdateThread.start()
        mode = Mode.FOLLOW_TRAJECTORY
        movementStart = clock.seconds()
        System.out.println("MOVEMENT START $movementStart")
        movementStartPosition = poseEstimate
        trajectoryWaypointActions = waypointActions.sortedBy { it.first }.toMutableList()
    }

    /**
     * Follow a given trajectory and loop until completion
     *
     * @param trajectory The trajectory to follow
     * @param waypointActions List of actions to accomplish at a certain percentage (from 0.0 to 1.0) of movement completion
     */
    @JvmOverloads fun followTrajectorySync(trajectory: Trajectory, waypointActions: List<Pair<Double, ()->Unit>>? = null, safeMode: Boolean = false) {
        followTrajectory(trajectory, waypointActions ?: emptyList())
        waitForIdle(safeMode)
    }

    /**
     * Creates a turn profile and sets drive system to follow it
     *
     * @param angle The angle at which to turn, in degrees
     */
    fun turn(angle: Double) {

//        doMotorConfigForAutonomous()

        val heading = poseEstimate.heading

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(heading, 0.0, 0.0, 0.0),
                MotionState(heading + angle, 0.0, 0.0, 0.0),
                driveConstraints.maxAngVel,
                driveConstraints.maxAngAccel,
                driveConstraints.maxAngJerk
        )

        movementStart = clock.seconds()
        lastReadTime = System.currentTimeMillis()
//        driveSignalUpdateThread = Thread(driveSignalUpdateRunnable)
//        driveSignalUpdateThread.start()
        mode = Mode.TURN
    }

    /**
     * Turn to a given angle and loop until completion
     */
    fun turnSync(angle: Double) {
        turn(angle)
        waitForIdle()
    }

    /**
     * Wait until the robot mode is IDLE
     */
    private fun waitForIdle(safeMode: Boolean = false) {
        while (!Thread.currentThread().isInterrupted && isBusy()) {
            update(safeMode)
        }
    }

    /**
     * See whether the robot is currently following a motion profile
     *
     * @return Boolean representing whether the drive system mode is idle
     */
    fun isBusy() : Boolean {
        return mode == Mode.FOLLOW_TRAJECTORY
                || mode == Mode.TURN
                || (mode == Mode.LEGACY_FOLLOW && holonomic.motorsAreBusy())
    }


    /**
     * Get the PID coefficients for the drivetrain
     *
     * @param runMode The motor run mode for which PID coefficients should be obtained
     * @return The PID coefficients for the given runmode, from the front left motor
     */
    fun getPIDCoefficients(runMode: DcMotor.RunMode) : PIDCoefficients {
        val pidfCoefficients = (frontLeftExt as DcMotorEx).getPIDFCoefficients(runMode)
        return PIDCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d)
    }

    /**
     * Set the PID coefficients for the drivetrain
     *
     * @param runMode The motor run mode for which PID coefficients should be obtained
     * @param coefficients The PID coefficients that should be set to the motors
     */
    fun setPIDCoefficients(runMode: DcMotor.RunMode, coefficients: PIDCoefficients) {
        motorsExt.forEach {
            (it as DcMotorEx).setPIDFCoefficients(runMode, PIDFCoefficients(coefficients.kP, coefficients.kI, coefficients.kD, kF))
            // set kF to motor velocity F coefficient, look at RR quickstart, DriveConstantsNew
        }
    }

    /**
     * Get the positions for all four motors as a list
     */
    @NonNull
    override fun getWheelPositions() : List<Double> {
        return listOf(
                encoderTicksToInches(frontLeftExt.currentPosition.toDouble()),
                encoderTicksToInches(backLeftExt.currentPosition.toDouble()),
                encoderTicksToInches(backRightExt.currentPosition.toDouble()),
                encoderTicksToInches(frontRightExt.currentPosition.toDouble())
        )
    }

    /**
     * Get the velocities for all four motors as a list
     */
    @NonNull
    override fun getWheelVelocities() : List<Double> {
        return listOf(
                encoderTicksToInches(frontLeftExt.velocity),
                encoderTicksToInches(backLeftExt.velocity),
                encoderTicksToInches(backRightExt.velocity),
                encoderTicksToInches(frontRightExt.velocity)
        )
    }

    /**
     * Returns the current heading of the robot, from IMU
     */
    override val rawExternalHeading: Double
        get() = imuController.getHeading()

    /**
     * Sets motor powers to each of the four motors
     */
    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        frontLeftExt .power =  frontLeft * reversedMotorMultiplier(frontLeftExt)
        backLeftExt  .power =  rearLeft * reversedMotorMultiplier(backLeftExt)
        backRightExt .power = -rearRight * reversedMotorMultiplier(backRightExt)
        frontRightExt.power = -frontRight * reversedMotorMultiplier(frontRightExt)
    }

    private fun reversedMotorMultiplier(motor: DcMotorEx) = if (motor.direction == DcMotorSimple.Direction.REVERSE) -1 else 1

    /**
     * Redefine the drive parameters with new values from [RobotConstantsAccessor]
     */
    fun redefine() {
        follower = HolonomicPIDVAFollower(translationalXPID, translationalYPID, headingPID)
        driveConstraints = MecanumConstraints(baseConstraints, trackWidth)
    }

    @JvmOverloads fun trajectoryBuilder(tangent : Double, _driveConstraints : DriveConstraints = driveConstraints) =
            TrajectoryBuilder(poseEstimate, startTangent  = tangent, constraints = _driveConstraints)
    @JvmOverloads fun trajectoryBuilder(_driveConstraints : DriveConstraints = driveConstraints) =
            TrajectoryBuilder(poseEstimate, constraints = _driveConstraints)


    /*


        Legacy holonomic functions


     */
    var holonomic = HolonomicImpl(frontLeftExt, backLeftExt, frontRightExt, backRightExt)

    override fun runWithoutEncoder(x: Double, y: Double, z: Double) {
//        mode = if (x == 0.0 && y == 0.0 && z == 0.0)
        mode = Mode.LEGACY_MOVE
        holonomic.runWithoutEncoder(x, y, z)
    }

    override fun runWithoutEncoderVectored(x: Double, y: Double, z: Double, offsetTheta: Double) {
        mode = Mode.LEGACY_MOVE
        holonomic.runWithoutEncoderVectored(x, y, z, offsetTheta)
    }

    override fun setMotorsMode(runMode: DcMotor.RunMode?) {
        holonomic.setMotorsMode(runMode)
    }

    override fun stop() {
        mode = Mode.IDLE
        holonomic.stop()
    }

    override fun runWithoutEncoderPrime(xPrime: Double, yPrime: Double, z: Double) {
        mode = Mode.LEGACY_MOVE
        holonomic.runWithoutEncoderPrime(xPrime, yPrime, z)
    }

    override fun runUsingEncoder(xTarget: Double, yTarget: Double, inputPower: Double) {
        mode = Mode.LEGACY_FOLLOW
        holonomic.runUsingEncoder(xTarget, yTarget, inputPower)
    }

    override fun turnUsingEncoder(degrees: Double, power: Double) {
        mode = Mode.LEGACY_FOLLOW
        holonomic.turnUsingEncoder(degrees, power)
    }

    override fun motorsAreBusy(): Boolean = isBusy()

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        holonomic.setZeroPowerBehavior(zeroPowerBehavior)
    }
}