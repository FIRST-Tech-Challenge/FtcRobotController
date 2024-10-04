package org.firstinspires.ftc.teamcode.trajectorysequence

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.config.DriveConstants
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import org.firstinspires.ftc.teamcode.util.LogFiles

@Config
class TrajectorySequenceRunner(
    private val follower: TrajectoryFollower,
    headingPIDCoefficients: PIDCoefficients,
    voltageSensor: VoltageSensor,
    lastDriveEncPositions: List<Int>,
    lastDriveEncVels: List<Int>,
    lastTrackingEncPositions: List<Int>,
    lastTrackingEncVels: List<Int>
) {

    private val turnController: PIDFController = PIDFController(headingPIDCoefficients)

    private val clock: NanoClock

    private var currentTrajectorySequence: TrajectorySequence? = null

    private var currentSegmentStartTime = 0.0
    private var currentSegmentIndex = 0
    private var lastSegmentIndex = 0

    private var lastPoseError: Pose2d = Pose2d()

    private var remainingMarkers: MutableList<TrajectoryMarker> = java.util.ArrayList<TrajectoryMarker>()

    private val dashboard: FtcDashboard
    private val poseHistory: java.util.LinkedList<Pose2d> = java.util.LinkedList<Pose2d>()

    private val voltageSensor: VoltageSensor

    private val lastDriveEncPositions: List<Int>
    private val lastDriveEncVels: List<Int>
    private val lastTrackingEncPositions: List<Int>
    private val lastTrackingEncVels: List<Int>

    init {
        turnController.setInputBounds(0.0, 2 * Math.PI)

        this.voltageSensor = voltageSensor

        this.lastDriveEncPositions = lastDriveEncPositions
        this.lastDriveEncVels = lastDriveEncVels
        this.lastTrackingEncPositions = lastTrackingEncPositions
        this.lastTrackingEncVels = lastTrackingEncVels

        clock = NanoClock.system()

        dashboard = FtcDashboard.getInstance()
        dashboard.telemetryTransmissionInterval = 25
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence) {
        currentTrajectorySequence = trajectorySequence
        currentSegmentStartTime = clock.seconds()
        currentSegmentIndex = 0
        lastSegmentIndex = -1
    }

    fun update(poseEstimate: Pose2d, poseVelocity: Pose2d?): DriveSignal? {
        var targetPose: Pose2d? = null
        var driveSignal: DriveSignal? = null

        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()

        if (currentTrajectorySequence == null) {
            return null
        }

        if (currentSegmentIndex >= currentTrajectorySequence?.size()!!) {
            for (marker in remainingMarkers) {
                marker.callback.onMarkerReached()
            }

            remainingMarkers.clear()

            currentTrajectorySequence = null

            return null
        }

        val now: Double = clock.seconds()
        val isNewTransition = currentSegmentIndex != lastSegmentIndex

        val currentSegment = currentTrajectorySequence!!.get(currentSegmentIndex)

        if (isNewTransition) {
            currentSegmentStartTime = now
            lastSegmentIndex = currentSegmentIndex

            for (marker in remainingMarkers) {
                marker.callback.onMarkerReached()
            }

            remainingMarkers.clear()

            remainingMarkers.addAll(currentSegment.markers)
            remainingMarkers.sortWith { t1, t2 -> t1.time.compareTo(t2.time) }
        }

        val deltaTime = now - currentSegmentStartTime

        if (currentSegment is TrajectorySegment) {
            val currentTrajectory: Trajectory = currentSegment.trajectory

            if (isNewTransition) follower.followTrajectory(currentTrajectory)

            if (!follower.isFollowing()) {
                currentSegmentIndex++

                driveSignal = DriveSignal()
            } else {
                driveSignal = follower.update(poseEstimate, poseVelocity)
                lastPoseError = follower.lastError
            }

            targetPose = currentTrajectory[deltaTime]
        } else if (currentSegment is TurnSegment) {
            val targetState: MotionState = currentSegment.motionProfile[deltaTime]

            turnController.targetPosition = targetState.x

            val correction: Double = turnController.update(poseEstimate.heading)

            val targetOmega: Double = targetState.v
            val targetAlpha: Double = targetState.a

            lastPoseError = Pose2d(0.0, 0.0, turnController.lastError)

            val startPose: Pose2d = currentSegment.startPose
            targetPose = startPose.copy(startPose.x, startPose.y, targetState.x)

            driveSignal = DriveSignal(
                Pose2d(0.0, 0.0, targetOmega + correction),
                Pose2d(0.0, 0.0, targetAlpha)
            )

            if (deltaTime >= currentSegment.duration) {
                currentSegmentIndex++
                driveSignal = DriveSignal()
            }
        } else if (currentSegment is WaitSegment) {
            lastPoseError = Pose2d()

            targetPose = currentSegment.startPose
            driveSignal = DriveSignal()

            if (deltaTime >= currentSegment.duration) {
                currentSegmentIndex++
            }
        }

        while (remainingMarkers.size > 0 && deltaTime > remainingMarkers[0].time) {
            remainingMarkers[0].callback.onMarkerReached()
            remainingMarkers.removeAt(0)
        }

        poseHistory.add(poseEstimate)

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst()
        }

        val nominalVoltage = 12.0
        val voltage: Double = voltageSensor.voltage
        if (driveSignal != null && !DriveConstants.RUN_USING_ENCODER) {
            driveSignal = DriveSignal(
                driveSignal.vel.times(nominalVoltage / voltage),
                driveSignal.accel.times(nominalVoltage / voltage)
            )
        }

        if (targetPose != null) {
            LogFiles.record(
                targetPose, poseEstimate, voltage,
                lastDriveEncPositions, lastDriveEncVels, lastTrackingEncPositions, lastTrackingEncVels
            )
        }

        packet.put("x", poseEstimate.x)
        packet.put("y", poseEstimate.y)
        packet.put("heading (deg)", Math.toDegrees(poseEstimate.heading))

        packet.put("xError", getLastPoseError().x)
        packet.put("yError", getLastPoseError().y)
        packet.put("headingError (deg)", Math.toDegrees(getLastPoseError().heading))

        draw(fieldOverlay, currentTrajectorySequence, currentSegment, targetPose, poseEstimate)

        dashboard.sendTelemetryPacket(packet)

        return driveSignal
    }

    private fun draw(
        fieldOverlay: Canvas,
        sequence: TrajectorySequence?, currentSegment: SequenceSegment?,
        targetPose: Pose2d?, poseEstimate: Pose2d
    ) {
        if (sequence != null) {
            for (i in 0 until sequence.size()) {
                when (val segment: SequenceSegment = sequence.get(i)) {
                    is TrajectorySegment -> {
                        fieldOverlay.setStrokeWidth(1)
                        fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY)

                        DashboardUtil.drawSampledPath(
                            fieldOverlay,
                            segment.trajectory.path
                        )
                    }

                    is TurnSegment -> {
                        val pose: Pose2d = segment.startPose

                        fieldOverlay.setFill(COLOR_INACTIVE_TURN)
                        fieldOverlay.fillCircle(pose.x, pose.y, 2.0)
                    }

                    is WaitSegment -> {
                        val pose: Pose2d = segment.startPose

                        fieldOverlay.setStrokeWidth(1)
                        fieldOverlay.setStroke(COLOR_INACTIVE_WAIT)
                        fieldOverlay.strokeCircle(pose.x, pose.y, 3.0)
                    }
                }
            }
        }

        if (currentSegment != null) {
            when (currentSegment) {
                is TrajectorySegment -> {
                    val currentTrajectory: Trajectory = currentSegment.trajectory

                    fieldOverlay.setStrokeWidth(1)
                    fieldOverlay.setStroke(COLOR_ACTIVE_TRAJECTORY)

                    DashboardUtil.drawSampledPath(fieldOverlay, currentTrajectory.path)
                }

                is TurnSegment -> {
                    val pose: Pose2d = currentSegment.startPose

                    fieldOverlay.setFill(COLOR_ACTIVE_TURN)
                    fieldOverlay.fillCircle(pose.x, pose.y, 3.0)
                }

                is WaitSegment -> {
                    val pose: Pose2d = currentSegment.startPose

                    fieldOverlay.setStrokeWidth(1)
                    fieldOverlay.setStroke(COLOR_ACTIVE_WAIT)
                    fieldOverlay.strokeCircle(pose.x, pose.y, 3.0)
                }
            }
        }

        if (targetPose != null) {
            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.setStroke("#4CAF50")
            DashboardUtil.drawRobot(fieldOverlay, targetPose)
        }

        fieldOverlay.setStroke("#3F51B5")
        DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory)

        fieldOverlay.setStroke("#3F51B5")
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate)
    }

    fun getLastPoseError(): Pose2d {
        return lastPoseError
    }

    val isBusy: Boolean
        get() = currentTrajectorySequence != null

    companion object {
        var COLOR_INACTIVE_TRAJECTORY: String = "#4caf507a"
        var COLOR_INACTIVE_TURN: String = "#7c4dff7a"
        var COLOR_INACTIVE_WAIT: String = "#dd2c007a"

        var COLOR_ACTIVE_TRAJECTORY: String = "#4CAF50"
        var COLOR_ACTIVE_TURN: String = "#7c4dff"
        var COLOR_ACTIVE_WAIT: String = "#dd2c00"

        var POSE_HISTORY_LIMIT: Int = 100
    }
}