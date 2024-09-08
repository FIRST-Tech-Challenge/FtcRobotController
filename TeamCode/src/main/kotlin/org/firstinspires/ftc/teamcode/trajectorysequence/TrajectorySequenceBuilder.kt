package org.firstinspires.ftc.teamcode.trajectorysequence

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathContinuityViolationException
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.DisplacementMarker
import com.acmerobotics.roadrunner.trajectory.DisplacementProducer
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.SpatialMarker
import com.acmerobotics.roadrunner.trajectory.TemporalMarker
import com.acmerobotics.roadrunner.trajectory.TimeProducer
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.util.Angle
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment
import java.lang.Math.toRadians
import kotlin.math.abs

class TrajectorySequenceBuilder(
    startPose: Pose2d,
    startTangent: Double?,
    private val baseVelConstraint: TrajectoryVelocityConstraint,
    private val baseAccelConstraint: TrajectoryAccelerationConstraint,
    baseTurnConstraintMaxAngVel: Double,
    baseTurnConstraintMaxAngAccel: Double
) {
    private val resolution = 0.25

    private var currentVelConstraint: TrajectoryVelocityConstraint
    private var currentAccelConstraint: TrajectoryAccelerationConstraint

    private val baseTurnConstraintMaxAngVel: Double
    private val baseTurnConstraintMaxAngAccel: Double

    private var currentTurnConstraintMaxAngVel: Double
    private var currentTurnConstraintMaxAngAccel: Double

    private val sequenceSegments: MutableList<SequenceSegment>

    private val temporalMarkers: MutableList<TemporalMarker>
    private val displacementMarkers: MutableList<DisplacementMarker>
    private val spatialMarkers: MutableList<SpatialMarker>

    private var lastPose: Pose2d

    private var tangentOffset: Double

    private var setAbsoluteTangent: Boolean
    private var absoluteTangent: Double

    private var currentTrajectoryBuilder: TrajectoryBuilder?

    private var currentDuration: Double
    private var currentDisplacement: Double

    private var lastDurationTraj: Double
    private var lastDisplacementTraj: Double

    init {
        this.currentVelConstraint = baseVelConstraint
        this.currentAccelConstraint = baseAccelConstraint

        this.baseTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel
        this.baseTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel

        this.currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel
        this.currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel

        sequenceSegments = ArrayList()

        temporalMarkers = ArrayList()
        displacementMarkers = ArrayList()
        spatialMarkers = ArrayList()

        lastPose = startPose

        tangentOffset = 0.0

        setAbsoluteTangent = (startTangent != null)
        absoluteTangent = startTangent ?: 0.0

        currentTrajectoryBuilder = null

        currentDuration = 0.0
        currentDisplacement = 0.0

        lastDurationTraj = 0.0
        lastDisplacementTraj = 0.0
    }

    constructor(
        startPose: Pose2d,
        baseVelConstraint: TrajectoryVelocityConstraint,
        baseAccelConstraint: TrajectoryAccelerationConstraint,
        baseTurnConstraintMaxAngVel: Double,
        baseTurnConstraintMaxAngAccel: Double
    ) : this(
        startPose, null,
        baseVelConstraint, baseAccelConstraint,
        baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel
    )

    fun lineTo(endPosition: Vector2d?): TrajectorySequenceBuilder {
        return addPath {
            if (endPosition != null) {
                currentTrajectoryBuilder?.lineTo(
                    endPosition,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        }
    }

    fun lineTo(
        endPosition: Vector2d?,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            if (endPosition != null) {
                currentTrajectoryBuilder?.lineTo(
                    endPosition,
                    velConstraint,
                    accelConstraint
                )
            }
        }
    }

    fun lineToConstantHeading(endPosition: Vector2d?): TrajectorySequenceBuilder {
        return addPath {
            if (endPosition != null) {
                currentTrajectoryBuilder?.lineToConstantHeading(
                    endPosition,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        }
    }

    fun lineToConstantHeading(
        endPosition: Vector2d?,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            if (endPosition != null) {
                currentTrajectoryBuilder?.lineToConstantHeading(
                    endPosition,
                    velConstraint,
                    accelConstraint
                )
            }
        }
    }

    fun lineToLinearHeading(endPose: Pose2d?): TrajectorySequenceBuilder {
        return addPath {
            if (endPose != null) {
                currentTrajectoryBuilder?.lineToLinearHeading(
                    endPose,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        }
    }

    fun lineToLinearHeading(
        endPose: Pose2d?,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            if (endPose != null) {
                currentTrajectoryBuilder?.lineToLinearHeading(
                    endPose,
                    velConstraint,
                    accelConstraint
                )
            }
        }
    }

    fun lineToSplineHeading(endPose: Pose2d?): TrajectorySequenceBuilder {
        return addPath {
            if (endPose != null) {
                currentTrajectoryBuilder?.lineToSplineHeading(
                    endPose,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        }
    }

    fun lineToSplineHeading(
        endPose: Pose2d?,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            if (endPose != null) {
                currentTrajectoryBuilder?.lineToSplineHeading(
                    endPose,
                    velConstraint,
                    accelConstraint
                )
            }
        }
    }

    fun strafeTo(endPosition: Vector2d?): TrajectorySequenceBuilder {
        return addPath {
            if (endPosition != null) {
                currentTrajectoryBuilder?.strafeTo(
                    endPosition,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        }
    }

    fun strafeTo(
        endPosition: Vector2d?,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            if (endPosition != null) {
                currentTrajectoryBuilder?.strafeTo(
                    endPosition,
                    velConstraint,
                    accelConstraint
                )
            }
        }
    }

    fun forward(distance: Double): TrajectorySequenceBuilder {
        return addPath {
            currentTrajectoryBuilder?.forward(
                distance,
                currentVelConstraint,
                currentAccelConstraint
            )
        }
    }

    fun forward(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            currentTrajectoryBuilder?.forward(
                distance,
                velConstraint,
                accelConstraint
            )
        }
    }

    fun back(distance: Double): TrajectorySequenceBuilder {
        return addPath {
            currentTrajectoryBuilder?.back(
                distance,
                currentVelConstraint,
                currentAccelConstraint
            )
        }
    }

    fun back(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            currentTrajectoryBuilder?.back(
                distance,
                velConstraint,
                accelConstraint
            )
        }
    }

    fun strafeLeft(distance: Double): TrajectorySequenceBuilder {
        return addPath {
            currentTrajectoryBuilder?.strafeLeft(
                distance,
                currentVelConstraint,
                currentAccelConstraint
            )
        }
    }

    fun strafeLeft(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            currentTrajectoryBuilder?.strafeLeft(
                distance,
                velConstraint,
                accelConstraint
            )
        }
    }

    fun strafeRight(distance: Double): TrajectorySequenceBuilder {
        return addPath {
            currentTrajectoryBuilder?.strafeRight(
                distance,
                currentVelConstraint,
                currentAccelConstraint
            )
        }
    }

    fun strafeRight(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            currentTrajectoryBuilder?.strafeRight(
                distance,
                velConstraint,
                accelConstraint
            )
        }
    }

    fun splineTo(endPosition: Vector2d?, endHeading: Double): TrajectorySequenceBuilder {
        return addPath {
            if (endPosition != null) {
                currentTrajectoryBuilder?.splineTo(
                    endPosition,
                    endHeading,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        }
    }

    fun splineTo(
        endPosition: Vector2d?,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            if (endPosition != null) {
                currentTrajectoryBuilder?.splineTo(
                    endPosition,
                    endHeading,
                    velConstraint,
                    accelConstraint
                )
            }
        }
    }

    fun splineToConstantHeading(endPosition: Vector2d?, endHeading: Double): TrajectorySequenceBuilder {
        return addPath {
            if (endPosition != null) {
                currentTrajectoryBuilder?.splineToConstantHeading(
                    endPosition,
                    endHeading,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        }
    }

    fun splineToConstantHeading(
        endPosition: Vector2d?,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            if (endPosition != null) {
                currentTrajectoryBuilder?.splineToConstantHeading(
                    endPosition,
                    endHeading,
                    velConstraint,
                    accelConstraint
                )
            }
        }
    }

    fun splineToLinearHeading(endPose: Pose2d?, endHeading: Double): TrajectorySequenceBuilder {
        return addPath {
            if (endPose != null) {
                currentTrajectoryBuilder?.splineToLinearHeading(
                    endPose,
                    endHeading,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        }
    }

    fun splineToLinearHeading(
        endPose: Pose2d?,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            if (endPose != null) {
                currentTrajectoryBuilder?.splineToLinearHeading(
                    endPose,
                    endHeading,
                    velConstraint,
                    accelConstraint
                )
            }
        }
    }

    fun splineToSplineHeading(endPose: Pose2d?, endHeading: Double): TrajectorySequenceBuilder {
        return addPath {
            if (endPose != null) {
                currentTrajectoryBuilder?.splineToSplineHeading(
                    endPose,
                    endHeading,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        }
    }

    fun splineToSplineHeading(
        endPose: Pose2d?,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath {
            if (endPose != null) {
                currentTrajectoryBuilder?.splineToSplineHeading(
                    endPose,
                    endHeading,
                    velConstraint,
                    accelConstraint
                )
            }
        }
    }

    private fun addPath(callback: AddPathCallback): TrajectorySequenceBuilder {
        if (currentTrajectoryBuilder == null) newPath()

        try {
            callback.run()
        } catch (e: PathContinuityViolationException) {
            newPath()
            callback.run()
        }

        val builtTraj: Trajectory = currentTrajectoryBuilder!!.build()

        val durationDifference: Double = builtTraj.duration() - lastDurationTraj
        val displacementDifference: Double = builtTraj.path.length() - lastDisplacementTraj

        lastPose = builtTraj.end()
        currentDuration += durationDifference
        currentDisplacement += displacementDifference

        lastDurationTraj = builtTraj.duration()
        lastDisplacementTraj = builtTraj.path.length()

        return this
    }

    fun setTangent(tangent: Double): TrajectorySequenceBuilder {
        setAbsoluteTangent = true
        absoluteTangent = tangent

        pushPath()

        return this
    }

    private fun setTangentOffset(offset: Double): TrajectorySequenceBuilder {
        setAbsoluteTangent = false

        this.tangentOffset = offset
        this.pushPath()

        return this
    }

    fun setReversed(reversed: Boolean): TrajectorySequenceBuilder {
        return if (reversed) this.setTangentOffset(toRadians(180.0)) else this.setTangentOffset(0.0)
    }

    fun setConstraints(
        velConstraint: TrajectoryVelocityConstraint,
        accelConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        this.currentVelConstraint = velConstraint
        this.currentAccelConstraint = accelConstraint

        return this
    }

    fun resetConstraints(): TrajectorySequenceBuilder {
        this.currentVelConstraint = this.baseVelConstraint
        this.currentAccelConstraint = this.baseAccelConstraint

        return this
    }

    fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint): TrajectorySequenceBuilder {
        this.currentVelConstraint = velConstraint

        return this
    }

    fun resetVelConstraint(): TrajectorySequenceBuilder {
        this.currentVelConstraint = this.baseVelConstraint

        return this
    }

    fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint): TrajectorySequenceBuilder {
        this.currentAccelConstraint = accelConstraint

        return this
    }

    fun resetAccelConstraint(): TrajectorySequenceBuilder {
        this.currentAccelConstraint = this.baseAccelConstraint

        return this
    }

    fun setTurnConstraint(maxAngVel: Double, maxAngAccel: Double): TrajectorySequenceBuilder {
        this.currentTurnConstraintMaxAngVel = maxAngVel
        this.currentTurnConstraintMaxAngAccel = maxAngAccel

        return this
    }

    fun resetTurnConstraint(): TrajectorySequenceBuilder {
        this.currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel
        this.currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel

        return this
    }

    fun addTemporalMarker(callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addTemporalMarker(currentDuration, callback)
    }

    fun UNSTABLE_addTemporalMarkerOffset(offset: Double, callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addTemporalMarker(currentDuration + offset, callback)
    }

    private fun addTemporalMarker(time: Double, callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addTemporalMarker(0.0, time, callback)
    }

    private fun addTemporalMarker(scale: Double, offset: Double, callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addTemporalMarker({ time -> scale * time + offset }, callback)
    }

    private fun addTemporalMarker(time: TimeProducer, callback: MarkerCallback): TrajectorySequenceBuilder {
        temporalMarkers.add(TemporalMarker(time, callback))
        return this
    }

    fun addSpatialMarker(point: Vector2d, callback: MarkerCallback): TrajectorySequenceBuilder {
        spatialMarkers.add(SpatialMarker(point, callback))
        return this
    }

    fun addDisplacementMarker(callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addDisplacementMarker(currentDisplacement, callback)
    }

    fun UNSTABLE_addDisplacementMarkerOffset(offset: Double, callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addDisplacementMarker(currentDisplacement + offset, callback)
    }

    private fun addDisplacementMarker(displacement: Double, callback: MarkerCallback): TrajectorySequenceBuilder {
        return this.addDisplacementMarker(0.0, displacement, callback)
    }

    private fun addDisplacementMarker(scale: Double, offset: Double, callback: MarkerCallback): TrajectorySequenceBuilder {
        return addDisplacementMarker(({ displacement -> scale * displacement + offset }), callback)
    }

    private fun addDisplacementMarker(
        displacement: DisplacementProducer,
        callback: MarkerCallback
    ): TrajectorySequenceBuilder {
        displacementMarkers.add(DisplacementMarker(displacement, callback))

        return this
    }

    @JvmOverloads
    fun turn(
        angle: Double,
        maxAngVel: Double = currentTurnConstraintMaxAngVel,
        maxAngAccel: Double = currentTurnConstraintMaxAngAccel
    ): TrajectorySequenceBuilder {
        pushPath()

        val turnProfile: MotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(lastPose.heading, 0.0, 0.0, 0.0),
            MotionState(lastPose.heading + angle, 0.0, 0.0, 0.0),
            maxAngVel,
            maxAngAccel
        )

        sequenceSegments.add(TurnSegment(lastPose, angle, turnProfile, mutableListOf()))

        lastPose = Pose2d(
            lastPose.x, lastPose.y,
            Angle.norm(lastPose.heading + angle)
        )

        currentDuration += turnProfile.duration()

        return this
    }

    fun waitSeconds(seconds: Double): TrajectorySequenceBuilder {
        pushPath()
        sequenceSegments.add(WaitSegment(lastPose, seconds, mutableListOf()))

        currentDuration += seconds
        return this
    }

    fun addTrajectory(trajectory: Trajectory): TrajectorySequenceBuilder {
        pushPath()

        sequenceSegments.add(TrajectorySegment(trajectory))
        return this
    }

    private fun pushPath() {
        if (currentTrajectoryBuilder != null) {
            val builtTraj: Trajectory = currentTrajectoryBuilder!!.build()
            sequenceSegments.add(TrajectorySegment(builtTraj))
        }

        currentTrajectoryBuilder = null
    }

    private fun newPath() {
        if (currentTrajectoryBuilder != null) pushPath()

        lastDurationTraj = 0.0
        lastDisplacementTraj = 0.0

        val tangent = if (setAbsoluteTangent) absoluteTangent else Angle.norm(lastPose.heading + tangentOffset)

        currentTrajectoryBuilder =
            TrajectoryBuilder(lastPose, tangent, currentVelConstraint, currentAccelConstraint, resolution)
    }

    fun build(): TrajectorySequence {
        pushPath()

        val globalMarkers = convertMarkersToGlobal(
            sequenceSegments,
            temporalMarkers, displacementMarkers, spatialMarkers
        )
        projectGlobalMarkersToLocalSegments(globalMarkers, sequenceSegments)

        return TrajectorySequence(sequenceSegments)
    }

    private fun convertMarkersToGlobal(
        sequenceSegments: List<SequenceSegment>,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): MutableList<TrajectoryMarker> {
        val trajectoryMarkers: java.util.ArrayList<TrajectoryMarker> = java.util.ArrayList<TrajectoryMarker>()

        // Convert temporal markers
        for (marker in temporalMarkers) {
            trajectoryMarkers.add(
                TrajectoryMarker(marker.producer.produce(currentDuration), marker.callback)
            )
        }

        // Convert displacement markers
        for (marker in displacementMarkers) {
            val time = displacementToTime(
                sequenceSegments,
                marker.producer.produce(currentDisplacement)
            )

            trajectoryMarkers.add(
                TrajectoryMarker(
                    time,
                    marker.callback
                )
            )
        }

        // Convert spatial markers
        for (marker in spatialMarkers) {
            trajectoryMarkers.add(
                TrajectoryMarker(
                    pointToTime(sequenceSegments, marker.point),
                    marker.callback
                )
            )
        }

        return trajectoryMarkers
    }

    private fun projectGlobalMarkersToLocalSegments(
        markers: MutableList<TrajectoryMarker>,
        sequenceSegments: List<SequenceSegment>
    ) {
        if (sequenceSegments.isEmpty()) return

        markers.sortWith(java.util.Comparator.comparingDouble(TrajectoryMarker::time))

        var timeOffset = 0.0
        var markerIndex = 0
        for (segment in sequenceSegments) {
            while (markerIndex < markers.size) {
                val marker: TrajectoryMarker = markers[markerIndex]
                if (marker.time >= timeOffset + segment.duration) {
                    break
                }

                segment.markers.add(
                    TrajectoryMarker(
                        0.0.coerceAtLeast(marker.time) - timeOffset, marker.callback
                    )
                )
                ++markerIndex
            }

            timeOffset += segment.duration
        }

        val segment: SequenceSegment = sequenceSegments[sequenceSegments.size - 1]
        while (markerIndex < markers.size) {
            val marker: TrajectoryMarker = markers[markerIndex]
            segment.markers.add(TrajectoryMarker(segment.duration, marker.callback))
            ++markerIndex
        }
    }

    // Taken from Road Runner's TrajectoryGenerator.displacementToTime() since it's private
    // note: this assumes that the profile position is monotonic increasing
    private fun motionProfileDisplacementToTime(profile: MotionProfile, s: Double): Double {
        var tLo = 0.0
        var tHi: Double = profile.duration()
        while (!(abs(tLo - tHi) < 1e-6)) {
            val tMid = 0.5 * (tLo + tHi)
            if (profile[tMid].x > s) {
                tHi = tMid
            } else {
                tLo = tMid
            }
        }
        return 0.5 * (tLo + tHi)
    }

    private fun displacementToTime(sequenceSegments: List<SequenceSegment>, s: Double): Double {
        var currentTime = 0.0
        var currentDisplacement = 0.0

        for (segment in sequenceSegments) {
            if (segment is TrajectorySegment) {
                val thisSegment: TrajectorySegment = segment

                val segmentLength: Double = thisSegment.trajectory.path.length()

                if (currentDisplacement + segmentLength > s) {
                    val target = s - currentDisplacement
                    val timeInSegment = motionProfileDisplacementToTime(
                        thisSegment.trajectory.profile,
                        target
                    )

                    return currentTime + timeInSegment
                } else {
                    currentDisplacement += segmentLength
                }
            }

            currentTime += segment.duration
        }

        return currentTime
    }

    private fun pointToTime(sequenceSegments: List<SequenceSegment>, point: Vector2d): Double {
        class ComparingPoints(
            val distanceToPoint: Double,
            val totalDisplacement: Double,
            val thisPathDisplacement: Double
        )

        val projectedPoints: MutableList<ComparingPoints> = ArrayList()

        for (segment in sequenceSegments) {
            if (segment is TrajectorySegment) {
                val thisSegment: TrajectorySegment = segment

                val displacement: Double = thisSegment.trajectory.path.project(point, 0.25)
                val projectedPoint: Vector2d = thisSegment.trajectory.path[displacement].vec()
                val distanceToPoint: Double = point.minus(projectedPoint).norm()

                var totalDisplacement = 0.0

                for (comparingPoint in projectedPoints) {
                    totalDisplacement += comparingPoint.totalDisplacement
                }

                totalDisplacement += displacement

                projectedPoints.add(ComparingPoints(distanceToPoint, displacement, totalDisplacement))
            }
        }

        var closestPoint: ComparingPoints? = null

        for (comparingPoint in projectedPoints) {
            if (closestPoint == null) {
                closestPoint = comparingPoint
                continue
            }

            if (comparingPoint.distanceToPoint < closestPoint.distanceToPoint) closestPoint = comparingPoint
        }

        return displacementToTime(sequenceSegments, closestPoint!!.thisPathDisplacement)
    }

    private fun interface AddPathCallback {
        fun run()
    }
}