@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.blacksmith

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.*
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.*
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcodekt.util.toIn
import org.firstinspires.ftc.teamcodekt.util.toRad

class Anvil(val drive: SampleMecanumDrive, startPose: Pose2d) {
    companion object {
        @JvmStatic
        fun formTrajectory(
            drive: SampleMecanumDrive,
            startPose: Pose2d,
            builder: Anvil.() -> Anvil
        ): Anvil {
            return builder(Anvil(drive, startPose))
        }

        private val builderScope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    }

    val trajectorySequenceBuilder = TrajectorySequenceBuilder(
        startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT, MAX_ANG_VEL, MAX_ANG_ACCEL
    )

    private var _endPose: Pose2d? = null

    var endPose: Pose2d
        get() = _endPose ?: trajectorySequenceBuilder.build().end()
        set(value) {
            _endPose = value
        }

    private val preconceivedTrajectories = mutableMapOf<Any, Deferred<TrajectorySequence>>()

    fun forward(distance: Double) = this.apply {
        trajectorySequenceBuilder.forward(distance.toIn())
    }

    fun back(distance: Double) = this.apply {
        trajectorySequenceBuilder.back(distance.toIn())
    }

    fun turn(angle: Double) = this.apply {
        trajectorySequenceBuilder.turn(angle.toRad())
    }

    fun splineTo(x: Double, y: Double, heading: Double) = this.apply {
        trajectorySequenceBuilder.splineTo(Vector2d(x.toIn(), y.toIn()), heading.toRad())
    }

    fun waitSeconds(time: Double) = this.apply {
        trajectorySequenceBuilder.waitSeconds(time)
    }

    fun setReversed(reversed: Boolean) = this.apply {
        trajectorySequenceBuilder.setReversed(reversed)
    }

    fun inReverse(pathsToDoInReverse: () -> Unit) = this.apply {
        setReversed(true)
        pathsToDoInReverse()
        setReversed(false)
    }

    fun withRawBuilder(builder: TrajectorySequenceBuilder.() -> Unit) = this.apply {
        builder(trajectorySequenceBuilder)
    }

    @JvmOverloads
    fun addTemporalMarker(
        offset: Double = 0.0,
        action: MarkerCallback
    ) = this.apply {
        trajectorySequenceBuilder.UNSTABLE_addTemporalMarkerOffset(offset, action)
    }

    @JvmOverloads
    fun thenRunAsync(
        nextTrajectory: (Pose2d) -> Anvil,
        startPose: Pose2d = endPose
    ) = this.addTemporalMarker {
        nextTrajectory(startPose).runAsync()
    }

    @JvmOverloads
    fun thenRunAsyncIf(
        nextTrajectory: (Pose2d) -> Anvil,
        startPose: Pose2d = endPose,
        predicate: () -> Boolean
    ) = this.apply {
        if (predicate()) {
            thenRunAsync(nextTrajectory, startPose)
        }
    }

    @JvmOverloads
    fun formTrajectoryInAdvance(
        key: Any,
        nextTrajectory: (Pose2d) -> Anvil,
        startPose: Pose2d = endPose
    ) = this.apply {
        preconceivedTrajectories[key] = builderScope.async { nextTrajectory(startPose).finish() }
    }

    fun thenRunAsync(key: Any) = this.addTemporalMarker {
        drive.followTrajectorySequenceAsync(
            runBlocking {
                preconceivedTrajectories[key]?.await()
            }
        )
    }

    fun thenRunAsyncIf(key: Any, predicate: () -> Boolean) = this.apply {
        if (predicate()) {
            thenRunAsync(key)
        }
    }

    fun finish(): TrajectorySequence = trajectorySequenceBuilder.build()

    fun runAsync(): Unit = drive.followTrajectorySequenceAsync(finish())
}
