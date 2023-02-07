@file:Suppress("MemberVisibilityCanBePrivate", "KDocUnresolvedReference", "unused")

package ftc.rogue.blacksmith

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import ftc.rogue.blacksmith.internal.*
import ftc.rogue.blacksmith.internal.anvil.AnvilInternal
import ftc.rogue.blacksmith.internal.anvil.AnvilRunConfig
import ftc.rogue.blacksmith.internal.anvil.AnvilRunner
import ftc.rogue.blacksmith.util.*
import kotlinx.coroutines.*

/**
 * A WIP component that wraps around the [TrajectorySequenceBuilder] to provide a much cleaner API
 * with powerful features such as preforming trajectories in parallel for quick creation on the fly.
 * Works well with the [Scheduler] API.
 *
 * Usage example:
 * ```kotlin
 * override fun runOpMode() {
 *     val startPose = Pose2d(91.toIn(), (-159).toIn(), 90.toRad())
 *     val startTraj = goForwardAndRaiseLift(startPose)
 *
 *     // Starts auto + sets drive's pose estimate to the startPose
 *     // Uses `drive.followTrajectorySequenceAsync` to run the trajectory in parallel,
 *     // so it should be updated every loop (Scheduler recommended for this).
 *     Anvil.startAsyncAutoWith(startTraj)
 *
 *     // Of course, you don't need to use Scheduler, but it's cool
 *     Scheduler.launch(this) {
 *         drive.update()
 *         // Other PID stuff and such...
 *     }
 * }
 *
 * var cycles = 0
 *
 * fun goForwardAndRaiseLift(startPose: Pose2d): Anvil {
 *    // Here pass in the SampleMechanumDrive
 *    return Anvil.forgeTrajectory(drive, startPose)
 *      // Concurrently creates the trajectory while still running the rest of the auto
 *      // This is useful for creating trajectories on the fly without stalling
 *      // the auto while the trajectory is being created.
 *
 *      // The `key` can be anything (comparable), it just needs to be unique for each
 *      // trajectory in this specific Anvil instance.
 *      .preform(key = 0, ::goBackwardAndLowerLift, startPose)
 *
 *      .temporalMarker {
 *          // ...
 *      }
 *      .forward(24)
 *
 *      // We can then use the key to run the next trajectory after the sequence is done
 *      // .runAsync simply creates a temporal marker that runs the given trajectory at the
 *      // end of this sequence.
 *      .runAsync(key = 0)
 * }
 *
 * // Note that you can just use Kotlin's single expression functions to make this even cleaner:
 * fun goBackwardAndLowerLift(startPose: Pose2d): Anvil =
 *    Anvil.forgeTrajectory(drive, startPose)
 *      .preform(key = "cycle again", ::goForwardAndRaiseLift, startPose))
 *      .preform(key = "park", ::park, startPose)
 *
 *      .temporalMarker {
 *          // ...
 *      }
 *      .back(24)
 *
 *      // You can conditionally run trajectories with this method, just pass
 *      // in a lambda that evaluates to a boolean.
 *      // Useful for creating loops or conditional sequences.
 *      .runAsyncIf(key = "cycle again") { cycles++ < 5 }
 *      .runAsync(key = "park")
 *
 * // There's a version of the builder API that accepts a builder lambda as well:
 * fun park(startPose: Pose2d): Anvil =
 *   Anvil.forgeTrajectory(drive, startPose) {
 *      when (coneSignalNum) {
 *          1 -> forward(.1)
 *          2 -> forward(10)
 *          else -> forward(20)
 *      }
 *   }
 * ```
 *
 * @param drive The [SampleMecanumDrive] to use for the trajectory
 * @param startPose The starting pose of the trajectory
 *
 * @author KG
 *
 * @see Scheduler
 * @see TrajectorySequenceBuilder
 */
class Anvil(drive: Any, startPose: Pose2d) {
    companion object {
        @JvmStatic
        @JvmOverloads
        inline fun forgeTrajectory(
            drive: Any,
            startPose: Pose2d,
            builder: Anvil.() -> Anvil = { this }
        ): Anvil {
            return builder( Anvil(drive, startPose) )
        }

        @JvmStatic
        fun startAutoWith(instance: Anvil): AnvilRunner {
            return AnvilRunner().startAutoWith(instance)
        }
    }

    @PublishedApi
    @get:JvmSynthetic
    internal val internal = AnvilInternal(this, drive, startPose)

    // -- Direct path mappings (Basic) --

    fun forward(distance: Number) = apply {
        internal._forward(distance)
    }

    fun back(distance: Number) = apply {
        internal._back(distance)
    }

    fun turn(angle: Number) = apply {
        internal._turn(angle)
    }

    fun strafeLeft(distance: Number) = apply {
        internal._strafeLeft(distance)
    }

    fun strafeRight(distance: Number) = apply {
        internal._strafeRight(distance)
    }

    // -- Direct path mappings (Lines) --

    fun lineTo(x: Number, y: Number) = apply {
        internal._lineTo(x, y)
    }

    fun strafeTo(x: Number, y: Number) = apply {
        internal._lineTo(x, y)
    }

    fun lineToConstantHeading(x: Number, y: Number) = apply {
        internal._lineTo(x, y)
    }

    fun lineToLinearHeading(x: Number, y: Number, heading: Number) = apply {
        internal._lineToLinearHeading(x, y, heading)
    }

    fun lineToSplineHeading(x: Number, y: Number, heading: Number) = apply {
        internal._lineToSplineHeading(x, y, heading)
    }

    // -- Direct path mappings (Splines) --

    fun splineTo(x: Number, y: Number, endTangent: Number) = apply {
        internal._splineTo(x, y, endTangent)
    }

    fun splineToConstantHeading(x: Number, y: Number, endTangent: Number) = apply {
        internal._splineToConstantHeading(x, y, endTangent)
    }

    fun splineToLinearHeading(x: Number, y: Number, heading: Number, endTangent: Number) = apply {
        internal._splineToLinearHeading(x, y, heading, endTangent)
    }

    fun splineToSplineHeading(x: Number, y: Number, heading: Number, endTangent: Number) = apply {
        internal._splineToSplineHeading(x, y, heading, endTangent)
    }

    // -- Advanced mappings --

    fun waitTime(time: Number) = apply {
        internal._waitTime(time)
    }

    fun setReversed(reversed: Boolean) = apply {
        internal._setReversed(reversed)
    }

    fun setTangent(tangent: Number) = apply {
        internal._setTangent(tangent)
    }

    fun addTrajectory(trajectory: Trajectory) = apply {
        internal._addTrajectory(trajectory)
    }

    fun addTrajectory(trajectory: () -> Trajectory) = apply {
        internal._addTrajectory(trajectory)
    }

    // -- Markers --

    @JvmOverloads
    fun addTemporalMarker(offset: Number = 0.0, action: MarkerCallback) = apply {
        internal._addTemporalMarker(offset, action)
    }

    @JvmOverloads
    fun addDisplacementMarker(offset: Number = 0.0, action: MarkerCallback) = apply {
        internal._addDisplacementMarker(offset, action)
    }

    fun addSpatialMarker(offsetX: Number, offsetY: Number, action: MarkerCallback) = apply {
        internal._addSpatialMarker(offsetX, offsetY, action)
    }

    // -- Utilities --

    fun setPoseEstimateNow(pose: Pose2d) = apply {
        internal.setPoseEstimate(pose)
    }

    fun setPoseEstimateInTemporalMarker(pose: Pose2d) = apply {
        internal.__setPoseEstimateInTemporalMarker(pose)
    }

    fun inReverse(pathsToDoInReverse: AnvilConsumer) = apply {
        internal.__inReverse(pathsToDoInReverse)
    }

    fun doInReverse() = apply {
        internal.`$doInReverse`()
    }

    @get:JvmName("noop")
    val noop: Anvil
        get() = also { internal._noop() }

    fun <T> withRawBuilder(builder: Consumer<T>) = apply {
        internal._withRawBuilder(builder)
    }

    fun doTimes(times: Int, pathsToDo: AnvilCycle) = apply {
        internal.doTimes(instance = this, times, pathsToDo)
    }

    // -- Constraints --

    fun resetConstraints() = apply {
        internal._resetConstraints()
    }

    fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint) = apply {
        internal._setVelConstraint(velConstraint)
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setVelConstraint(maxVel: Number, maxAngularVel: Number, trackWidth: Number) = apply {
        internal._setVelConstraint(maxVel, maxAngularVel, trackWidth)
    }

    fun resetVelConstraint() = apply {
        internal._resetVelConstraint()
    }

    fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint) = apply {
        internal._setAccelConstraint(accelConstraint)
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setAccelConstraint(maxAccel: Number) = apply {
        internal._setAccelConstraint(maxAccel)
    }

    fun resetAccelConstraint() = apply {
        internal._resetAccelConstraint()
    }

    fun setTurnConstraint(maxAngVel: Number, maxAngAccel: Number) = apply {
        internal._setTurnConstraint(maxAngVel, maxAngAccel)
    }

    fun resetTurnConstraint() = apply {
        internal._resetTurnConstraint()
    }

    // -- Building, creating, running --

    @JvmOverloads
    fun thenRun(
        nextTrajectory: (Pose2d) -> Anvil,
        configBuilder: AnvilRunConfigBuilder = AnvilRunConfig.DEFAULT
    ) = apply {
        internal.`$thenRun`(nextTrajectory, configBuilder)
    }

    fun <T> build(): T {
        return internal.`$build`()
    }

    // -- Internal --

    private inline fun apply(action: BuilderAction) = also {
        action()
    }
}
