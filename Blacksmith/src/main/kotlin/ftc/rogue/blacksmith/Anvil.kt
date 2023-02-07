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
class Anvil(drive: Any, @get:JvmSynthetic internal val startPose: Pose2d) {
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

        // Private coroutine scope used for async trajectory creation, dw about it
        private val builderScope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    }

    @PublishedApi
    @get:JvmSynthetic
    internal val internal = AnvilInternal(drive, startPose)

    // -- Direct path mappings (Basic) --

    fun forward(distance: Number) = action {
        internal._forward(distance)
    }

    fun back(distance: Number) = action {
        internal._back(distance)
    }

    fun turn(angle: Number) = action {
        internal._turn(angle)
    }

    fun strafeLeft(distance: Number) = action {
        internal._strafeLeft(distance)
    }

    fun strafeRight(distance: Number) = action {
        internal._strafeRight(distance)
    }

    // -- Direct path mappings (Lines) --

    fun lineTo(x: Number, y: Number) = action {
        internal._lineTo(x, y)
    }

    fun strafeTo(x: Number, y: Number) = action {
        internal._lineTo(x, y)
    }

    fun lineToConstantHeading(x: Number, y: Number) = action {
        internal._lineTo(x, y)
    }

    fun lineToLinearHeading(x: Number, y: Number, heading: Number) = action {
        internal._lineToLinearHeading(x, y, heading)
    }

    fun lineToSplineHeading(x: Number, y: Number, heading: Number) = action {
        internal._lineToSplineHeading(x, y, heading)
    }

    // -- Direct path mappings (Splines) --

    fun splineTo(x: Number, y: Number, endTangent: Number) = action {
        internal._splineTo(x, y, endTangent)
    }

    fun splineToConstantHeading(x: Number, y: Number, endTangent: Number) = action {
        internal._splineToConstantHeading(x, y, endTangent)
    }

    fun splineToLinearHeading(x: Number, y: Number, heading: Number, endTangent: Number) = action {
        internal._splineToLinearHeading(x, y, heading, endTangent)
    }

    fun splineToSplineHeading(x: Number, y: Number, heading: Number, endTangent: Number) = action {
        internal._splineToSplineHeading(x, y, heading, endTangent)
    }

    // -- Advanced mappings --

    fun waitTime(time: Number) = action {
        internal._waitTime(time)
    }

    fun setReversed(reversed: Boolean) = action {
        internal._setReversed(reversed)
    }

    fun setTangent(tangent: Number) = action {
        internal._setTangent(tangent)
    }

    fun addTrajectory(trajectory: Trajectory) = action {
        internal._addTrajectory(trajectory)
    }

    fun addTrajectory(trajectory: () -> Trajectory) = action {
        internal._addTrajectory(trajectory)
    }

    // -- Markers --

    @JvmOverloads
    fun addTemporalMarker(offset: Number = 0.0, action: MarkerCallback) = action {
        internal._addTemporalMarker(offset, action)
    }

    @JvmOverloads
    fun addDisplacementMarker(offset: Number = 0.0, action: MarkerCallback) = action {
        internal._addDisplacementMarker(offset, action)
    }

    fun addSpatialMarker(offsetX: Number, offsetY: Number, action: MarkerCallback) = action {
        internal._addSpatialMarker(offsetX, offsetY, action)
    }

    // -- Utilities --

    fun setPoseEstimateNow(pose: Pose2d) = action {
        internal.setPoseEstimate(pose)
    }

    fun setPoseEstimateInTemporalMarker(pose: Pose2d) = action {
        internal.__setPoseEstimateInTemporalMarker(pose)
    }

    fun inReverse(pathsToDoInReverse: AnvilConsumer) = action {
        internal.__inReverse(instance = this, pathsToDoInReverse)
    }

    fun doInReverse() = action {
        internal.`$doInReverse`()
    }

    @get:JvmName("noop")
    val noop: Anvil
        get() = also { internal._noop() }

    @Suppress("UNCHECKED_CAST")
    fun <T> withRawBuilder(builder: Consumer<T>) = action {
        internal._withRawBuilder(builder)
    }

    fun doTimes(times: Int, pathsToDo: AnvilCycle) = action {
        internal.doTimes(instance = this, times, pathsToDo)
    }

    // -- Constraints --

    fun resetConstraints() = action {
        internal._resetConstraints()
    }

    fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint) = action {
        internal._setVelConstraint(velConstraint)
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setVelConstraint(maxVel: Number, maxAngularVel: Number, trackWidth: Number) = action {
        internal._setVelConstraint(maxVel, maxAngularVel, trackWidth)
    }

    fun resetVelConstraint() = action {
        internal._resetVelConstraint()
    }

    fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint) = action {
        internal._setAccelConstraint(accelConstraint)
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setAccelConstraint(maxAccel: Number) = action {
        internal._setAccelConstraint(maxAccel)
    }

    fun resetAccelConstraint() = action {
        internal._resetAccelConstraint()
    }

    fun setTurnConstraint(maxAngVel: Number, maxAngAccel: Number) = action {
        internal._setTurnConstraint(maxAngVel, maxAngAccel)
    }

    fun resetTurnConstraint() = action {
        internal._resetTurnConstraint()
    }

    // -- Building, creating, running --

    @JvmOverloads
    fun thenRun(
        nextTrajectory: (Pose2d) -> Anvil,
        configBuilder: AnvilRunConfigBuilder = AnvilRunConfig.DEFAULT
    ) = action {
        internal.`$thenRun`(nextTrajectory, configBuilder)
    }

    fun <T> build(): T {
        return internal.`$build`()
    }

    // -- Internal --

    private inline fun action(action: BuilderAction) = apply {
        action()
    }
}
