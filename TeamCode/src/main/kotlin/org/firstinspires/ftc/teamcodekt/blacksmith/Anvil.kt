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
import org.firstinspires.ftc.teamcodekt.util.AngleUnit
import org.firstinspires.ftc.teamcodekt.util.DistanceUnit
import org.firstinspires.ftc.teamcodekt.util.toIn
import org.firstinspires.ftc.teamcodekt.util.toRad

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
 *     Scheduler.start(this) {
 *         drive.update()
 *         // Other PID stuff and such...
 *     }
 * }
 *
 * var cycles = 0
 *
 * fun goForwardAndRaiseLift(startPose: Pose2d): Anvil {
 *    // Here pass in the SampleMechanumDrive
 *    return Anvil.formTrajectory(drive, startPose)
 *      // Concurrently creates the trajectory while still running the rest of the auto
 *      // This is useful for creating trajectories on the fly without stalling
 *      // the auto while the trajectory is being created.
 *
 *      // The `key` can be anything (comparable), it just needs to be unique for each
 *      // trajectory in this specific Anvil instance.
 *      .preform(key = 0, ::goBackwardAndLowerLift, startPose)
 *
 *      .temporalMarker {
 *          // Raise lift
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
 *    Anvil.formTrajectory(drive, startPose)
 *      .preform(key = "cycle again", ::goForwardAndRaiseLift, startPose))
 *      .preform(key = "park", ::park, startPose)
 *
 *      .temporalMarker {
 *          // Lower lift
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
 *   Anvil.formTrajectory(drive, startPose) {
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
class Anvil(val drive: SampleMecanumDrive, private val startPose: Pose2d) {
    companion object {
        /**
         * Creates a new [Anvil] instance with the given [drive] and [startPose]. This is the entry
         * point for the builder API.
         *
         * Usage example:
         * ```kotlin
         * fun trajectory1(): Anvil =
         *     Anvil.formTrajectory(drive, startPose) {
         *         forward(24) // Note there are no '.'s, this uses
         *         turn(90) // Kotlin's 'lambda with reciever' syntax
         *     }
         *
         * fun trajectory2(): Anvil =
         *     Anvil.formTrajectory(drive, startPose)
         *         .forward(24) // Also usable without a lambda
         *         .turn(90) // (hi mom)
         * ```
         *
         * @param drive The [SampleMecanumDrive] to use for the trajectory
         * @param startPose The starting pose of the trajectory
         * @param builder The builder lambda
         *
         * @return The [Anvil] instance
         */
        @JvmStatic
        @JvmOverloads
        inline fun formTrajectory(
            drive: SampleMecanumDrive,
            startPose: Pose2d,
            builder: Anvil.() -> Anvil = { this }
        ): Anvil = builder(Anvil(drive, startPose))

        /**
         * Starts the given Anvil instance asynchronously. This is the entry point for the auto.
         * In doing so, it also sets the [SampleMecanumDrive.poseEstimate] to the [startPose]
         * the [Anvil] instance was created with.
         *
         * Usage example:
         * ```kotlin
         * val startPose: Pose2d = Pose2d(91.toIn(), (-159).toIn(), 90.toRad())
         * val startTraj: Anvil = trajectory1(startPose)
         *
         * // Keep in mind that as this is "async", this needs to be updated every loop
         *
         * // Equivalent to:
         * // drive.poseEstimate = startPose
         * // drive.followTrajectorySequenceAsync(startTraj(startPose).finish())
         * Anvil.startAsyncAutoWith(startTraj)
         * ```
         *
         * @param builder The [Anvil] instance to run asynchronously
         */
        @JvmStatic
        fun startAsyncAutoWith(builder: Anvil) {
            builder.setPoseEstimate().runAsync()
        }

        /**
         * Allows you to change the units of distance measurements in the builder API.
         *
         * Defaults to [DistanceUnit.INCHES]
         */
        var distanceUnit = DistanceUnit.INCHES

        /**
         * Allows you to change the units of angle measurements in the builder API.
         *
         * Defaults to [AngleUnit.RADIANS]
         */
        var angleUnit = AngleUnit.RADIANS

        // Private coroutine scope used for async trajectory creation, dw about it
        private val builderScope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    }

    /**
     * The [TrajectorySequenceBuilder] instance used to build the trajectory.
     */
    val trajectorySequenceBuilder = drive.trajectorySequenceBuilder(startPose)!!

    /**
     * The end position of the trajectory. This should NOT be accessed EITHER before the trajectory
     * is finished building, or unless the end pose was set manually.
     *
     * Intended for fine corrections by tricking the robot into thinking it's at a different position
     * to combat constant drift.
     */
    var endPose: Pose2d
        get() = _endPose ?: trajectorySequenceBuilder.build().end()
        set(value) {
            _endPose = value
        }

    // Internally stores the end pose if it was set manually
    private var _endPose: Pose2d? = null

    // Stores all of the preformed trajectories in this Anvil instance
    // Since the keys are 'Any', you can use whatever you want as a key
    private val preformedTrajectories = mutableMapOf<Any, Deferred<TrajectorySequence>>()

    /**
     * Maps to the [TrajectorySequenceBuilder.forward] method.
     */
    fun forward(distance: Double) = this.apply {
        trajectorySequenceBuilder.forward(distance.toIn(distanceUnit))
    }

    /**
     * Maps to the [TrajectorySequenceBuilder.back] method.
     */
    fun back(distance: Double) = this.apply {
        trajectorySequenceBuilder.back(distance.toIn(distanceUnit))
    }

    /**
     * Maps to the [TrajectorySequenceBuilder.turn] method.
     */
    fun turn(angle: Double) = this.apply {
        trajectorySequenceBuilder.turn(angle.toRad(angleUnit))
    }

    /**
     * Maps to the [TrajectorySequenceBuilder.splineTo] method.
     *
     * Note that it internally creates the [Vector2d] for you.
     */
    fun splineTo(x: Double, y: Double, heading: Double) = this.apply {
        trajectorySequenceBuilder.splineTo(Vector2d(x.toIn(distanceUnit), y.toIn(distanceUnit)), heading.toRad(angleUnit))
    }

    /**
     * Maps to the [TrajectorySequenceBuilder.waitSeconds] method.
     */
    fun waitSeconds(time: Double) = this.apply {
        trajectorySequenceBuilder.waitSeconds(time)
    }

    /**
     * Maps to the [TrajectorySequenceBuilder.setReversed] method.
     */
    fun setReversed(reversed: Boolean) = this.apply {
        trajectorySequenceBuilder.setReversed(reversed)
    }

    /**
     * Uses a 'lambda with receiver' to make running a path in reverse cleaner.
     *
     * Equivalent to:
     * ```kotlin
     * .setReversed(true)
     * .doStuff()
     * .setReversed(false)
     * ```
     *
     * Acutal usage example:
     * ```kotlin
     * .inReverse {
     *    doStuff() // Note no '.'s
     * }
     */
    inline fun inReverse(pathsToDoInReverse: Anvil.() -> Unit) = this.apply {
        setReversed(true)
        pathsToDoInReverse(this)
        setReversed(false)
    }

    /**
     * Allows for interation with the raw [TrajectorySequenceBuilder] instance to allow for usage
     * of methods not included in this class.
     *
     * Usage example:
     * ```kotlin
     * .withRawBuilder {
     *     splineToLinearHeading(stuff) // Again, no '.'s
     * }
     */
    inline fun withRawBuilder(builder: TrajectorySequenceBuilder.() -> Unit) = this.apply {
        builder(trajectorySequenceBuilder)
    }

    /**
     * Maps to the [TrajectorySequenceBuilder.UNSTABLE_addTemporalMarkerOffset] method.
     *
     * Note that if you don't pass in an `offset`, it's equivalent to the
     * [TrajectorySequenceBuilder.addTemporalMarker] method.
     */
    @JvmOverloads
    fun addTemporalMarker(
        offset: Double = 0.0,
        action: MarkerCallback
    ) = this.apply {
        trajectorySequenceBuilder.UNSTABLE_addTemporalMarkerOffset(offset, action)
    }

    /**
     * Switches to the trajectory formed by given [Anvil] instance when the function is reached
     * in the current trajectory. Useful for creating trajectories on the fly.
     *
     * Note that this is a blocking call, so it will wait for the trajectory to be formed before
     * switching to it. This can slow down your auto a bit as creating a trajectory may take time.
     *
     * I recommend using [preform] to speed up your auto.
     *
     * An additional manual `startPose` can be passed in to trick the robot into thinking it's at
     * a different position to combat constant drift.
     *
     * Usage example:
     * ```kotlin
     * fun goToPole(startPose: Pose2d): Anvil =
     *     Anvil.formTrajectory(drive, startPose)
     *         .forward(24.0)
     *         .turn(90.0)
     *         .thenRunAsync(::depositCone)
     *
     * fun depositCone(startPose: Pose2d): Anvil =
     *     // ...
     */
    @JvmOverloads
    inline fun thenRunAsync(
        crossinline nextTrajectory: (Pose2d) -> Anvil,
        startPose: Pose2d = endPose
    ) = this.addTemporalMarker {
        nextTrajectory(startPose).runAsync()
    }

    /**
     * Like the above, but only runs if the given predicate is `true`. Intended for
     * conditional paths, or creating loops.
     *
     * Usage example:
     * ```kotlin
     * var cycleNum;
     *
     * fun goForwards(startPose: Pose2d): Anvil =
     *     Anvil.formTrajectory(drive, startPose)
     *        .forward(24.0)
     *        .thenRunAsync(::goBackwards)
     *
     * fun goBackwards(startPose: Pose2d): Anvil =
     *     Anvil.formTrajectory(drive, startPose)
     *        .back(24.0)
     *        // Creates a loop that goes forwards and backwards thrice
     *        .thenRunAsyncIf(::goForwards) { cycleNum++ < 3 }
     *        .thenRUnAsync(::park)
     */
    @JvmOverloads
    inline fun thenRunAsyncIf(
        crossinline nextTrajectory: (Pose2d) -> Anvil,
        startPose: Pose2d = endPose,
        predicate: () -> Boolean
    ) = this.apply {
        if (predicate()) {
            thenRunAsync(nextTrajectory, startPose)
        }
    }

    /**
     * Preforming creates a new [TrajectorySequence] via [Anvil] in parallel to the current
     * trajectory using coroutines (like multi-threading). This allows for the on-the-fly creation
     * of trajectories without slowing down your auto as it may take a bit to create.
     *
     * A key (of [Any] type) is required to identify the trajectory. Using a key allows you to
     * preform multiple trajectories concurrently, and conditionally execute one later.
     *
     * *This should be called as early in the trajectory sequence as possible to maximize
     * the concurrent building time.*
     *
     * Usage example:
     * ```kotlin
     * fun goToPole(startPose: Pose2d): Anvil =
     *     Anvil.formTrajectory(drive, startPose)
     *        .preform(key = 0, ::depositCone)
     *
     *        .forward(24.0) // The 'depositCone' trajectory is being built concurrently
     *        .turn(90.0) // while these are being executed
     *
     *        .thenRunAsyncIf(key = 0) // This will run after the both goes forwards and turns
     */
    @JvmOverloads
    fun preform(
        key: Any,
        nextTrajectory: (Pose2d) -> Anvil,
        startPose: Pose2d = endPose
    ) = this.apply {
        preformedTrajectories[key] = builderScope.async { nextTrajectory(startPose).finish() }
    }

    /**
     * Like the other `thenRunAsync`, but runs a preformed trajectory associated with the given key.
     * *This is blocking and will wait for the trajectory to be finished creating if it's not
     * already created by the time the trajectory reaches this point (in the actual trajectory,
     * not just in the builder).*
     */
    fun thenRunAsync(key: Any) = this.addTemporalMarker {
        drive.followTrajectorySequenceAsync(
            runBlocking {
                preformedTrajectories[key]?.await()
            }
        )
    }

    /**
     * Love child of the above method and the other `thenRunAsyncIf`.
     */
    inline fun thenRunAsyncIf(key: Any, predicate: () -> Boolean) = this.apply {
        if (predicate()) {
            thenRunAsync(key)
        }
    }

    /**
     * Maps to the [TrajectorySequenceBuilder.build] method, and returns the resulting
     * [TrajectorySequence].
     */
    fun finish(): TrajectorySequence = trajectorySequenceBuilder.build()

    /**
     * Builds the [Anvil] instance into a [TrajectorySequence], and then runs it asynchronously
     * with the given [SampleMecanumDrive]
     */
    fun runAsync(): Unit = drive.followTrajectorySequenceAsync(finish())

    // Just used for the "static" 'startAsyncAutoWith' function
    private fun setPoseEstimate() = this.apply {
        drive.poseEstimate = startPose
    }
}
