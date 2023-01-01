@file:Suppress("MemberVisibilityCanBePrivate", "KDocUnresolvedReference", "unused")

package ftc.rogue.blacksmith

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import ftc.rogue.blacksmith.proxies._SampleMecanumDrive
import ftc.rogue.blacksmith.units.AngleUnit
import ftc.rogue.blacksmith.units.DistanceUnit
import ftc.rogue.blacksmith.units.TimeUnit
import ftc.rogue.blacksmith.util.*
import kotlinx.coroutines.*
import kotlin.math.PI

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
 *    Anvil.formTrajectory(drive, startPose)
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
class Anvil(drive: Any, private val startPose: Pose2d) {
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
            drive: Any,
            startPose: Pose2d,
            builder: Anvil.() -> Anvil = { this }
        ): Anvil {
            return builder( Anvil(drive, startPose) )
        }

        private lateinit var initialTrajectory: Any
        private lateinit var initialInstance: Anvil

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
        fun startAutoWith(instance: Anvil) {
            initialTrajectory = instance.setPoseEstimate().build()
        }

        @JvmStatic
        @JvmOverloads
        fun start(async: Boolean = true) {
            initialInstance.run(initialTrajectory, async)
        }

        /**
         * Allows you to change the units of distance measurements in the builder API.
         *
         * Defaults to [DistanceUnit.INCHES]
         */
        var distanceUnit = DistanceUnit.CM

        /**
         * Allows you to change the units of angle measurements in the builder API.
         *
         * Defaults to [AngleUnit.RADIANS]
         */
        var angleUnit = AngleUnit.RADIANS

        /**
         * Allows you to change the units of time measurements in the builder API.
         *
         * Defaults to [TimeUnit.SECONDS]
         */
        var timeUnit = TimeUnit.SECONDS

        /**
         * Changes all of the units in the builder API to the given units.
         */
        @JvmStatic
        fun setUnits(
            distanceUnit: DistanceUnit = DistanceUnit.CM,
            angleUnit: AngleUnit = AngleUnit.RADIANS,
            timeUnit: TimeUnit = TimeUnit.SECONDS,
        ) {
            this.distanceUnit = distanceUnit
            this.angleUnit = angleUnit
            this.timeUnit = timeUnit
        }

        @JvmStatic
        fun warmup() = WarmupHelper.start(100) // TODO: Test to see if this helps auto times

        // Private coroutine scope used for async trajectory creation, dw about it
        @PublishedApi
        internal val builderScope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    }

    @PublishedApi
    internal val driveProxy = _SampleMecanumDrive(drive)

    @PublishedApi
    internal val builderProxy = this.driveProxy.getBuilderProxy(startPose)

    @PublishedApi
    internal val preformedTrajectories = mutableMapOf<Any, Deferred<Any>>()

    @PublishedApi
    internal lateinit var builtTrajectory: Any

    // -- Direct path mappings (Basic) --

    fun forward(distance: Double) = this.apply {
        builderProxy.forward(distance.toIn)
    }

    fun back(distance: Double) = this.apply {
        builderProxy.back(distance.toIn)
    }

    fun turn(angle: Double) = this.apply {
        builderProxy.turn(angle.toRad)
    }

    fun strafeLeft(distance: Double) = this.apply {
        builderProxy.strafeLeft(distance.toIn)
    }

    fun strafeRight(distance: Double) = this.apply {
        builderProxy.strafeRight(distance.toIn)
    }

    // -- Direct path mappings (Lines) --

    fun lineToConstantHeading(x: Double, y: Double) = this.apply {
        builderProxy.strafeTo( Vector2d(x.toIn, y.toIn) )
    }

    fun lineToLinearHeading(x: Double, y: Double, heading: Double) = this.apply {
        builderProxy.lineToLinearHeading( Pose2d(x.toIn, y.toIn, heading.toRad) )
    }

    fun lineToSplineHeading(x: Double, y: Double, heading: Double) = this.apply {
        builderProxy.lineToSplineHeading( Pose2d(x.toIn, y.toIn, heading.toRad) )
    }

    // -- Direct path mappings (Splines) --

    fun splineTo(x: Double, y: Double, endTangent: Double) = this.apply {
        builderProxy.splineTo(Vector2d(x.toIn, y.toIn), endTangent.toRad)
    }

    fun splineToConstantHeading(x: Double, y: Double, endTangent: Double) = this.apply {
        builderProxy.splineToConstantHeading( Vector2d(x.toIn, y.toIn), endTangent.toRad)
    }

    fun splineToLinearHeading(x: Double, y: Double, heading: Double, endTangent: Double) = this.apply {
        builderProxy.splineToLinearHeading( Pose2d(x.toIn, y.toIn, heading.toRad), endTangent.toRad)
    }

    fun splineToSplineHeading(x: Double, y: Double, heading: Double, endTangent: Double) = this.apply {
        builderProxy.splineToSplineHeading( Pose2d(x.toIn, y.toIn, heading.toRad), endTangent.toRad)
    }

    // -- Advanced mappings --

    fun waitTime(time: Number) = this.apply {
        builderProxy.waitSeconds(time.toSec)
    }

    fun setReversed(reversed: Boolean) = this.apply {
        builderProxy.setReversed(reversed)
    }

    fun setTangent(tangent: Double) = this.apply {
        builderProxy.setTangent(tangent)
    }

    fun addTrajectory(trajectory: Trajectory) = this.apply {
        builderProxy.addTrajectory(trajectory)
    }

    fun addTrajectory(trajectory: () -> Trajectory) = this.apply {
        builderProxy.addTrajectory(trajectory())
    }

    // -- Markers --

    @JvmOverloads
    fun addTemporalMarker(
        offset: Double = 0.0,
        action: MarkerCallback
    ) = this.apply {
        builderProxy.UNSTABLE_addTemporalMarkerOffset(offset, action)
    }

    @JvmOverloads
    fun addDisplacementMarker(
        offset: Double = 0.0,
        action: MarkerCallback
    ) = this.apply {
        builderProxy.UNSTABLE_addDisplacementMarkerOffset(offset, action)
    }

    fun addSpatialMarker(
        offset: Vector2d,
        action: MarkerCallback
    ) = this.apply {
        builderProxy.addSpatialMarker(offset, action)
    }

    // -- Utilities --

    /**
     * Provides a new scope in which the trajectories are reversed.
     *
     * Usage example:
     * ```
     * // Java:                        |   // Kotlin:
     * anvil                           |   anvil
     *     .inReverse((builder) -> {   |       .inReverse {
     *         builder.splineTo(...);  |          splineTo(...)
     *     });                         |       }
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
     * ```
     * // Java:
     * anvil
     *    .<TrajectorySequenceBuilder>rawBuilder((builder) -> {
     *        builder.setTangent(...);
     *    };
     *
     * // Kotlin:
     * anvil
     *     .withRawBuilder<TrajectorySequenceBuilder> {
     *         setTangent(...)
     *     }
     */
    @Suppress("UNCHECKED_CAST")
    inline fun <T> withRawBuilder(builder: T.() -> Unit) = this.apply {
        builder(builderProxy.internalBuilder as T)
    }

    /**
     * Does the things to the builder `n` times.
     *
     * Usage example:
     * ```
     * // (Goes forwards and backwards 3 times)
     * // Java:                         |   // Kotlin:
     * anvil                            |   anvil
     *     .doTimes(3, (builder) -> {   |       .doTimes(3) {
     *         builder.forward(...);    |          forward(...)
     *         builder.back(...);       |          back(...)
     *     });                          |       }
     */
    inline fun doTimes(times: Int, pathsToDo: Anvil.(Int) -> Unit) = this.apply {
        repeat(times) {
            pathsToDo(this, it)
        }
    }

    // -- Constraints --

    fun resetConstraints() = this.apply {
        builderProxy.resetConstraints()
    }

    fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint) = this.apply {
        builderProxy.setVelConstraint(velConstraint)
    }

    fun resetVelConstraint() = this.apply {
        builderProxy.resetVelConstraint()
    }

    fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint) = this.apply {
        builderProxy.setAccelConstraint(accelConstraint)
    }

    fun resetAccelConstraint() = this.apply {
        builderProxy.resetAccelConstraint()
    }

    fun setTurnConstraint(maxAngVel: Double, maxAngAccel: Double) = this.apply {
        builderProxy.setTurnConstraint(maxAngVel, maxAngAccel)
    }

    fun resetTurnConstraint() = this.apply {
        builderProxy.resetTurnConstraint()
    }

    // -- Building, creating, running --

    /**
     * Preforming creates a new [TrajectorySequence] via [Anvil] in parallel to the current
     * trajectory using coroutines (like multi-threading). This allows for the on-the-fly creation
     * of trajectories without slowing down your auto as TrajectorySequences may take a notable
     * amount of time to build, especially if they're long (as in they cover a not insignificant
     * distance).
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
     *         .preform(key = 0, ::depositCone) // Starts when actual traj reaches this point
     *
     *         .forward(24.0) // The 'depositCone' trajectory is being built concurrently
     *         .turn(90.0) // while these are being executed
     *
     *         .thenRunAsync(key = 0) // This will run after the both goes forwards and turns
     */
    @JvmOverloads
    inline fun preform(
        key: Any,
        crossinline nextTrajectory: (Pose2d) -> Anvil,
        crossinline nextStartPose: () -> Pose2d = ::getEndPose
    ): Anvil {
        addTemporalMarker {
            preformedTrajectories[key] = builderScope.async { nextTrajectory( nextStartPose() ).build() }
        }
        return this
    }

    /**
     * Runs a preformed trajectory when the current trajectory reaches this function.
     *
     * Usage example:
     * ```kotlin
     * anvil
     *     .preform(key, ...)
     *     .forward(...)  // Goes forwards like normal
     *     .thenRunPreformed(key) // Runs the preformed trajectory after going forwards
     *     .back(...)  // This is ignored as the drive switches trajectories
     *
     */
    @JvmOverloads
    fun thenRunPreformed(
        key: Any,
        async: Boolean = true,
    ) = this.apply {
        thenRunPreformedIf(key, async) { true }
    }

    /**
     * Conditionally runs a preformed trajectory when the current trajectory reaches this function.
     * This is useful for loops or conditional trajectories.
     *
     * The given lambda is only evaluated when the trajectory reaches this point.
     *
     * Usage example:
     * ```kotlin
     * anvil
     *     .preform(key, ...)
     *     .forward(...)  // Goes forwards like normal
     *     .thenRunPreformedIf(key) { false }  // Doesn't go anything as the condition is false
     *     .back(...)  // Goes back like normal
     */
    @JvmOverloads
    inline fun thenRunPreformedIf(
        key: Any,
        async: Boolean = true,
        crossinline predicate: () -> Boolean
    ): Anvil {
        addTemporalMarker {
            if (key !in preformedTrajectories) {
                throw IllegalArgumentException("No preformed trajectory with key '$key'")
            }

            var followerFunction: (Any) -> Unit = driveProxy::followTrajectorySequence

            if (async) { // Have to do this or else the compiler crashes for some reason
                followerFunction = driveProxy::followTrajectorySequenceAsync
            }

//            val followerFunction: (Any) -> Unit = if (async) {    Does anyone know why
//                driveProxy::followTrajectorySequenceAsync         this crashes it?
//            } else {
//                driveProxy::followTrajectorySequence              org.jetbrains.kotlin.backend.common.BackendException:
//            }                                                     Backend Internal error: Exception during psi2ir

//            The root cause java.lang.NullPointerException was thrown at:
//            org.jetbrains.kotlin.psi2ir.generators.ReflectionReferencesGenerator.generateCallableReference(ReflectionReferencesGenerator.kt:75)
//            null: KtCallableReferenceExpression: driveProxy::followTrajectorySequenceAsync

            if (predicate()) {
                followerFunction(
                    runBlocking { preformedTrajectories[key]?.await() }!!
                )
            }
        }
        return this
    }

    /**
     * Syntactic sugar for:
     * ```
     * anvil
     *     .thenRunPreformedIf(key) { condition }
     *     .thenRunPreformedIf(key) { !condition }
     */
    @JvmOverloads
    inline fun thenBranchPreformed(
        trueKey: Any,
        elseKey: Any,
        async: Boolean = true,
        crossinline predicate: () -> Boolean,
    ) = this.apply {
        thenRunPreformedIf(trueKey, async) { predicate() }
        thenRunPreformed(elseKey, async)
    }

    @JvmOverloads
    inline fun thenRun(
        crossinline action: () -> Anvil,
        async: Boolean = true,
    ) = this.apply {
        thenRunIf(action, async) { true }
    }

    @JvmOverloads
    inline fun thenRunIf(
        crossinline action: () -> Anvil,
        async: Boolean = true,
        crossinline predicate: () -> Boolean,
    ): Anvil {
        addTemporalMarker {
            if (predicate()) {
                action().build<Any>()
                run(builtTrajectory, async)
            }
        }
        return this
    }

    @JvmOverloads
    inline fun thenBranch(
        crossinline trueAction: () -> Anvil,
        crossinline elseAction: () -> Anvil,
        async: Boolean = true,
        crossinline predicate: () -> Boolean,
    ) = this.apply {
        thenRunIf(trueAction, async) { predicate() }
        thenRun(elseAction, async)
    }

    /**
     * Maps to the [TrajectorySequenceBuilder.build] method, and returns the resulting
     * [TrajectorySequence].
     */
    @Suppress("UNCHECKED_CAST")
    fun <T : Any> build(): T = (builderProxy.build() as T).also { builtTrajectory = it }

    // -- Internal --

    private fun setPoseEstimate() = this.apply {
        driveProxy.setPoseEstimate(startPose)
    }

    @PublishedApi
    internal fun run(trajectory: Any, async: Boolean = true) = if (async) {
        driveProxy.followTrajectorySequenceAsync(trajectory)
    } else {
        driveProxy.followTrajectorySequence(trajectory)
    }

    private val Number.toIn get() = distanceUnit.toIn(this.toDouble())

    private val Number.toRad get() = angleUnit.toRad(this.toDouble())

    private val Number.toSec get() = timeUnit.toSec(this.toDouble())

    @PublishedApi
    internal fun getEndPose() = builtTrajectory.invokeMethodRethrowing<Pose2d>("end")

    private object WarmupHelper {
        fun start(numTimes: Int) = repeat(numTimes) {
            createAndBuildSequence()
        }

        private val dummyDrive = Any()

        private fun createAndBuildSequence() =
            formTrajectory(dummyDrive, Pose2d()) {
                forward(rand())
                turn(rand())
                addTemporalMarker(rand()) {
                }
                waitTime(rand())
                inReverse {
                    splineTo(rand(), rand(), rand())
                }
                doTimes(randInt()) {
                    forward(rand())
                }
                lineToLinearHeading(rand(), rand(), rand())
                splineTo(rand(), rand(), rand())
                back(rand())
                setReversed(true)
            }.build<Any>()

        private fun rand() = Math.random() * 100

        private fun randInt() = (Math.random() * 5).toInt()
    }
}
