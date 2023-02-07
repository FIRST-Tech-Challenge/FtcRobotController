@file:Suppress("MemberVisibilityCanBePrivate", "KDocUnresolvedReference", "unused")

package ftc.rogue.blacksmith

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import ftc.rogue.blacksmith.internal.*
import ftc.rogue.blacksmith.internal.invokeMethodRethrowing
import ftc.rogue.blacksmith.internal.proxies._SampleMecanumDrive
import ftc.rogue.blacksmith.units.GlobalUnits
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
class Anvil(drive: Any, private val startPose: Pose2d) {
    companion object {
        /**
         * Creates a new [Anvil] instance with the given [drive] and [startPose]. This is the entry
         * point for the builder API.
         *
         * Usage example:
         * ```kotlin
         * fun trajectory1(): Anvil =
         *     Anvil.forgeTrajectory(drive, startPose) {
         *         forward(24) // Note there are no '.'s, this uses
         *         turn(90) // Kotlin's 'lambda with reciever' syntax
         *     }
         *
         * fun trajectory2(): Anvil =
         *     Anvil.forgeTrajectory(drive, startPose)
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
        inline fun forgeTrajectory(
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
        fun startAutoWith(instance: Anvil) = AnvilLaunchConfig1().also {
            initialTrajectory = instance.setPoseEstimate(instance.startPose).build()
            initialInstance = instance
        }

        @JvmStatic
        @JvmOverloads
        fun start(async: Boolean = true) {
            if (!::initialTrajectory.isInitialized) {
                throw IllegalStateException("Anvil.startAutoWith() should be called before Anvil.start()")
            }
            initialInstance.run(initialTrajectory, async)
        }

        // Private coroutine scope used for async trajectory creation, dw about it
        private val builderScope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    }

    private val driveProxy = _SampleMecanumDrive(drive)

    private val builderProxy = driveProxy.getBuilderProxy(startPose)

    private val preforgedTrajectories = mutableMapOf<Any, Deferred<Any>>()

    private lateinit var builtTrajectory: Any

    private val builderDeque = ArrayDeque<BuilderAction>()

    // -- Direct path mappings (Basic) --

    fun forward(distance: Number) = queueAndReturnThis {
        builderProxy.forward(distance.toIn())
    }

    fun back(distance: Number) = queueAndReturnThis {
        builderProxy.back(distance.toIn())
    }

    fun turn(angle: Number) = queueAndReturnThis {
        builderProxy.turn(angle.toRad())
    }

    fun strafeLeft(distance: Number) = queueAndReturnThis {
        builderProxy.strafeLeft(distance.toIn())
    }

    fun strafeRight(distance: Number) = queueAndReturnThis {
        builderProxy.strafeRight(distance.toIn())
    }

    // -- Direct path mappings (Lines) --

    fun lineTo(x: Number, y: Number) = queueAndReturnThis {
        builderProxy.lineTo( GlobalUnits.vec(x, y) )
    }

    fun strafeTo(x: Number, y: Number): Anvil {
        return lineTo(x, y)
    }

    fun lineToConstantHeading(x: Number, y: Number): Anvil {
        return lineTo(x, y)
    }

    fun lineToLinearHeading(x: Number, y: Number, heading: Number) = queueAndReturnThis {
        builderProxy.lineToLinearHeading( GlobalUnits.pos(x, y, heading) )
    }

    fun lineToSplineHeading(x: Number, y: Number, heading: Number) = queueAndReturnThis {
        builderProxy.lineToSplineHeading( GlobalUnits.pos(x, y, heading) )
    }

    // -- Direct path mappings (Splines) --

    fun splineTo(x: Number, y: Number, endTangent: Number) = queueAndReturnThis {
        builderProxy.splineTo(
            GlobalUnits.vec(x, y),
            endTangent.toRad(),
        )
    }

    fun splineToConstantHeading(x: Number, y: Number, endTangent: Number) = queueAndReturnThis {
        builderProxy.splineToConstantHeading(
            GlobalUnits.vec(x, y),
            endTangent.toRad(),
        )
    }

    fun splineToLinearHeading(x: Number, y: Number, heading: Number, endTangent: Number) = queueAndReturnThis {
        builderProxy.splineToLinearHeading(
            GlobalUnits.pos(x, y, heading),
            endTangent.toRad(),
        )
    }

    fun splineToSplineHeading(x: Number, y: Number, heading: Number, endTangent: Number) = queueAndReturnThis {
        builderProxy.splineToSplineHeading(
            GlobalUnits.pos(x, y, heading),
            endTangent.toRad(),
        )
    }

    // -- Advanced mappings --

    fun waitTime(time: Number) = queueAndReturnThis {
        builderProxy.waitSeconds(time.toSec())
    }

    fun setReversed(reversed: Boolean) = queueAndReturnThis {
        builderProxy.setReversed(reversed)
    }

    fun setTangent(tangent: Number) = queueAndReturnThis {
        builderProxy.setTangent(tangent.toDouble())
    }

    fun addTrajectory(trajectory: Trajectory) = queueAndReturnThis {
        builderProxy.addTrajectory(trajectory)
    }

    fun addTrajectory(trajectory: () -> Trajectory) = queueAndReturnThis {
        builderProxy.addTrajectory(trajectory())
    }

    // -- Markers --

    @JvmOverloads
    fun addTemporalMarker(
        offset: Number = 0.0,
        action: MarkerCallback
    ) = queueAndReturnThis {
        builderProxy.UNSTABLE_addTemporalMarkerOffset(
            offset.toSec(),
            action,
        )
    }

    @JvmOverloads
    fun addDisplacementMarker(
        offset: Number = 0.0,
        action: MarkerCallback
    ) = queueAndReturnThis {
        builderProxy.UNSTABLE_addDisplacementMarkerOffset(
            offset.toSec(),
            action,
        )
    }

    fun addSpatialMarker(
        offsetX: Number,
        offsetY: Number,
        action: MarkerCallback
    ) = queueAndReturnThis {
        builderProxy.addSpatialMarker(
            GlobalUnits.vec(offsetX, offsetY),
            action,
        )
    }

    // -- Utilities --

    fun setPoseEstimate(pose: Pose2d) = this.apply {
        driveProxy.setPoseEstimate(pose)
    }

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
    fun inReverse(pathsToDoInReverse: AnvilConsumer) = this.apply {
        setReversed(true)

        with(pathsToDoInReverse) {
            this@Anvil.consume()
        }

        setReversed(false)
    }

    /**
     * Performs the last action in reverse.
     *
     * NOTE THAT THIS POPS THE LAST ITEM OFF THE STACK AND REVERTS THAT.
     *
     * For example, if you have something like this:
     * ```java
     * anvil
     *     .inReverse((builder) -> {
     *         builder.whatever(...);
     *     });
     * ```
     * the stack looks like this:
     *
     * -> setReversed(true)
     *
     * -> builder.whatever(...)
     *
     * -> setReversed(false)
     *
     * so if you try to do `.inReverse(() -> ...).doInReverse()`, that doesn't cancel out the
     * `.inReverse`. The stack would end up looking like this:
     *
     * -> setReversed(true)
     *
     * -> setReversed(true)
     *
     * -> setReversed(false)
     *
     * -> builder.whatever(...)
     *
     * -> setReversed(false)
     *
     * the `setReversed(false)` was popped and *that* was reversed, which, as you can probably
     * guess, is useless & does absolutely nothing.
     */
    fun doInReverse() = this.apply {
        val thingToDoInReverse = builderDeque.removeLast()
        inReverse {
            builderDeque += thingToDoInReverse
        }
    }

    /**
     * Does absolutly nothing. It's just a no-op.
     */
    @get:JvmName("noop")
    val noop: Anvil
        get() = queueAndReturnThis {}

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
    fun <T> withRawBuilder(builder: Consumer<T>) = queueAndReturnThis {
        with(builder) {
            (builderProxy.internalBuilder as T).consume()
        }
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
    fun doTimes(times: Int, pathsToDo: AnvilCycle) = this.apply {
        repeat(times) { iterationNum ->
            with(pathsToDo) {
                this@Anvil.doCycle(iterationNum)
            }
        }
    }

    // -- Constraints --

    fun resetConstraints() = queueAndReturnThis {
        builderProxy.resetConstraints()
    }

    fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint) = queueAndReturnThis {
        builderProxy.setVelConstraint(velConstraint)
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setVelConstraint(maxVel: Number, maxAngularVel: Number, trackWidth: Number) = queueAndReturnThis {
        builderProxy.setVelConstraint(driveProxy.getVelocityConstraint(maxVel, maxAngularVel, trackWidth))
    }

    fun resetVelConstraint() = queueAndReturnThis {
        builderProxy.resetVelConstraint()
    }

    fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint) = queueAndReturnThis {
        builderProxy.setAccelConstraint(accelConstraint)
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setAccelConstraint(maxAccel: Number) = queueAndReturnThis {
        builderProxy.setAccelConstraint(driveProxy.getAccelerationConstraint(maxAccel))
    }

    fun resetAccelConstraint() = queueAndReturnThis {
        builderProxy.resetAccelConstraint()
    }

    fun setTurnConstraint(maxAngVel: Number, maxAngAccel: Number) = queueAndReturnThis {
        builderProxy.setTurnConstraint(maxAngVel.toDouble(), maxAngAccel.toDouble())
    }

    fun resetTurnConstraint() = queueAndReturnThis {
        builderProxy.resetTurnConstraint()
    }

    // -- Building, creating, running --

    @JvmOverloads
    fun thenRun(
        nextTrajectory: (Pose2d) -> Anvil,
        configBuilder: AnvilConfigBuilder = AnvilRunConfig.DEFAULT
    ): Anvil {
        val config = AnvilRunConfig()
        configBuilder.run { AnvilRunConfig().build() }

        val nextStartPoseSupplier = config.startPoseSupplier ?: ::getEndPose

        val key = Any()

        if (!config.buildsSynchronously) {
            builderDeque.addFirst {
                builderProxy.UNSTABLE_addTemporalMarkerOffset(0.0) {
                    preforgedTrajectories[key] = builderScope.async { nextTrajectory( nextStartPoseSupplier() ).build() }
                }
            }
        }

        addTemporalMarker {
            if (!config.predicate()) return@addTemporalMarker

            val nextTrajectoryBuilt = if (config.buildsSynchronously) {
                nextTrajectory( nextStartPoseSupplier() ).build()
            } else {
                runBlocking { preforgedTrajectories[key]?.await() }!!
            }

            if (config.buildsSynchronously) {
                run( nextTrajectoryBuilt, !config.runsSynchronously )
            }
        }

        return this
    }

    @Suppress("UNCHECKED_CAST")
    fun <T : Any> build(): T {
        for (i in builderDeque.indices) {
            builderDeque.removeFirst().invoke()
        }
        return (builderProxy.build() as T).also { builtTrajectory = it }
    }

    // -- Internal --

    private fun queueAndReturnThis(builderAction: BuilderAction) = this.apply {
        builderDeque += builderAction
    }

    private fun run(trajectory: Any, async: Boolean) = if (async) {
        driveProxy.followTrajectorySequenceAsync(trajectory)
    } else {
        driveProxy.followTrajectorySequence(trajectory)
    }

    private fun getEndPose(): Pose2d {
        if (!::builtTrajectory.isInitialized) {
            throw IllegalStateException("No trajectory has been built yet")
        }

        return builtTrajectory.invokeMethodRethrowing("end")
    }
}
