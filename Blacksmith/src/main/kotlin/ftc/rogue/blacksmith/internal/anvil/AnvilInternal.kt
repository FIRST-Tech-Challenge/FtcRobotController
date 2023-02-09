@file:Suppress("FunctionName")

package ftc.rogue.blacksmith.internal.anvil

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.internal.*
import ftc.rogue.blacksmith.internal.proxies._SampleMecanumDrive
import ftc.rogue.blacksmith.internal.proxies._TrajectorySequenceBuilder
import ftc.rogue.blacksmith.internal.util.*
import ftc.rogue.blacksmith.internal.util.invokeMethodRethrowing
import ftc.rogue.blacksmith.units.GlobalUnits
import ftc.rogue.blacksmith.util.toIn
import ftc.rogue.blacksmith.util.toRad
import ftc.rogue.blacksmith.util.toSec
import kotlinx.coroutines.*

/**
 * **PREFIXES:**
 * - '_' -> Directly adds to the end of the deque
 * - '__' -> Calls other functions that adds to the deque
 * - '$' -> Directly modifies the deque or something else special
 * - else -> Doesn't touch the deque
 */
class AnvilInternal
    internal constructor(
        private val instance: Anvil,
        drive: Any,
        @get:JvmSynthetic internal val startPose: Pose2d,
    ) {

    companion object {
        private val builderScope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    }

    private val driveProxy = _SampleMecanumDrive(drive)

    private val builderProxy = _TrajectorySequenceBuilder(driveProxy, startPose)

    private val builderDeque = ArrayDeque<() -> Unit>()

    private val preforgedTrajectories = mutableMapOf<Any, Deferred<Any>>()

    private lateinit var builtTrajectory: Any

    private fun add(builderAction: () -> Unit) {
        builderDeque += builderAction
    }

    @JvmSynthetic
    internal fun run(trajectory: Any, async: Boolean) = if (async) {
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

    // -- Direct path mappings (Basic) --

    fun _forward(distance: Number) = add {
        builderProxy.forward(distance.toIn())
    }

    fun _back(distance: Number) = add {
        builderProxy.back(distance.toIn())
    }

    fun _turn(angle: Number) = add {
        builderProxy.turn(angle.toRad())
    }

    fun _strafeLeft(distance: Number) = add {
        builderProxy.strafeLeft(distance.toIn())
    }

    fun _strafeRight(distance: Number) = add {
        builderProxy.strafeRight(distance.toIn())
    }

    // -- Direct path mappings (Lines) --

    fun _lineTo(x: Number, y: Number) = add {
        builderProxy.lineTo( GlobalUnits.vec(x, y) )
    }

    fun _lineToLinearHeading(x: Number, y: Number, heading: Number) = add {
        builderProxy.lineToLinearHeading( GlobalUnits.pos(x, y, heading) )
    }

    fun _lineToSplineHeading(x: Number, y: Number, heading: Number) = add {
        builderProxy.lineToSplineHeading( GlobalUnits.pos(x, y, heading) )
    }

    // -- Direct path mappings (Splines) --

    fun _splineTo(x: Number, y: Number, endTangent: Number) = add {
        builderProxy.splineTo(
            GlobalUnits.vec(x, y),
            endTangent.toRad(),
        )
    }

    fun _splineToConstantHeading(x: Number, y: Number, endTangent: Number) = add {
        builderProxy.splineToConstantHeading(
            GlobalUnits.vec(x, y),
            endTangent.toRad(),
        )
    }

    fun _splineToLinearHeading(x: Number, y: Number, heading: Number, endTangent: Number) = add {
        builderProxy.splineToLinearHeading(
            GlobalUnits.pos(x, y, heading),
            endTangent.toRad(),
        )
    }

    fun _splineToSplineHeading(x: Number, y: Number, heading: Number, endTangent: Number) = add {
        builderProxy.splineToSplineHeading(
            GlobalUnits.pos(x, y, heading),
            endTangent.toRad(),
        )
    }

    // -- Advanced mappings --

    fun _waitTime(time: Number) = add {
        builderProxy.waitSeconds(time.toSec())
    }

    fun _setReversed(reversed: Boolean) = add {
        builderProxy.setReversed(reversed)
    }

    fun _setTangent(tangent: Number) = add {
        builderProxy.setTangent(tangent.toDouble())
    }

    fun _addTrajectory(trajectory: Trajectory) = add {
        builderProxy.addTrajectory(trajectory)
    }

    fun _addTrajectory(trajectory: () -> Trajectory) = add {
        builderProxy.addTrajectory(trajectory())
    }

    // -- Markers --

    fun _addTemporalMarker(offset: Number, action: MarkerCallback) = add {
        builderProxy.UNSTABLE_addTemporalMarkerOffset(
            offset.toSec(),
            action,
        )
    }

    fun _addDisplacementMarker(offset: Number, action: MarkerCallback) = add {
        builderProxy.UNSTABLE_addDisplacementMarkerOffset(
            offset.toSec(),
            action,
        )
    }

    fun _addSpatialMarker(offsetX: Number, offsetY: Number, action: MarkerCallback) = add {
        builderProxy.addSpatialMarker(
            GlobalUnits.vec(offsetX, offsetY),
            action,
        )
    }

    // -- Utilities --

    fun setPoseEstimate(pose: Pose2d) {
        driveProxy.setPoseEstimate(pose)
    }

    fun __setPoseEstimateInTemporalMarker(pose: Pose2d) {
        _addTemporalMarker(0.0) {
            driveProxy.setPoseEstimate(pose)
        }
    }

    fun __inReverse(pathsToDoInReverse: AnvilConsumer) {
        _setReversed(true)
        pathsToDoInReverse.consume(instance)
        _setReversed(false)
    }

    fun `$doInReverse`() {
        val thingToDoInReverse = builderDeque.removeLast()

        __inReverse {
            thingToDoInReverse()
        }
    }

    fun _noop() = add {}

    @Suppress("UNCHECKED_CAST")
    fun <T> _withRawBuilder(builder: Consumer<T>) = add {
        (builderProxy.internalBuilder as T).let {
            builder.consume(it)
        }
    }

    fun doTimes(times: Int, pathsToDo: AnvilCycle) {
        repeat(times) { iteration ->
            pathsToDo.consume(instance, iteration)
        }
    }

    // -- Constraints --

    fun _resetConstraints() = add {
        builderProxy.resetConstraints()
    }

    fun _setVelConstraint(velConstraint: TrajectoryVelocityConstraint) = add {
        builderProxy.setVelConstraint(velConstraint)
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun _setVelConstraint(maxVel: Number, maxAngularVel: Number, trackWidth: Number) = add {
        builderProxy.setVelConstraint(driveProxy.getVelocityConstraint(maxVel, maxAngularVel, trackWidth))
    }

    fun _resetVelConstraint() = add {
        builderProxy.resetVelConstraint()
    }

    fun _setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint) = add {
        builderProxy.setAccelConstraint(accelConstraint)
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun _setAccelConstraint(maxAccel: Number) = add {
        builderProxy.setAccelConstraint(driveProxy.getAccelerationConstraint(maxAccel))
    }

    fun _resetAccelConstraint() = add {
        builderProxy.resetAccelConstraint()
    }

    fun _setTurnConstraint(maxAngVel: Number, maxAngAccel: Number) = add {
        builderProxy.setTurnConstraint(maxAngVel.toDouble(), maxAngAccel.toDouble())
    }

    fun _resetTurnConstraint() = add {
        builderProxy.resetTurnConstraint()
    }

    // -- Building, creating, running --

    @JvmOverloads
    fun `$thenRun`(
        nextTrajectory: (Pose2d) -> Anvil,
        configBuilder: AnvilRunConfigBuilder = AnvilRunConfig.DEFAULT
    ) {
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

        _addTemporalMarker(0.0) {
            if (!config.predicate()) return@_addTemporalMarker

            val nextTrajectoryBuilt = if (config.buildsSynchronously) {
                nextTrajectory( nextStartPoseSupplier() ).build()
            } else {
                runBlocking { preforgedTrajectories[key]?.await() }!!
            }

            if (config.buildsSynchronously) {
                run( nextTrajectoryBuilt, !config.runsSynchronously )
            }
        }
    }

    @Suppress("UNCHECKED_CAST", "unused")
    fun <T : Any> `$build`(): T {
        for (i in builderDeque.indices) {
            builderDeque.removeFirst().invoke()
        }

        return (builderProxy.build() as T).also { builtTrajectory = it }
    }
}
