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
 * [**LINK TO OFFICIAL DOCS (click on me) (please read) (I like cars)**](https://blacksmithftc.vercel.app/anvil/overview)
 *
 * A component that wraps around the [TrajectorySequenceBuilder] to provide a much cleaner API
 * with powerful features such as building trajectories in parallel for quick creation on the fly,
 * and implicitly converting units to any one you like! Program your auto in furlongs and arcseconds
 * if you'd like.
 *
 * Works well with the [Scheduler API][Scheduler].
 *
 * Basic example:
 * ```java
 * /* BaseAuto.java */
 *
 * abstract class BaseAuto extends BlackOp {
 *     @CreateOnGo
 *     protected AutoBotComponents bot;
 *
 *     @CreateOnGo(passHwMap = true)
 *     protected SampleMecanumDrive drive;
 *
 *     protected Pose2d startPose;
 *     protected abstract Anvil mainTraj(Pose2d startPose);
 *
 *     @Override
 *     public void go() {
 *         Anvil startTraj = mainTraj(startPose);
 *
 *         Anvil
 *              .startAutoWith(startTraj)
 *              .onSchedulerLaunch();
 *
 *         Scheduler.launchOnStart(this, () -> {
 *             drive.update();
 *             bot.update();
 *         });
 *     }
 * }
 *
 * /* AutoDemo.java */
 *
 * @Autonomous
 * class AutoDemo extends BaseAuto {
 *     public AutoDemo() {
 *         startPose = GlobalUnits.pos(x, y, r);
 *     }
 *
 *     @Override
 *     protected Anvil mainTraj(Pose2d startPose) {
 *         return Anvil.forgeTrajectory(drive, startPose)
 *             .forward(10)
 *             .addTemporalMarker(100, () -> {
 *                 // Do something
 *             })
 *             .back(10)
 *             .thenRun(this::parkTraj);
 *     }
 *
 *     private Anvil parkTraj(Pose2d startPose) {
 *         // Create and return parking trajectory
 *     }
 * }
 * ```
 *
 * @author KG
 *
 * @see Scheduler
 * @see TrajectorySequenceBuilder
 */
class Anvil
    internal constructor (
        drive: Any,
        startPose: Pose2d,
    ) {

    companion object {
        @JvmStatic
        @JvmOverloads
        fun forgeTrajectory(
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
        internal.doTimes(times, pathsToDo)
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

    private inline fun apply(action: () -> Unit) =
        also { action() }
}
