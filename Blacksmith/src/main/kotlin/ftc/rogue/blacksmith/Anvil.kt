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
import ftc.rogue.blacksmith.internal.util.AnvilConsumer
import ftc.rogue.blacksmith.internal.util.AnvilCycle
import ftc.rogue.blacksmith.internal.util.AnvilRunConfigBuilder
import ftc.rogue.blacksmith.internal.util.Consumer
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
    @PublishedApi
    internal constructor (
        drive: Any,
        startPose: Pose2d,
    ) {

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

    fun forward(distance: Number) = tap {
        internal._forward(distance)
    }

    fun back(distance: Number) = tap {
        internal._back(distance)
    }

    fun turn(angle: Number) = tap {
        internal._turn(angle)
    }

    fun strafeLeft(distance: Number) = tap {
        internal._strafeLeft(distance)
    }

    fun strafeRight(distance: Number) = tap {
        internal._strafeRight(distance)
    }

    // -- Direct path mappings (Lines) --

    fun lineTo(x: Number, y: Number) = tap {
        internal._lineTo(x, y)
    }

    fun strafeTo(x: Number, y: Number) = tap {
        internal._lineTo(x, y)
    }

    fun lineToConstantHeading(x: Number, y: Number) = tap {
        internal._lineTo(x, y)
    }

    fun lineToLinearHeading(x: Number, y: Number, heading: Number) = tap {
        internal._lineToLinearHeading(x, y, heading)
    }

    fun lineToSplineHeading(x: Number, y: Number, heading: Number) = tap {
        internal._lineToSplineHeading(x, y, heading)
    }

    // -- Direct path mappings (Splines) --

    fun splineTo(x: Number, y: Number, endTangent: Number) = tap {
        internal._splineTo(x, y, endTangent)
    }

    fun splineToConstantHeading(x: Number, y: Number, endTangent: Number) = tap {
        internal._splineToConstantHeading(x, y, endTangent)
    }

    fun splineToLinearHeading(x: Number, y: Number, heading: Number, endTangent: Number) = tap {
        internal._splineToLinearHeading(x, y, heading, endTangent)
    }

    fun splineToSplineHeading(x: Number, y: Number, heading: Number, endTangent: Number) = tap {
        internal._splineToSplineHeading(x, y, heading, endTangent)
    }

    // -- Advanced mappings --

    fun waitTime(time: Number) = tap {
        internal._waitTime(time)
    }

    fun setReversed(reversed: Boolean) = tap {
        internal._setReversed(reversed)
    }

    fun setTangent(tangent: Number) = tap {
        internal._setTangent(tangent)
    }

    fun addTrajectory(trajectory: Trajectory) = tap {
        internal._addTrajectory(trajectory)
    }

    fun addTrajectory(trajectory: () -> Trajectory) = tap {
        internal._addTrajectory(trajectory)
    }

    // -- Markers --

    @JvmOverloads
    fun addTemporalMarker(offset: Number = 0.0, action: MarkerCallback) = tap {
        internal._addTemporalMarker(offset, action)
    }

    @JvmOverloads
    fun addDisplacementMarker(offset: Number = 0.0, action: MarkerCallback) = tap {
        internal._addDisplacementMarker(offset, action)
    }

    fun addSpatialMarker(offsetX: Number, offsetY: Number, action: MarkerCallback) = tap {
        internal._addSpatialMarker(offsetX, offsetY, action)
    }

    // -- Utilities --

    fun setPoseEstimateNow(pose: Pose2d) = tap {
        internal.setPoseEstimate(pose)
    }

    fun setPoseEstimateInTemporalMarker(pose: Pose2d) = tap {
        internal.__setPoseEstimateInTemporalMarker(pose)
    }

    fun inReverse(pathsToDoInReverse: AnvilConsumer) = tap {
        internal.__inReverse(pathsToDoInReverse)
    }

    fun doInReverse() = tap {
        internal.`$doInReverse`()
    }

    @get:JvmName("noop")
    val noop: Anvil
        get() = also { internal._noop() }

    fun <T> withRawBuilder(builder: Consumer<T>) = tap {
        internal._withRawBuilder(builder)
    }

    fun doTimes(times: Int, pathsToDo: AnvilCycle) = tap {
        internal.doTimes(times, pathsToDo)
    }

    // -- Constraints --

    fun resetConstraints() = tap {
        internal._resetConstraints()
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint) = tap {
        internal._setVelConstraint(velConstraint)
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setVelConstraint(maxVel: Number, maxAngularVel: Number, trackWidth: Number) = tap {
        internal._setVelConstraint(maxVel, maxAngularVel, trackWidth)
    }

    fun resetVelConstraint() = tap {
        internal._resetVelConstraint()
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint) = tap {
        internal._setAccelConstraint(accelConstraint)
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setAccelConstraint(maxAccel: Number) = tap {
        internal._setAccelConstraint(maxAccel)
    }

    fun resetAccelConstraint() = tap {
        internal._resetAccelConstraint()
    }

    /**
     * __IMPORTANT:__ These units are NOT auto-converted
     */
    fun setTurnConstraint(maxAngVel: Number, maxAngAccel: Number) = tap {
        internal._setTurnConstraint(maxAngVel, maxAngAccel)
    }

    fun resetTurnConstraint() = tap {
        internal._resetTurnConstraint()
    }

    // -- Building, creating, running --

    @JvmOverloads
    fun thenRun(
        nextTrajectory: (Pose2d) -> Anvil,
        configBuilder: AnvilRunConfigBuilder = AnvilRunConfig.DEFAULT
    ) = tap {
        internal.`$thenRun`(nextTrajectory, configBuilder)
    }

    fun <T> build(): T {
        return internal.`$build`()
    }

    // -- Internal --

    private inline fun tap(action: () -> Unit) =
        also { action() }
}
