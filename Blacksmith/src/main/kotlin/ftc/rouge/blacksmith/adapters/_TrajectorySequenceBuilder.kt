@file:Suppress("ClassName", "unused")

package ftc.rouge.blacksmith.adapters

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.DisplacementProducer
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.TimeProducer
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import ftc.rouge.blacksmith.util.invokeMethodRethrowing
import java.lang.reflect.InvocationTargetException

class _TrajectorySequenceBuilder(val internalBuilder: Any) {

    private fun getMethod(name: String, vararg args: Class<*>): java.lang.reflect.Method {
        return internalBuilder::class.java.getMethod(name, *args)
    }
    
    private fun returnThisAndRethrow(toDo: () -> Unit) = this.apply {
        try {
            toDo()
        } catch (e: InvocationTargetException) {
            throw e.targetException
        }
    }

    // -- START MACHINE GENERATED CODE --

    fun lineTo(endPosition: Vector2d) = returnThisAndRethrow {
        getMethod("lineTo", Vector2d::class.java).invoke(internalBuilder, endPosition)
    }

    fun lineToConstantHeading(endPosition: Vector2d) = returnThisAndRethrow {
        getMethod("lineToConstantHeading", Vector2d::class.java).invoke(internalBuilder, endPosition)
    }

    fun lineToLinearHeading(endPose: Pose2d) = returnThisAndRethrow {
        getMethod("lineToLinearHeading", Pose2d::class.java).invoke(internalBuilder, endPose)
    }

    fun lineToSplineHeading(endPose: Pose2d) = returnThisAndRethrow {
        getMethod("lineToSplineHeading", Pose2d::class.java).invoke(internalBuilder, endPose)
    }

    fun strafeTo(endPosition: Vector2d) = returnThisAndRethrow {
        getMethod("strafeTo", Vector2d::class.java).invoke(internalBuilder, endPosition)
    }

    fun forward(distance: Double) = returnThisAndRethrow {
        getMethod("forward", Double::class.java).invoke(internalBuilder, distance)
    }

    fun back(distance: Double) = returnThisAndRethrow {
        getMethod("back", Double::class.java).invoke(internalBuilder, distance)
    }

    fun strafeLeft(distance: Double) = returnThisAndRethrow {
        getMethod("strafeLeft", Double::class.java).invoke(internalBuilder, distance)
    }

    fun strafeRight(distance: Double) = returnThisAndRethrow {
        getMethod("strafeRight", Double::class.java).invoke(internalBuilder, distance)
    }

    fun splineTo(endPosition: Vector2d, endHeading: Double) = returnThisAndRethrow {
        getMethod("splineTo", Vector2d::class.java, Double::class.java).invoke(internalBuilder, endPosition, endHeading)
    }

    fun splineToConstantHeading(endPosition: Vector2d, endHeading: Double) = returnThisAndRethrow {
        getMethod("splineToConstantHeading", Vector2d::class.java, Double::class.java).invoke(internalBuilder, endPosition, endHeading)
    }

    fun splineToLinearHeading(endPose: Pose2d, endHeading: Double) = returnThisAndRethrow {
        getMethod("splineToLinearHeading", Pose2d::class.java, Double::class.java).invoke(internalBuilder, endPose, endHeading)
    }

    fun splineToSplineHeading(endPose: Pose2d, endHeading: Double) = returnThisAndRethrow {
        getMethod("splineToSplineHeading", Pose2d::class.java, Double::class.java).invoke(internalBuilder, endPose, endHeading)
    }

    fun setTangent(tangent: Double) = returnThisAndRethrow {
        getMethod("setTangent", Double::class.java).invoke(internalBuilder, tangent)
    }

    fun setReversed(reversed: Boolean) = returnThisAndRethrow {
        getMethod("setReversed", Boolean::class.java).invoke(internalBuilder, reversed)
    }

    fun resetConstraints() = returnThisAndRethrow {
        getMethod("resetConstraints", ).invoke(internalBuilder, )
    }

    fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint) = returnThisAndRethrow {
        getMethod("setVelConstraint", TrajectoryVelocityConstraint::class.java).invoke(internalBuilder, velConstraint)
    }

    fun resetVelConstraint() = returnThisAndRethrow {
        getMethod("resetVelConstraint", ).invoke(internalBuilder, )
    }

    fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint) = returnThisAndRethrow {
        getMethod("setAccelConstraint", TrajectoryAccelerationConstraint::class.java).invoke(internalBuilder, accelConstraint)
    }

    fun resetAccelConstraint() = returnThisAndRethrow {
        getMethod("resetAccelConstraint", ).invoke(internalBuilder, )
    }

    fun setTurnConstraint(maxAngVel: Double, maxAngAccel: Double) = returnThisAndRethrow {
        getMethod("setTurnConstraint", Double::class.java, Double::class.java).invoke(internalBuilder, maxAngVel, maxAngAccel)
    }

    fun resetTurnConstraint() = returnThisAndRethrow {
        getMethod("resetTurnConstraint", ).invoke(internalBuilder, )
    }

    fun addTemporalMarker(callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("addTemporalMarker", MarkerCallback::class.java).invoke(internalBuilder, callback)
    }

    fun UNSTABLE_addTemporalMarkerOffset(offset: Double, callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("UNSTABLE_addTemporalMarkerOffset", Double::class.java, MarkerCallback::class.java).invoke(internalBuilder, offset, callback)
    }

    fun addTemporalMarker(time: Double, callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("addTemporalMarker", Double::class.java, MarkerCallback::class.java).invoke(internalBuilder, time, callback)
    }

    fun addTemporalMarker(scale: Double, offset: Double, callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("addTemporalMarker", Double::class.java, Double::class.java, MarkerCallback::class.java).invoke(internalBuilder, scale, offset, callback)
    }

    fun addTemporalMarker(time: TimeProducer, callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("addTemporalMarker", TimeProducer::class.java, MarkerCallback::class.java).invoke(internalBuilder, time, callback)
    }

    fun addSpatialMarker(point: Vector2d, callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("addSpatialMarker", Vector2d::class.java, MarkerCallback::class.java).invoke(internalBuilder, point, callback)
    }

    fun addDisplacementMarker(callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("addDisplacementMarker", MarkerCallback::class.java).invoke(internalBuilder, callback)
    }

    fun UNSTABLE_addDisplacementMarkerOffset(offset: Double, callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("UNSTABLE_addDisplacementMarkerOffset", Double::class.java, MarkerCallback::class.java).invoke(internalBuilder, offset, callback)
    }

    fun addDisplacementMarker(displacement: Double, callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("addDisplacementMarker", Double::class.java, MarkerCallback::class.java).invoke(internalBuilder, displacement, callback)
    }

    fun addDisplacementMarker(scale: Double, offset: Double, callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("addDisplacementMarker", Double::class.java, Double::class.java, MarkerCallback::class.java).invoke(internalBuilder, scale, offset, callback)
    }

    fun addDisplacementMarker(displacement: DisplacementProducer, callback: MarkerCallback) = returnThisAndRethrow {
        getMethod("addDisplacementMarker", DisplacementProducer::class.java, MarkerCallback::class.java).invoke(internalBuilder, displacement, callback)
    }

    fun turn(angle: Double) = returnThisAndRethrow {
        getMethod("turn", Double::class.java).invoke(internalBuilder, angle)
    }

    fun turn(angle: Double, maxAngVel: Double, maxAngAccel: Double) = returnThisAndRethrow {
        getMethod("turn", Double::class.java, Double::class.java, Double::class.java).invoke(internalBuilder, angle, maxAngVel, maxAngAccel)
    }

    fun waitSeconds(seconds: Double) = returnThisAndRethrow {
        getMethod("waitSeconds", Double::class.java).invoke(internalBuilder, seconds)
    }

    fun addTrajectory(trajectory: Trajectory) = returnThisAndRethrow {
        getMethod("addTrajectory", Trajectory::class.java).invoke(internalBuilder, trajectory)
    }

    // -- END MACHINE GENERATED CODE --

    fun build(): Any {
        return internalBuilder.invokeMethodRethrowing("build", )
    }
}
