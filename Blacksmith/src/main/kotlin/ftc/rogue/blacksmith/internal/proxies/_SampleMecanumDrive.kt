@file:Suppress("ClassName")

package ftc.rogue.blacksmith.internal.proxies

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import ftc.rogue.blacksmith.internal.getMethod
import ftc.rogue.blacksmith.internal.invokeMethodI
import java.util.*

@PublishedApi
internal class _SampleMecanumDrive(private val drive: Any) {
    fun followTrajectorySequence(builder: Any) {
        drive.invokeMethodI<Any?>("followTrajectorySequence", builder)
    }

    fun followTrajectorySequenceAsync(builder: Any) {
        drive.invokeMethodI<Any?>("followTrajectorySequenceAsync", builder)
    }

    fun setPoseEstimate(pose: Pose2d) {
        drive.invokeMethodI<Any?>("setPoseEstimate", pose)
    }

    fun getVelocityConstraint(maxVel: Number, maxAngularVel: Number, trackWidth: Number): TrajectoryVelocityConstraint {
        return drive::class.java.getMethod("getVelocityConstraint", Double::class.java, Double::class.java, Double::class.java)
            .let {
                it.invoke(null, maxVel.toDouble(), maxAngularVel.toDouble(), trackWidth.toDouble()) as TrajectoryVelocityConstraint
            }
    }

    fun getAccelerationConstraint(maxAccel: Number): TrajectoryAccelerationConstraint {
        return drive::class.java.getMethod("getAccelerationConstraint", Double::class.java)
            .let {
                it.invoke(null, maxAccel.toDouble()) as TrajectoryAccelerationConstraint
            }
    }
}
