@file:Suppress("ClassName")

package ftc.rogue.blacksmith.internal.proxies

import com.acmerobotics.roadrunner.geometry.Pose2d
import ftc.rogue.blacksmith.util.invokeMethodI
import java.util.*

@PublishedApi
internal class _SampleMecanumDrive(private val drive: Any) {
    fun getBuilderProxy(startPose: Pose2d) =
        _TrajectorySequenceBuilder(
            drive.invokeMethodI("trajectorySequenceBuilder", startPose)
        )

    fun followTrajectorySequence(builder: Any) {
        drive.invokeMethodI<Any?>("followTrajectorySequence", builder)
    }

    fun followTrajectorySequenceAsync(builder: Any) {
        drive.invokeMethodI<Any?>("followTrajectorySequenceAsync", builder)
    }

    fun setPoseEstimate(pose: Pose2d) {
        drive.invokeMethodI<Any?>("setPoseEstimate", pose)
    }
}
