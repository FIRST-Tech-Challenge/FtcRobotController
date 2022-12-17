@file:Suppress("ClassName")

package ftc.rouge.blacksmith.proxies

import com.acmerobotics.roadrunner.geometry.Pose2d
import ftc.rouge.blacksmith.util.invokeMethodInfer
import java.util.*

@PublishedApi
internal class _SampleMecanumDrive(private val drive: Any) {
    fun getBuilderProxy(startPose: Pose2d) =
        _TrajectorySequenceBuilder(
            drive.invokeMethodInfer("trajectorySequenceBuilder", startPose)
        )

    fun followTrajectorySequenceAsync(builder: Any) {
        drive.invokeMethodInfer<Any>("followTrajectorySequenceAsync", builder)
    }

    fun setPoseEstimate(pose: Pose2d) {
        drive.invokeMethodInfer<Any>("setPoseEstimate", pose)
    }
}
