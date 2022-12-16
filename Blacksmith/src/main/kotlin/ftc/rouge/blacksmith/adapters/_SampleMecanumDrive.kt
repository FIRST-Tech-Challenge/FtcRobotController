package ftc.rouge.blacksmith.adapters

import com.acmerobotics.roadrunner.geometry.Pose2d

@Suppress("ClassName")
class _SampleMecanumDrive(private val drive: Any) {
    fun getBuilder(startPose: Pose2d): _TrajectorySequenceBuilder =
        methods.first { it.name == "trajectorySequenceBuilder" }.invoke(drive, startPose)!!
            .run { _TrajectorySequenceBuilder(this) }

    fun followTrajectorySequenceAsync(builder: Any) =
        methods.first { it.name == "followTrajectorySequenceAsync" }.invoke(drive, builder)

    fun setPoseEstimate(pose: Pose2d) =
        methods.first { it.name == "setPoseEstimate" }.invoke(drive, pose)

    private val methods = drive::class.java.declaredMethods
}
