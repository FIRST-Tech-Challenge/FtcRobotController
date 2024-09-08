package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import com.acmerobotics.roadrunner.util.Angle

class TurnSegment(
    startPose: Pose2d,
    totalRotation: Double,
    val motionProfile: MotionProfile,
    markers: MutableList<TrajectoryMarker>
) : SequenceSegment(
    motionProfile.duration(),
    startPose,
    Pose2d(
        startPose.x, startPose.y,
        Angle.norm(startPose.heading + totalRotation)
    ),
    markers
)