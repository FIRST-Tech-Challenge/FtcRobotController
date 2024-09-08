package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker

abstract class SequenceSegment protected constructor(
    val duration: Double,
    val startPose: Pose2d,
    val endPose: Pose2d,
    val markers: MutableList<TrajectoryMarker>
)