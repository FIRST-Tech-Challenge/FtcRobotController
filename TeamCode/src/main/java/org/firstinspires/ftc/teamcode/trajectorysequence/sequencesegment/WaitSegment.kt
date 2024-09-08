package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker

class WaitSegment(pose: Pose2d, seconds: Double, markers: MutableList<TrajectoryMarker>) :
    SequenceSegment(seconds, pose, pose, markers)