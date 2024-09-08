package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment

import com.acmerobotics.roadrunner.trajectory.Trajectory

// Note: Markers are already stored in the `Trajectory` itself.
// This class should not hold any markers
class TrajectorySegment(val trajectory: Trajectory) :
    SequenceSegment(trajectory.duration(), trajectory.start(), trajectory.end(), mutableListOf())