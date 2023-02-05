package org.firstinspires.ftc.masters.trajectorySequence;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

@Config
public class TrajectorySequenceRunnerCancelable extends TrajectorySequenceRunner {


    public TrajectorySequenceRunnerCancelable(TrajectoryFollower follower, PIDCoefficients headingPIDCoefficients){
        super (follower, headingPIDCoefficients);
    }

    public void breakFollowing() {
        currentTrajectorySequence = null;
        remainingMarkers.clear();
    }
}
