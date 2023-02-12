package org.firstinspires.ftc.masters.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequenceRunnerCancelable;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Trajectory-cancelable version of the simple mecanum drive hardware implementation for REV hardware.
 * Ensure that this is copied into your project.
 */
@Config
public class SampleMecanumDriveCancelable extends SampleMecanumDrive {

    //private final TrajectorySequenceRunnerCancelable trajectorySequenceRunnerCancelable;


    public SampleMecanumDriveCancelable(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);

        trajectorySequenceRunner = new TrajectorySequenceRunnerCancelable(follower, HEADING_PID);
    }


    public void breakFollowing() {
        trajectorySequenceRunner.breakFollowing();
    }



}
