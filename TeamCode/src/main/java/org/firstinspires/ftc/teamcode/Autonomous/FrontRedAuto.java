package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.ArrayList;

@Config
@Autonomous(name = "FrontRedAuto", group = "Linear Opmode")
public class FrontRedAuto extends MeepMeepBoilerplate {
    enum Detection {
        LEFT,
        CENTER,
        RIGHT
    }

    @Override
    public void runOpMode() {
        Detection detection = Detection.RIGHT;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(28.0)
                        .build();

        assert getCurrentTrajectorySequence(drive) != null; // Null Pointer Protection

        switch (detection) {
            case LEFT:
                followTrajectorySequence(
                        drive.trajectorySequenceBuilder(getCurrentTrajectorySequence(drive).end())
                                .turn(Math.toRadians(90))
                                .forward(2)
                                .build()
                );
                break;
            case CENTER:
                break;
            case RIGHT:
                followTrajectorySequence(
                        drive.trajectorySequenceBuilder(getCurrentTrajectorySequence(drive).end())
                                .turn(Math.toRadians(-90))
                                .forward(2)
                                .build()
                );
                break;
        }

        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}
