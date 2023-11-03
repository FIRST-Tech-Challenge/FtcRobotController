package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;
import org.firstinspires.ftc.teamcode.Variables;

import java.util.ArrayList;

@Config
@Autonomous(name = "BackRedAuto", group = "LinearOpmode")
public class BackRedAuto extends MeepMeepBoilerplate {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Detection detection = getDetectionsSingleTFOD();
        initVision(VisionProcessors.TFOD);


        waitForStart();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(28.0)
                .build();

        assert getCurrentTrajectorySequence(drive) != null;

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
