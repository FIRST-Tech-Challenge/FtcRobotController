package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.Variables.*;
import java.util.ArrayList;

public class RandomAutoTest extends MeepMeepBoilerplate {
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        initVision(VisionProcessors.TFOD);
        Detection detection = getDetectionsSingleTFOD();


        waitForStart();

        followTrajectorySequence(
                drive.trajectorySequenceBuilder(STARTING_POSE)
                        .forward(28.0)
                        .build()
        );

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

//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}
