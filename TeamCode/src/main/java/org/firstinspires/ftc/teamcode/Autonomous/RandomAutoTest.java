package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

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
