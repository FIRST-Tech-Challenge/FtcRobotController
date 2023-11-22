package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Variables.Detection;
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;

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

        followTrajectorySequence(traj1);

        switch (detection) {
            case LEFT:
                followTrajectorySequence(
                        drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                                .turn(Math.toRadians(90))
                                .forward(2)
                                .build()
                );
                break;
            case CENTER:
                break;
            case RIGHT:
                followTrajectorySequence(
                        drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                                .turn(Math.toRadians(-90))
                                .forward(2)
                                .build()
                );
                break;
        }

//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}
