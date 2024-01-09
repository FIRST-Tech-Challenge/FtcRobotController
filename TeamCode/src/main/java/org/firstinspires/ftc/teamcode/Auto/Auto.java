package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class Auto extends CommandOpMode {

    private SampleMecanumDrive drive;
    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory = new TrajectorySequenceBuilder(startPose, null, null, 0, 0)
                .forward(12)
                .build();

        while(opModeInInit()){
            telemetry.update();
        }

        schedule(new TrajectorySequenceCommand(drive, trajectory));
    }
}
