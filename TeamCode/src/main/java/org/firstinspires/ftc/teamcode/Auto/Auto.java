package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class Auto extends CommandOpMode {
    public static TrajectorySequence parkRight;

    private SampleMecanumDrive drive;
    private Pose2d startPose = new Pose2d(12, -64, Math.toRadians(0));


    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        schedule(new BulkCacheCommand(hardwareMap));
        drive.setPoseEstimate(startPose);

        parkRight = drive.trajectorySequenceBuilder(startPose)
                .forward(48)
                .build();

        while(opModeInInit()){
            telemetry.update();
        }

        schedule(new TrajectorySequenceCommand(drive, parkRight));
    }
}
