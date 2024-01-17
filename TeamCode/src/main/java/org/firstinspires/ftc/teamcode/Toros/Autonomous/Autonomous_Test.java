package org.firstinspires.ftc.teamcode.Toros.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriveRR.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto")
public class Autonomous_Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
    if(opModeIsActive()){
        Trajectory tra1 = drive.trajectoryBuilder(new Pose2d(12, 62, Math.toRadians(270)))
                .strafeLeft(15)
                .build();
        Trajectory tra2 = drive.trajectoryBuilder(tra1.end())
                .forward(15)
               .build();
        Trajectory tra3 = drive.trajectoryBuilder(tra2.end())
                .back(18)
                .splineToLinearHeading(new Pose2d(48, 42, Math.toRadians(180)),Math.toRadians(0))
               .build();
                TrajectorySequence tra4 = drive.trajectorySequenceBuilder(tra3.end())
              .strafeRight(18)
                        .back(6)
                        .build();
        drive.followTrajectory(tra1);
        drive.followTrajectory(tra2);
        drive.followTrajectory(tra3);
        //claw drop
        drive.followTrajectorySequence(tra4);
        while (opModeIsActive()){


        }
    }
    }
}
