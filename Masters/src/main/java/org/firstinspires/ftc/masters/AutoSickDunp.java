package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

@Autonomous(name = "Sick Dunp")
public class AutoSickDunp extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        int i = 0;

        waitForStart();

        TrajectorySequence startForward = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d( new Vector2d(0,24), Math.toRadians(90)), Math.toRadians(90))
                .build();

        TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d( new Vector2d(0,24), Math.toRadians(90)), Math.toRadians(90))
                .build();

        TrajectorySequence back = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(startPose, Math.toRadians(270))
                .build();

//        while (i<12) {
        drive.followTrajectorySequence(startForward);

        drive.followTrajectorySequence(back);

            drive.followTrajectorySequence(forward);
//            sleep(1000);
            drive.followTrajectorySequence(back);
//            sleep(1000);
//
//            i++;
//        }



    }

}
