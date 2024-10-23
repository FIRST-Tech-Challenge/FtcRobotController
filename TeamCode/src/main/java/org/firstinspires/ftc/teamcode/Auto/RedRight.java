package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
@Autonomous
public class RedRight extends LinearOpMode {

    Pose2d startPose = new Pose2d(14,-61, Math.toRadians(90));
    boolean testingConditions = false;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        Drivetrain drive = new Drivetrain(this);
     //   Slides slides = new Slides(this);
    //    Arm arm = new Arm(this);

        drive.setPoseEstimate(startPose);
        TrajectorySequence trajectorySequenceStart = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, -34, Math.toRadians(90)))
                .waitSeconds(1)
                .build();
        TrajectorySequence outtakePreload = drive.trajectorySequenceBuilder(trajectorySequenceStart.end())
                .lineToLinearHeading(new Pose2d(10, -35, Math.toRadians(90)))
                .waitSeconds(1)
                .build();
        TrajectorySequence cycleGoToSubjects = drive.trajectorySequenceBuilder(outtakePreload.end())
                .lineToLinearHeading(new Pose2d(32, -35, Math.toRadians(90)))
                .waitSeconds(1)
                .build();
        TrajectorySequence cycleGrabSubjects = drive.trajectorySequenceBuilder(cycleGoToSubjects.end())
                .lineToLinearHeading(new Pose2d(60, -35, Math.toRadians(90)))
                .waitSeconds(1)
                .build();
        TrajectorySequence pathToOuttakeSubjects = drive.trajectorySequenceBuilder(cycleGrabSubjects.end())
                .lineToLinearHeading(new Pose2d(55, -35, Math.toRadians(90)))
                .waitSeconds(1)
                .build();
        TrajectorySequence outtakingSubjects = drive.trajectorySequenceBuilder(pathToOuttakeSubjects.end())
                .turn(-90)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(55, -55, Math.toRadians(0)))
                .build();

        drive.followTrajectorySequence(trajectorySequenceStart);
        drive.followTrajectorySequence(outtakePreload);
     //   slides.slidesLowBasket();
     //   arm.goOuttakePos();
        if (testingConditions) {
            drive.followTrajectorySequence(cycleGoToSubjects);
            drive.followTrajectorySequence(cycleGrabSubjects);
            drive.followTrajectorySequence(outtakingSubjects);
        }
    }
}
