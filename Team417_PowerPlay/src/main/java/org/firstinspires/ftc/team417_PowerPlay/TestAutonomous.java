package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team417_PowerPlay.drive.SampleMecanumDrive;

@Autonomous (name="strafe test")
public class TestAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(40, 40), Math.toRadians(0))
                .build();

        Trajectory traject1 = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(5, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, -24), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(41, -24, Math.toRadians(90)), Math.toRadians(0))
                //.splineTo(new Vector2d(41, -24), Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(47, -24), Math.toRadians(90))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traject1);
        telemetry.addData("heading", drive.getLocalizer().getPoseEstimate());
        telemetry.update();
        wait(3000);
    }
}
