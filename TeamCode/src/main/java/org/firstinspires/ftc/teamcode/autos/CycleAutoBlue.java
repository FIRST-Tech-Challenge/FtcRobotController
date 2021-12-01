package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class CycleAutoBlue extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Carousel carousel = new Carousel(Color.BLUE);
        Lift lift = new Lift();
        Hopper hopper = new Hopper();
        Intake intake = new Intake();

        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        hopper.init(hardwareMap);
        intake.init(hardwareMap);


        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                .strafeRight(1.5)
                .back(15)
                .turn(Math.toRadians(40))
                .back(14)
                .build();

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .forward(14)
                .turn(Math.toRadians(50))
                .addTemporalMarker(() -> intake.intakeMotor.setPower(1))
                .forward(33)
                .addTemporalMarker(() -> intake.intakeMotor.setPower(0))
                .build();
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .back(33)
                .turn(Math.toRadians(-50))
                .back(14)
                .build();
        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
                .forward(14)
                .turn(Math.toRadians(50))
                .forward(30)
                .build();

        waitForStart();
        drive.followTrajectorySequence(trajectory1);
        lift.goTo(1450,0.8);
        delay(750);
        hopper.hopper.setPosition(0.33);
        delay(700);
        hopper.hopper.setPosition(0);
        lift.goTo(0,0.8);
        drive.followTrajectorySequence(trajectory2);
        drive.followTrajectorySequence(trajectory3);
        lift.goTo(1450,0.8);
        delay(750);
        hopper.hopper.setPosition(0.33);
        delay(700);
        hopper.hopper.setPosition(0);
        lift.goTo(0,0.8);
        drive.followTrajectorySequence(trajectory4);
    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
