package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.drive.*;


@Autonomous(name = "LeftAuto", group = "Taus2022-23")
public class RoadRunnerLeft extends LinearOpMode {

    private ElapsedTime timer = new ElapsedTime();
    private Vector2d myVector = new Vector2d(10, -5);
    private Pose2d myPose = new Pose2d(10, -5, Math.toRadians(90));
    private SampleMecanumDrive drive;
    private ManipulatorMethods manipulator;

    int START_TICKS = (int)(271/1.5);
    int INTAKE_TICKS = 0;
    int LOW_TICKS = (int)(2063/1.5);
    int MID_TICKS = (int)(3500/1.5);
    int HIGH_TICKS = (int)(4900/1.5);


    static final double FEET_PER_METER = 3.28084;


    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        manipulator = new ManipulatorMethods(hardwareMap);
        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("Waiting for start");
        telemetry.update();
        timer.reset();

        drive.setPoseEstimate(new Pose2d(-36, -63, Math.toRadians(90)));

        telemetry.addLine("SetPose");
        telemetry.update();

        /*Trajectory pushCone = drive.trajectoryBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                .forward(55)
                .build();
        Trajectory pushConeBack = drive.trajectoryBuilder(pushCone.end())
                .back(5)
                .build();*/
        /*Trajectory firstMidPole = drive.trajectoryBuilder(pushConeBack.end())
                .lineToLinearHeading(new Pose2d(-39, -24, 0))
                .build();*/
        Trajectory firstMidPole = drive.trajectoryBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-28.5, -19.5, Math.toRadians(315)), Math.toRadians(315))
                .build();
        telemetry.addLine("Traj1");
        telemetry.update();
        /*Trajectory firstMidPoleForward = drive.trajectoryBuilder(firstMidPole.end())
                .forward(6.5)
                .build();*/
       Trajectory firstMidPoleBack = drive.trajectoryBuilder(firstMidPole.end())
                .back(6)
                .build();
        telemetry.addLine("Traj2");
        telemetry.update();
        Trajectory stackMidPoint = drive.trajectoryBuilder(firstMidPole.end())
                .splineToLinearHeading(new Pose2d(-38, -12, Math.toRadians(180)), Math.toRadians(270))
                .build();
        telemetry.addLine("Traj3");
        telemetry.update();
        Trajectory stack = drive.trajectoryBuilder(stackMidPoint.end())
                .lineToLinearHeading(new Pose2d(-64, -12, Math.toRadians(180)))
                .build();
        telemetry.addLine("Traj4");
        telemetry.update();
        Trajectory stackBack = drive.trajectoryBuilder(stack.end())
                .back(4)
                .build();
        telemetry.addLine("Traj5");
        telemetry.update();
        Trajectory secMidPole = drive.trajectoryBuilder(stackBack.end())
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(320)))
                .build();
        telemetry.addLine("Traj6");
        telemetry.update();
        Trajectory secMidPoleForward = drive.trajectoryBuilder(secMidPole.end())
                .forward(8)
                .build();
        telemetry.addLine("Traj7");
        telemetry.update();
        Trajectory secMidPoleBack = drive.trajectoryBuilder(secMidPoleForward.end())
                .back(8)
                .build();
        telemetry.addLine("Traj8");
        telemetry.update();
        Trajectory stack2 = drive.trajectoryBuilder(secMidPoleBack.end())
                .lineToLinearHeading(new Pose2d(-64, -12, Math.toRadians(180)))
                .build();

        telemetry.addLine("Built Trajs");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Start requested");
        telemetry.update();
        if(isStopRequested()) return;
        manipulator.resetSlides();
        manipulator.moveSlideEncoder(MID_TICKS, 1);

//        drive.followTrajectory(traj);
//        manipulator.moveSlideEncoder(MID_TICKS, 1);
//        drive.followTrajectory(pushConeBack);
        //drive.followTrajectory(firstMidPole);
        drive.followTrajectory(firstMidPole);
        manipulator.outtake();
        sleep(1250);
        manipulator.stopIntake();
//        drive.followTrajectory(firstMidPoleBack);
        manipulator.moveSlideEncoder(LOW_TICKS, 1);

        //drive.turn(Math.toRadians(90));
        drive.followTrajectory(stackMidPoint);
        drive.followTrajectory(stack);
        manipulator.moveSlideEncoder(LOW_TICKS - 850, 1);
        manipulator.intake();
        sleep(1000);
        manipulator.stopIntake();
        manipulator.moveSlideEncoder(LOW_TICKS, 1);
        sleep(1000);
        drive.followTrajectory(stackBack);
        manipulator.moveSlideEncoder(MID_TICKS, 1);
        drive.followTrajectory(secMidPole);
        drive.followTrajectory(secMidPoleForward);
        manipulator.outtake();
        sleep(1250);
        manipulator.stopIntake();
        drive.followTrajectory(secMidPoleBack);
        manipulator.moveSlideEncoder(LOW_TICKS - 100, 1);
        drive.followTrajectory(stack2);
        manipulator.moveSlideEncoder(LOW_TICKS - 1050, 1);
        manipulator.intake();
        sleep(1250);
        manipulator.stopIntake();
        manipulator.moveSlideEncoder(LOW_TICKS, 1);
        sleep(250);
        drive.followTrajectory(stackBack);
        manipulator.moveSlideEncoder(MID_TICKS, 1);
        drive.followTrajectory(secMidPole);
        drive.followTrajectory(secMidPoleForward);
        manipulator.outtake();
        sleep(1250);
        manipulator.stopIntake();
        drive.followTrajectory(secMidPoleBack);


    }

}

