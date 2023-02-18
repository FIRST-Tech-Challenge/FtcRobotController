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


@Autonomous(name = "RRTest", group = "Taus2022-23")
public class RoadRunnerTest extends LinearOpMode {

    //SimpleHardware robot = new SimpleHardware(true);
    ElapsedTime timer = new ElapsedTime();
    Vector2d myVector = new Vector2d(10, -5);
    Pose2d myPose = new Pose2d(10, -5, Math.toRadians(90));
    SampleMecanumDrive drive;


    static final double FEET_PER_METER = 3.28084;


    public void runOpMode() {
         drive = new SampleMecanumDrive(hardwareMap);
        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("Waiting for start");
        telemetry.update();
        timer.reset();

        drive.setPoseEstimate(new Pose2d(-36, -63, Math.toRadians(90)));

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                .forward(60)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .lineToLinearHeading(new Pose2d(-39, -24, 0))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-38, -12, Math.toRadians(180)), Math.toRadians(270))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-64, -12, Math.toRadians(180)))
                .build();
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj);
        drive.followTrajectory(traj2);
        //drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
    }

}

