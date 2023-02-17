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

        drive.setPoseEstimate(new Pose2d());

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(70, -36, 0))
                .forward(70,
                        SampleMecanumDrive.getVelocityConstraint(13, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .back(12)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj);
        //drive.followTrajectory(traj2);
        drive.turn(Math.toRadians(90));
    }

}

