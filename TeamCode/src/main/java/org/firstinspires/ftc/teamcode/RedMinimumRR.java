package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Robot.*;


@Autonomous(name = "Auto Red Minimum RR", preselectTeleOp = "teleop")
@Disabled
public class RedMinimumRR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ////////////init motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        MainMecanumDrive drive = new MainMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(57, 20, Math.toRadians(0)); //init starting position
        drive.setPoseEstimate(startPose);
        //////NO Camera Streaming//////

////////Program start////////////////////////////////////////////////////////////////////////
        waitForStart();


        //move to line
        Trajectory standby = drive.trajectoryBuilder(startPose,true)
                .splineTo(new Vector2d(-50,5), Math.toRadians(180))
                .build();
        drive.followTrajectory(standby);

        sleep(24000);

        Trajectory line = drive.trajectoryBuilder(standby.end())
                .lineTo(new Vector2d(-10,2))
                .build();
        drive.followTrajectory(line);

        if (isStopRequested()) return;
        sleep(2000);

    }
}
