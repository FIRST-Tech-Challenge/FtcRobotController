package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFWaypoint;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Autonomous(name = "RFMotionControllerTest")
public class RFMotionControllerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this,true);
        RFMecanumDrive drive = new RFMecanumDrive();
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        drive.setPose(startPose);

        waitForStart();
        if (isStopRequested()) return;


//        while (opModeIsActive()) {
        for(int i=0; i<1;i++) {
            if (!drive.isFollowing()) {
                drive.setReversed(false);
                drive.setCurviness(0.5);
                drive.addWaypoint(new RFWaypoint(new Vector2d(50, 0)));
                drive.setReversed(true);
                drive.addWaypoint(new RFWaypoint(new Vector2d(0, 0)));
            }
            drive.update();
            robot.update();
        }
//            robot.update();
        }
//    }
}
