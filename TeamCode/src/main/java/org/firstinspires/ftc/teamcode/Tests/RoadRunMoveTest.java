package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Autonomous(name = "RoadRunMoveTest")
public class RoadRunMoveTest extends LinearOpMode {
    public static double DISTANCE = 48; // in
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY,false,false,0);

        Pose2d startPose = new Pose2d(57, -54, 0);

        robot.roadrun.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence trajSeq = robot.roadrun.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(57,-30))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(9,-30))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(9,-54))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(57,-54))
                .build();
        robot.roadrun.followTrajectorySequence(trajSeq);
        robot.roadrun.followTrajectorySequence(trajSeq);
    }
}
