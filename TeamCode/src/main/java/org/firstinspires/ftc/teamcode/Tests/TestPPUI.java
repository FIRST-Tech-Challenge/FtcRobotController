package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.secret.Waypoint;

import java.util.ArrayList;

@Autonomous
public class TestPPUI extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        robot.roadrun.setPoseEstimate(new Pose2d(15,-60, toRadians(90)));
        robot.update();
        waitForStart();
        ArrayList<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(currentPose, 12));
        path.add(new Waypoint(new Pose2d(15,-40,0),12));
        path.add(new Waypoint(new Pose2d(35, -30,toRadians(90)),12));
        while(!isStopRequested()&&opModeIsActive()){
            robot.loopPPUI(path, 1);
            robot.update();
        }
    }
}
