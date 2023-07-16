package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;

@Autonomous(name = "SleeveObserverTest")

public class SleeveObserverTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, true);
        sleep(500);
        robot.roadrun.setPoseEstimate(new Pose2d(40.2, 62.25, Math.toRadians(270)));
//        robot.roadrun.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));
        robot.cv.observeSleeve();
        waitForStart();
        double[] loopStart={0,0};
        while(opModeIsActive()) {
            telemetry.addData("pos",robot.cv.getPosition());
            telemetry.update();
        }
        robot.stop();
    }
}


//follow this tutorial "FTC EasyOpenCV Tutorial + SkyStone Example"

//https://www.youtube.com/watch?v=JO7dqzJi8lw&t=400s


