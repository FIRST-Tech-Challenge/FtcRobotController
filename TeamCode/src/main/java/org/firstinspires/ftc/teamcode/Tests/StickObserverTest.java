package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
@Autonomous(name = "StickObserverTest")

public class StickObserverTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, true);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        sleep(500);
        robot.roadrun.setPoseEstimate(new Pose2d(40.2, 62.25, Math.toRadians(270)));
//        robot.roadrun.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));
        robot.cv.observeStick();
        waitForStart();
        double[] loopStart={0,0};
        while(opModeIsActive()) {
                telemetry.addData("cvtheta",robot.cv.rotatedPolarCoord()[0]);
                telemetry.addData("cvdistance",robot.cv.rotatedPolarCoord()[1]);
//                telemetry.addData("aimTo",robot.roadrun.getPoseEstimate().getHeading() + robot.cv.rotatedPolarCoord()[0]);
                telemetry.update();
                loopStart[0] = getRuntime();
                robot.roadrun.update();
//                robot.autoAim();
//                logger.log("/RobotLogs/GeneralRobot", "distance"+robot.cv.rotatedPolarCoord()[1]);
            logger.log("/RobotLogs/GeneralRobot", "angle"+robot.cv.rotatedPolarCoord()[0]);
            robot.setFirstLoop(false);
        }
        robot.stop();
    }
}


//follow this tutorial "FTC EasyOpenCV Tutorial + SkyStone Example"

//https://www.youtube.com/watch?v=JO7dqzJi8lw&t=400s


