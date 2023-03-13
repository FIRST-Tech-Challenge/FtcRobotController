package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import static java.lang.Math.toRadians;

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
        Pose2d startPose = new Pose2d(-29.6, 62.25, toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);
        robot.cv.observeSleeve();
        waitForStart();
        double[] loopStart={0,0};
        robot.cv.observeStick();

        while(opModeIsActive()) {

//                if(robot.field.lookingAtPole()){
                    Pose2d target = robot.field.polePos();
//                TrajectorySequence trajectory = roadrun.getCurrentTraj();
//                roadrun.changeTrajectorySequence(roadrun.trajectorySequenceBuilder(trajectory.start())
//                                .setReversed(true)
//                        .splineTo(target.vec(), target.getHeading()).build());
////                field.setDoneLookin(true);
                    telemetry.addData("polePos", target);
                    telemetry.addData("curPos",robot.roadrun.getPoseEstimate());
                    telemetry.addData("coords0",robot.cv.rotatedPolarCoord()[0]);
                    telemetry.addData("coords1",robot.cv.rotatedPolarCoord()[1]);

                    telemetry.update();
                    logger.log("/RobotLogs/GeneralRobot", "mr.obama"+target+"im pole"+robot.roadrun.getPoseEstimate());
                    logger.log("/RobotLogs/GeneralRobot", "coords"+robot.cv.rotatedPolarCoord()[0]+","+robot.cv.rotatedPolarCoord()[1]);

//                }
                robot.roadrun.update();

            robot.setFirstLoop(false);
        }
        robot.stop();
    }
}


//follow this tutorial "FTC EasyOpenCV Tutorial + SkyStone Example"

//https://www.youtube.com/watch?v=JO7dqzJi8lw&t=400s


