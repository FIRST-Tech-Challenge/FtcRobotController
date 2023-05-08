package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
@Autonomous(name = "ConeObserverTest")

public class ConeObserverTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, true);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        sleep(500);
        Pose2d startPose = new Pose2d(-29.6, 62.25, toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);
        robot.cv.observeCone();
        waitForStart();
        double[] loopStart={0,0};
        while(opModeIsActive()) {

            if(robot.field.lookingAtCone()){
                Pose2d target = robot.field.conePos();
//                TrajectorySequence trajectory = roadrun.getCurrentTraj();
//                roadrun.changeTrajectorySequence(roadrun.trajectorySequenceBuilder(trajectory.start())
//                                .setReversed(true)
//                        .splineTo(target.vec(), target.getHeading()).build());
////                field.setDoneLookin(true);
                telemetry.addData("polePos", target);
                telemetry.addData("curPos",robot.roadrun.getPoseEstimate());
                telemetry.addData("coords0",robot.cv.knockedConarCoord()[0]);
                telemetry.addData("coords1",robot.cv.knockedConarCoord()[1]);

                telemetry.update();
                logger.log("/RobotLogs/GeneralRobot", "mr.obama"+target+"im pole"+robot.roadrun.getPoseEstimate());
                logger.log("/RobotLogs/GeneralRobot", "coords"+robot.cv.rotatedConarCoord()[0]+","+robot.cv.rotatedConarCoord()[1]);

            }
            robot.roadrun.update();

            robot.setFirstLoop(false);
        }
        robot.stop();
    }
}


//follow this tutorial "FTC EasyOpenCV Tutorial + SkyStone Example"

//https://www.youtube.com/watch?v=JO7dqzJi8lw&t=400s


