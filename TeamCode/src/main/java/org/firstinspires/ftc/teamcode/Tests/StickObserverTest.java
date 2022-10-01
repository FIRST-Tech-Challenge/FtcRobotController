package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
@Autonomous(name = "StickObserverTest")

public class StickObserverTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, true);
        sleep(500);
        robot.roadrun.setPoseEstimate(new Pose2d(41, 61, Math.toRadians(270)));
        robot.cv.observeStick();
        waitForStart();
        double[] loopStart={0,0};
        while(opModeIsActive()) {
            while(getRuntime()-loopStart[1]<5&&opModeIsActive()) {
                if(robot.field.lookingAtPole()){
                    double[] pole = robot.field.lookedAtPole();
                    telemetry.addData("poleTheta", pole[3]);
                    telemetry.addData("poleDistance", pole[2]);

                }
                telemetry.addData("centerOffset", robot.cv.centerOfPole());
                telemetry.addData("centerSize", robot.cv.poleSize());
                telemetry.addData("theta",robot.cv.rotatedPolarCoordDelta()[0]*180/PI);
                telemetry.addData("distance",robot.cv.rotatedPolarCoordDelta()[1]);
                telemetry.update();
                robot.roadrun.update();
                loopStart[0] = getRuntime();
                robot.roadrun.update();
            }
//            opencv.resetAverages();
            //77.26,77.5,77.5,77.56
            loopStart[1]=getRuntime();
        }
        robot.cv.stopCamera();
        stop();
    }
}


//follow this tutorial "FTC EasyOpenCV Tutorial + SkyStone Example"

//https://www.youtube.com/watch?v=JO7dqzJi8lw&t=400s


