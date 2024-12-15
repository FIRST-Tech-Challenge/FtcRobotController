package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.HangBot;

@Autonomous(name = "2 Sample Auto", group = "Auto")
public class TwoSampleAuto extends LinearOpMode {
    protected HangBot robot = new HangBot(this);
    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = true;
        robot.init(hardwareMap);

        robot.pivotTarget = 280;

        while (!opModeIsActive()) {
            telemetry.addData("rotate position", robot.rotate.getPosition());
            telemetry.addData("slide position", robot.slideMotor.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        robot.readySpecimenPos(true, true); //raise arm

        // drive to the submersible and set up for scoring
        robot.driveToCoordinate(-15000,-5000,0,800,1,false);
        robot.waitForCoordinateDrive();

        // drive towards the submersible
        robot.driveToCoordinate(-15000,-14000,0,300,0.3,true);
        robot.waitForCoordinateDrive();

        //score preloaded specimen
        robot.sleep(500);
        robot.slideMotor.setPower(0.2);
        robot.slideTarget -= 400;
        robot.sleep(400);
        robot.openPinch();
        robot.slideMotor.setPower(0.6);

        //back out of the submersible
        robot.driveToCoordinate(-14000,-5000,0,800,1,false);
        robot.waitForCoordinateDrive();
        robot.slideTarget = 100;
        robot.pivotTarget = 200;
        robot.driveToCoordinate(3500,-5000,0,800,1,false);
        robot.waitForCoordinateDrive();
        //line up for first sample
        robot.driveToCoordinate(3300,-8000,0,300,0.3,false);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(3300,-11200,0,300,0.3,true);
        robot.waitForCoordinateDrive();

// Configure robot to score on bucket after picking up sample
        robot.pivotMotor.setPower(0.3);
        robot.pivotTarget = -30;
        robot.sleep(400);
        robot.closePinch();
        robot.sleep(400);
        robot.pivotMotor.setPower(0.6);
        robot.scoreBucket(true);

        //turn to score sample
        robot.driveToCoordinate(0,-8000,0,800,1,false);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(0,0,180,800,1,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(11000,1000,180,800,1,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(11000,3700,200,300,0.3,true);
        robot.waitForCoordinateDrive();

        //score sample
        robot.sleep(200);
        robot.openPinch();
        robot.sleep(200);

        //turn back
        robot.driveToCoordinate(11000,0,180,800,1,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(0,0,180,800,1,false);
        robot.waitForCoordinateDrive();

        //lower arm
        robot.scoreBucket(true);
        robot.slideTarget = 105;
        robot.sleep(1000);
        robot.pivotTarget = 280;

        robot.driveToCoordinate(-5000,0,180,800,1,false);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(-5000,-8000,0,800,1,false);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(7850,-12500,0,300,0.3,true);
        robot.waitForCoordinateDrive();

        //pick up second sample
        robot.pivotMotor.setPower(0.2);
        robot.pivotTarget =-30;
        robot.sleep(300);
        robot.closePinch();
        robot.sleep(300);
        robot.pivotMotor.setPower(0.6);
        robot.scoreBucket(true);
        robot.pivotTarget = 1200;
        robot.slideTarget = 2400;

        robot.driveToCoordinate(0,-8000,0,800,0.8,false);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(0,0,180,800,0.8,false);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(11500,0,180,800,0.8,false);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(11500,3300,210,300,0.2,true);
        robot.waitForCoordinateDrive();

        robot.sleep(200);
        robot.openPinch();
        robot.sleep(200);

        robot.driveToCoordinate(11000,0,180,800,0.8,false);
        robot.waitForCoordinateDrive();
        robot.scoreBucket(true);
        robot.slideTarget = 0;
        robot.sleep(1000);
        robot.pivotTarget = 0;
        robot.sleep(2000);

//        robot.scoreBucket(true);
    }
}