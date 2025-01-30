package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.HangBot;

@Autonomous(name = "1 Sample Auto", group = "Auto")
public class OneSampleAuto extends LinearOpMode {
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
        robot.sleep(4000);
        telemetry.addData("sleeping: ","4 seconds");

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
        robot.driveToCoordinate(3000,-8000,0,150,0.3,false);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(3000,-11000,0,150,0.3,true);
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

        robot.driveToCoordinate(11000,3000,200,300,0.3,true);
        robot.waitForCoordinateDrive();

        //score sample
        robot.sleep(1000);
        robot.openPinch();
        robot.sleep(200);

        robot.driveToCoordinate(11000,0,180,800,1,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(0,0,180,800,1,false);
        robot.waitForCoordinateDrive();

        //lower arm
        robot.slideTarget = 105;
        robot.sleep(1000);
        robot.pivotTarget = 280;

        robot.driveToCoordinate(0,-13500,180,800,1,true);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(8600,-13500,180,800,1,true);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(8600,-500,180,800,1,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(7000,-14500,180,600, 1, true);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(10800,-14500,180,900,1,true);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(10800,-800,180,800,1,true);
        robot.waitForCoordinateDrive();
    }
}