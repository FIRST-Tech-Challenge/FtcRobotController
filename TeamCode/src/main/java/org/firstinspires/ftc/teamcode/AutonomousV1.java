package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.HangBot;
import org.firstinspires.ftc.teamcode.bots.OdometryBot;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "Sample Auto", group = "Auto")
public class AutonomousV1 extends LinearOpMode {
    protected HangBot robot = new HangBot(this);
    @Override
    public void runOpMode() throws InterruptedException {

        robot.isAuto = true;
        robot.init(hardwareMap);

        robot.pivotTarget = 300;

        while (!opModeIsActive()) {
            telemetry.addData("rotate position", robot.rotate.getPosition());
            telemetry.addData("slide position", robot.slideMotor.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        robot.readySpecimenPos(true, true); //raise arm

        // drive to the submersible and set up for scoring
        robot.driveToCoordinate(-14000,-5000,0,800,1,true);
        robot.waitForCoordinateDrive();

        // drive towards the submersible
        robot.driveToCoordinate(-14000,-12000,0,300,0.3,true);
        robot.waitForCoordinateDrive();

        //score preloaded specimen
        robot.sleep(500);
        robot.slideMotor.setPower(0.2);
        robot.slideTarget -= 500;
        robot.sleep(1000);
        robot.openPinch();
        robot.slideMotor.setPower(0.6);

        //back out of the submersible
        robot.driveToCoordinate(-14000,-5000,0,800,1,true);
        robot.waitForCoordinateDrive();
        robot.slideTarget = 100;
        robot.pivotTarget = 200;
        robot.driveToCoordinate(3500,-5000,0,800,1,true);
        robot.waitForCoordinateDrive();
        //line up for first sample
        robot.driveToCoordinate(3500,-8000,0,300,0.3,true);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(3500,-11300,0,300,0.3,true);
        robot.waitForCoordinateDrive();

        robot.pivotMotor.setPower(0.2);
        robot.pivotTarget = 80;
        robot.sleep(200);
        robot.closePinch();
        robot.pivotMotor.setPower(0.6);
        robot.scoreBucket(true);

        robot.driveToCoordinate(0,-8000,0,800,0.8,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(0,0,180,800,0.8,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(11000,0,180,800,0.8,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(11000,3000,200,300,0.2,true);
        robot.waitForCoordinateDrive();

        robot.sleep(200);
        robot.openPinch();
        robot.sleep(200);

        robot.driveToCoordinate(11000,0,180,800,0.8,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(0,0,180,800,0.8,true);
        robot.waitForCoordinateDrive();

        robot.scoreBucket(true);
        robot.slideTarget = 100;
        robot.sleep(1000);
        robot.pivotTarget = 200;

        robot.driveToCoordinate(-5000,0,180,800,0.8,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(-5000,-8000,0,800,0.8,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(8250,-11800,0,300,0.3,true);
        robot.waitForCoordinateDrive();

        robot.pivotMotor.setPower(0.2);
        robot.pivotTarget = 80;
        robot.sleep(200);
        robot.closePinch();
        robot.pivotMotor.setPower(0.6);
        robot.scoreBucket(true);
        robot.pivotTarget = 1200;
        robot.slideTarget = 2400;

        robot.driveToCoordinate(0,-8000,0,800,0.8,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(0,0,180,800,0.8,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(11000,0,180,800,0.8,true);
        robot.waitForCoordinateDrive();

        robot.driveToCoordinate(11000,3000,200,300,0.2,true);
        robot.waitForCoordinateDrive();

        robot.sleep(200);
        robot.openPinch();
        robot.sleep(200);


//        robot.scoreBucket(true);
    }
}