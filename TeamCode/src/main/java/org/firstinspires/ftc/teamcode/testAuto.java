package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "testAuto",group = "UltGoal")
public class testAuto extends LinearOpMode {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    OdometryGlobalCoordinatePosition globalCoordinatePosition;
    double powerA, powerB;
    double motorX, motorY = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        telemetry.addData("Status", "Initializing Hardware");
        telemetry.update();

        waitForStart();

        Thread positionThread = new Thread(globalCoordinatePosition);
        positionThread.start();

        driveTo(40, 40, 50);

    }

    public void driveTo(double x, double y, double theta) {
        while (opModeIsActive() && withinpos(globalCoordinatePosition.globalPositionX, x, globalCoordinatePosition.globalPositionY, y, 50) && withinpos(globalCoordinatePosition.robotOrientationDegrees, theta, 5)) {
            findPower(x, y);
            drivetrain.setMotorPowers(powerB, powerA, powerA, powerB); //doesnt integrate turning
        }
    }

    public void findPower(double x, double y) {
        double slope = y/x;
        motorX = Math.cos(Math.toRadians(globalCoordinatePosition.changeinrobotorientationDegrees-45));//PROBLEM IN CODE
        motorY = Math.sin(Math.toRadians(globalCoordinatePosition.changeinrobotorientationDegrees-45));//PROBLEM IN CODE
        powerA = ((motorX/1) + (motorY/slope))/2; //(1/1) condensed to 1
        powerB = ((-motorX/1) + (motorY/slope))/2; //(-1/1) condensed to -1
        if (Math.abs(powerA) > Math.abs(powerB)) {
            powerB = 1/(powerA/powerB);
            powerA = 1;
        } else {
            powerA = 1/(powerB/powerA);
            powerB = 1;
        }
        if (x < 0){
            powerA = -powerA;
        } else {
            powerB = -powerB;
        }
    }


    public boolean withinpos(double currpos, double wantpos, double within){
        return currpos > wantpos-within && currpos < wantpos + within;
    }
    public boolean withinpos(double currpos, double wantpos, double currpos2, double wantpos2, double within){
        return withinpos(currpos, wantpos, within) && withinpos(currpos2, wantpos2, within);
    }
}
