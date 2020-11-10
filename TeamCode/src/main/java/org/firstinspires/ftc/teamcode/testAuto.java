package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "testAuto",group = "UltGoal")
public class testAuto extends LinearOpMode {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    OdometryGlobalCoordinatePosition globalCoordinatePosition = new OdometryGlobalCoordinatePosition(100, 100, 90);
    AutoOmniMovement autoOmniMovement = new AutoOmniMovement(100, 100, 90);
    double powerA, powerB;
    double motorX, motorY = 1.0;
    double angleRad = Math.toRadians(225-45);

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
        while (opModeIsActive() && boundaryPos(x, y, 100) && boundaryTurn(theta, 5)) {
            autoOmniMovement.findPower(globalCoordinatePosition.Xpos(), globalCoordinatePosition.Ypos(), x, y);
            autoOmniMovement.addTurnPower(theta);
            drivetrain.setMotorPowers(autoOmniMovement.wheelPowers[3], autoOmniMovement.wheelPowers[1], autoOmniMovement.wheelPowers[2], autoOmniMovement.wheelPowers[0]); //doesnt integrate turning
        }
    }
    public boolean boundaryPos(double x, double y, double within){
        return Math.sqrt(Math.pow(globalCoordinatePosition.Xpos()-x, 2) + Math.pow(globalCoordinatePosition.Ypos()-y, 2)) < within;
    }

    public boolean boundaryTurn(double theta, double withinDeg){
        return globalCoordinatePosition.theta() > theta-withinDeg && globalCoordinatePosition.theta() < theta+withinDeg;
    }

    public boolean withinpos(double currpos, double wantpos, double within){
        return currpos > wantpos-within && currpos < wantpos + within;
    }
    public boolean withinpos(double currpos, double wantpos, double currpos2, double wantpos2, double within){
        return withinpos(currpos, wantpos, within) && withinpos(currpos2, wantpos2, within);
    }


}
