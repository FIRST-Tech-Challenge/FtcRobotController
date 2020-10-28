package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "testAuto",group = "UltGoal")
public class testAuto extends LinearOpMode {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    OdometryGlobalCoordinatePosition globalCoordinatePosition;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        telemetry.addData("Status", "Initializing Hardware");
        telemetry.update();

        waitForStart();

        Thread positionThread = new Thread(globalCoordinatePosition);
        positionThread.start();



    }

    public void driveTo(double x, double y, double theta) {
        while (opModeIsActive() && withinpos(globalCoordinatePosition.globalPositionX, x, globalCoordinatePosition.globalPositionY, y, 50) && withinpos(globalCoordinatePosition.robotOrientationDegrees, theta, 5)) {
            
        }
    }

    public boolean withinpos(double currpos, double wantpos, double within){
        return currpos > wantpos-within && currpos < wantpos + within;
    }
    public boolean withinpos(double currpos, double wantpos, double currpos2, double wantpos2, double within){
        return withinpos(currpos, wantpos, within) && withinpos(currpos2, wantpos2, within);
    }
}
