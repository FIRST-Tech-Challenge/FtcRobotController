package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "testAuto",group = "UltGoal")
public class testAuto extends LinearOpMode {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    OdometryGlobalCoordinatePosition globalCoordinatePosition = new OdometryGlobalCoordinatePosition(100, 100, 90);
    AutoOmniMovement autoOmniMovement = new AutoOmniMovement(100, 100, 90);

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

    public void driveTo(double deg, boolean specific){
        deg = (specific) ? deg : globalCoordinatePosition.theta()+deg;
        while (opModeIsActive() && boundaryPosOneD(globalCoordinatePosition.theta(), deg, 5)){
            drivetrain.spin(globalCoordinatePosition.theta() > deg, (boundaryPosOneD(globalCoordinatePosition.theta(), deg, 20) ? 0.5 : 1.0));
        }
    }

    public void driveTo(Drivetrain.moveDirection direction, double amount){
        double value;
        amount = (direction== Drivetrain.moveDirection.LEFT||direction== Drivetrain.moveDirection.BACKWARD) ? -amount : amount;
        do {
            value = (direction== Drivetrain.moveDirection.FORWARD||direction== Drivetrain.moveDirection.BACKWARD) ? globalCoordinatePosition.Xpos() : globalCoordinatePosition.Ypos();
            drivetrain.moveDirect(direction, (boundaryPosOneD(value, value+amount,500) ? 0.2 : autoOmniMovement.MAX_POWER));
        } while (opModeIsActive() && !boundaryPosOneD(value, value+amount,100));
    }

    public void driveTo(double x, double y, double theta) {
        while (opModeIsActive() && !boundaryPosTwoD(x, y, 100) && !boundaryTurn(theta, 5)) {
            autoOmniMovement.findPower(globalCoordinatePosition.Xpos(), globalCoordinatePosition.Ypos(), x, y);
            autoOmniMovement.addTurnPower(theta);
            drivetrain.setMotorPowers(autoOmniMovement.wheelPowers[3], autoOmniMovement.wheelPowers[1], autoOmniMovement.wheelPowers[2], autoOmniMovement.wheelPowers[0]); //doesnt integrate turning
        }
    }
    public boolean boundaryPosTwoD(double x, double y, double within){
        return Math.sqrt(Math.pow(globalCoordinatePosition.Xpos()-x, 2) + Math.pow(globalCoordinatePosition.Ypos()-y, 2)) < within;
    }

    public boolean boundaryPosOneD(double initial, double fin, double within){
        return Math.abs(fin-initial) < within;
    }

    public boolean boundaryTurn(double theta, double withinDeg){
        return globalCoordinatePosition.theta() > theta-withinDeg && globalCoordinatePosition.theta() < theta+withinDeg;
    }


}
