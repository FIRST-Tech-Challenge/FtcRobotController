package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import java.util.concurrent.TimeUnit;

public class DriveToTargetCommandAlterate extends SounderBotCommandBase {

    private static final String LOG_TAG = DriveToTargetCommandAlterate.class.getSimpleName();

    double minXPower = 0.03;

    double minYPower = 0.1;

    double minHPower = 0.1;

    double distanceTolerance = 10;

    AutoMecanumDriveTrain driveTrain;
    GoBildaPinpointDriver odo;
    Telemetry telemetry;
    double targetX, targetY, targetHeading;

    private SonicPIDFController xPid = new SonicPIDFController(0.0015, 0, 0.0002);
    private SonicPIDFController yPid = new SonicPIDFController(0.0025, 0, 0.0002);
    private SonicPIDFController hPid = new SonicPIDFController(0.5, 0, 0.001);

    public DriveToTargetCommandAlterate(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double targetX, double targetY, double targetHeadingInDegrees) {
        this.driveTrain = driveTrain;
        this.odo = driveTrain.getOdo();
        this.telemetry = telemetry;
        this.targetX = targetX;
        this.targetY = -1 * targetY;
        this.targetHeading = -1 * Math.toRadians(targetHeadingInDegrees);
        addRequirements(driveTrain);
    }

    @Override
    public void doExecute() {
        odo.update();

        Log.i(LOG_TAG, String.format("tx=%f, ty=%f, x=%f, y=%f, heading=%f", targetX, targetY, odo.getPosX(), odo.getPosY(), Math.toDegrees(odo.getHeading())));

        if(isTargetReached()) {
            driveTrain.stop();

            sleep(150);
            odo.update();

            if(isTargetReached()) {
                finished = true;
                return;
            }
        }

        odo.update();

        double xError = targetX - odo.getPosX();
        double yError = targetY - odo.getPosY();
        double hError = targetHeading - odo.getHeading();

        double xPower = xPid.calculatePIDAlgorithm(xError);
        double yPower = yPid.calculatePIDAlgorithm(yError);
        double hPower = hPid.calculatePIDAlgorithm(hError);

        if(Math.abs(xPower) < minXPower) {
            xPower = minXPower * Math.signum(xPower);
        }

        if(Math.abs(yPower) < minYPower) {
            yPower = minYPower * Math.signum(yPower);
        }

        if(Math.abs(hPower) < minHPower) {
            hPower = minHPower * Math.signum(hPower);
        }

//        telemetry.addData("x",odo.getPosX());
//        telemetry.addData("y", odo.getPosY());
//        telemetry.addData("h", Math.toDegrees(odo.getHeading()));
//
//        telemetry.addData("xp", xPower);
//        telemetry.addData("yp", yPower);
//
//        telemetry.update();

        driveTrain.driveRobotCentric(yPower, hPower, xPower);
    }

    @Override
    protected boolean isTargetReached() {
        return (Math.abs(targetX - odo.getPosX()) < distanceTolerance) &&
                (Math.abs(targetY - odo.getPosY())) < distanceTolerance &&
                (Math.abs(targetHeading - odo.getHeading()) < Math.toRadians(2));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.stop();
    }
}
