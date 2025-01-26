package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import java.util.concurrent.TimeUnit;

@Config
public class DriveToTargetCommand extends SounderBotCommandBase {

    private static final String LOG_TAG = DriveToTargetCommand.class.getSimpleName();
    double minPower = 0.15;

    double maxPower = 1.0;

    double distanceTolerance = 20;

    AutoMecanumDriveTrain driveTrain;

    GoBildaPinpointDriver odo;

    Telemetry telemetry;
    double targetX, targetY, targetHeading;

    public static double xPid_p = 0.0010;
    public static double xPid_i = 0;
    public static double xPid_d = 0;
    public static double xPid_f = 0.005;

    public static double yPid_p = -0.0033;
    public static double yPid_i = 0;
    public static double yPid_d = 0;
    public static double yPid_f = 0.02;

    public static double hPid_p = 1;
    public static double hPid_i = 0;
    public static double hPid_d = 0;
    public static double hPid_f = 0;

    SonicPIDFController xPid;

    SonicPIDFController yPid;

    SonicPIDFController hPid;

    public DriveToTargetCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double targetX, double targetY, double targetHeading, double minPower) {
        this(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, 1.0, 20);
    }

    public DriveToTargetCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double targetX, double targetY, double targetHeading, double minPower, double maxPower, double distanceTolerance) {
        this(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, maxPower, distanceTolerance,  1800);
    }

    public DriveToTargetCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double targetX, double targetY, double targetHeading, double minPower, double maxPower, double distanceTolerance, long timeOut) {
        super(timeOut);

        this.driveTrain = driveTrain;
        this.odo = driveTrain.getOdo();
        this.telemetry = telemetry;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading = Math.toRadians(targetHeading);
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.distanceTolerance = distanceTolerance;

        xPid = new SonicPIDFController(xPid_p, xPid_i, xPid_d, xPid_f);
        yPid = new SonicPIDFController(yPid_p, yPid_i, yPid_d, yPid_f);
        hPid = new SonicPIDFController(hPid_p, hPid_i, hPid_d, hPid_f);

        addRequirements(driveTrain);
    }


    @Override
    public void doExecute() {
        odo.update();

        boolean addTelemetry = true;

        if(addTelemetry) {
            telemetry.addData("DriveToTarget tx: ", targetX);
            telemetry.addData("DriveToTarget ty: ", targetY);
            telemetry.addData("DriveToTarget th: ", targetHeading);

            telemetry.addData("DriveToTarget x: ", odo.getPosX());
            telemetry.addData("DriveToTarget y: ", odo.getPosY());
            telemetry.addData("DriveToTarget heading: ", Math.toDegrees(odo.getHeading()));
        };

        Log.i(LOG_TAG, String.format("tx=%f, ty=%f, x=%f, y=%f, heading=%f", targetX, targetY, odo.getPosX(), odo.getPosY(), Math.toDegrees(odo.getHeading())));

        if(isTargetReached()) {
            // Give a 200ms to identify overshoot
            sleep(200);
            odo.update();

            if(isTargetReached()) {

                if(addTelemetry) {
                    telemetry.addLine("Done");
                }

                finished = true;
                return;
            }
        }

        odo.update();

        // Battery reading of 13.49 required a Kp of 0.015
        double x = xPid.calculatePIDAlgorithm(targetX - odo.getPosX());
        double y = yPid.calculatePIDAlgorithm(targetY - odo.getPosY());
        double h = hPid.calculatePIDAlgorithm(targetHeading - odo.getHeading());

        onFlagEnabled(addTelemetry, () -> {
            telemetry.addData("drive to target xPid output", x);
            telemetry.addData("drive to target yPid output", y);
            telemetry.addData("drive to target hPid output", h);
        });


        double botHeading = odo.getHeading();

        double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
        double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

        double frontLeftPower = (rotX + rotY + h);
        double backLeftPower = (rotX - rotY + h);
        double frontRightPower = (rotX - rotY - h);
        double backRightPower = (rotX + rotY - h);

        double max = Math.max(
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                    );

        if (max > maxPower) {
            // Normalize to 0 to 1 motor power range
            frontLeftPower = maxPower * frontLeftPower / max ;
            frontRightPower = maxPower * frontRightPower / max;
            backLeftPower = maxPower * backLeftPower / max;
            backRightPower = maxPower * backRightPower / max;
        }
        else if (max < minPower) {
            // Proportionally increase power in all motors until max wheel power is enough
            double proportion = minPower / max;
            frontLeftPower *= proportion;
            frontRightPower *= proportion;
            backLeftPower *= proportion;
            backRightPower *= proportion;
        }

        Log.i(LOG_TAG, String.format("Wheels power: fL: %f, fR: %f, bL: %f, bR: %f", frontLeftPower, frontRightPower, backLeftPower, backRightPower));

        if(addTelemetry) {
            telemetry.addData("frontLeft power", frontLeftPower);
            telemetry.addData("frontRight power", frontRightPower);
            telemetry.addData("backLeft power", backLeftPower);
            telemetry.addData("backRight power", backRightPower);
            telemetry.update();
        }

        driveTrain.setWheelsPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    @Override
    protected boolean isTargetReached() {
        return (Math.abs(targetX - odo.getPosX()) < distanceTolerance)
                && (Math.abs(targetY - odo.getPosY())) < distanceTolerance
                && Math.abs(targetHeading - odo.getHeading()) < Math.toRadians(3);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        finished = true;
        driveTrain.stop();
    }
}
