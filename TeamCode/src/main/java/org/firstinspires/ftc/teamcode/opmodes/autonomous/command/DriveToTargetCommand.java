package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import java.util.concurrent.TimeUnit;

public class DriveToTargetCommand extends SounderBotCommandBase {

    private static final String LOG_TAG = DriveToTargetCommand.class.getSimpleName();
    double minPower = 0.15;

    double maxPower = 1.0;

    double distanceTolerance = 20;

    AutoMecanumDriveTrain driveTrain;

    GoBildaPinpointDriver odo;

    Telemetry telemetry;
    double targetX, targetY, targetHeading;

    SonicPIDFController xPid = new SonicPIDFController(0.0018, 0, 0, 0.005);

    SonicPIDFController yPid = new SonicPIDFController(-0.0033, 0, 00, 0.02);

    SonicPIDFController hPid = new SonicPIDFController(1, 0, 0, 0);

    public DriveToTargetCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double targetX, double targetY, double targetHeading, double minPower) {
        this(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, 1.0, 20);
    }

    public DriveToTargetCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double targetX, double targetY, double targetHeading, double minPower, double maxPower, double distanceTolerance) {
        this(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, maxPower, distanceTolerance,  1500);
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

        addRequirements(driveTrain);
    }


    @Override
    public void doExecute() {
        odo.update();

        boolean addTelemetry = true;

        if(addTelemetry) {
            telemetry.addData("tx: ", targetX);
            telemetry.addData("ty: ", targetY);
            telemetry.addData("th: ", targetHeading);

            telemetry.addData("x: ", odo.getPosX());
            telemetry.addData("y: ", odo.getPosY());
            telemetry.addData("heading: ", Math.toDegrees(odo.getHeading()));
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
            // Normalize to 0..1 motor power range
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
        driveTrain.stop();
    }
}
