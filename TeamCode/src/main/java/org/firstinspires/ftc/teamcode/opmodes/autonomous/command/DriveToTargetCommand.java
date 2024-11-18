package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.SonicPIDController;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class DriveToTargetCommand extends SounderBotCommandBase {

    private static final String LOG_TAG = DriveToTargetCommand.class.getSimpleName();
    double minPower = 0.15;

    double distanceTolerance = 20;

    AutoMecanumDriveTrain driveTrain;
    GoBildaPinpointDriver odo;
    Telemetry telemetry;
    double targetX, targetY, targetHeading;

    SonicPIDController xpid = new SonicPIDController(0.0015, 0, 0);
    SonicPIDController ypid = new SonicPIDController(-0.001, 0, 0);
    SonicPIDController hpid = new SonicPIDController(1, 0, 0);

    public DriveToTargetCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double targetX, double targetY, double targetHeading, double minPower) {
        this.driveTrain = driveTrain;
        this.odo = driveTrain.getOdo();
        this.telemetry = telemetry;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetHeading = Math.toRadians(targetHeading);
        this.minPower = minPower;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        odo.update();

        telemetry.addData("tx: ", targetX);
        telemetry.addData("ty: ", targetY);
        telemetry.addData("th: ", targetHeading);

        telemetry.addData("x: ", odo.getPosX());
        telemetry.addData("y: ", odo.getPosY());
        telemetry.addData("heading: ", Math.toDegrees(odo.getHeading()));
        telemetry.update();

        Log.i(LOG_TAG, String.format("tx=%f, ty=%f, x=%f, y=%f, heading=%f", targetX, targetY, odo.getPosX(), odo.getPosY(), Math.toDegrees(odo.getHeading())));

        if(isTargetReached()) {
            // Give a 100ms to identify overshoot
            Uninterruptibles.sleepUninterruptibly(200, TimeUnit.MILLISECONDS);
            odo.update();

            if(isTargetReached()) {

                telemetry.addLine("Done");
                telemetry.update();

                finished.set(true);
                return;
            }
        }

        odo.update();

        // Battery reading of 13.49 required a Kp of 0.015
        double x = xpid.calculatePIDAlgorithm(targetX - odo.getPosX());
        double y = ypid.calculatePIDAlgorithm(targetY - odo.getPosY());
        double h = hpid.calculatePIDAlgorithm(targetHeading - odo.getHeading());

        double botHeading = odo.getHeading();

        double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
        double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);



        double frontLeftPower = (rotX + rotY + h);
        double backLeftPower = (rotX - rotY + h);
        double frontRightPower = (rotX - rotY - h);
        double backRightPower = (rotX + rotY - h);

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        if(Math.abs(frontLeftPower) < minPower) {
            frontLeftPower = minPower * Math.signum(frontLeftPower);
        }

        if(Math.abs(frontRightPower) < minPower) {
            frontRightPower = minPower * Math.signum(frontRightPower);
        }

        if(Math.abs(backLeftPower) < minPower) {
            backLeftPower = minPower * Math.signum(backLeftPower);
        }

        if(Math.abs(backRightPower) < minPower) {
            backRightPower = minPower * Math.signum(backRightPower);
        }

        Log.i(LOG_TAG, String.format("Wheels power: fL: %f, fR: %f, bL: %f, bR: %f", frontLeftPower, frontRightPower, backLeftPower, backRightPower));
//        telemetry.addData("frontLeft power", frontLeftPower);
//        telemetry.addData("frontRight power", frontRightPower);
//        telemetry.addData("backLeft power", backLeftPower);
//        telemetry.addData("backRight power", backRightPower);
//        telemetry.update();

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
