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
    double minPower = 0.3;
    double distanceTolerance = 5;

    AutoMecanumDriveTrain driveTrain;
    GoBildaPinpointDriver odo;
    Telemetry telemetry;
    double targetX, targetY;

    public DriveToTargetCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double targetX, double targetY) {
        this.driveTrain = driveTrain;
        this.odo = driveTrain.getOdo();
        this.telemetry = telemetry;
        this.targetX = targetX;
        this.targetY = targetY;
        addRequirements(driveTrain);
        driveTrain.resetOdo();
    }

    @Override
    public void execute() {
        telemetry.addLine("Driving");
        telemetry.update();

        odo.update();
        telemetry.addData("tx: ", targetX);
        telemetry.addData("ty: ", targetY);

        telemetry.addData("x: ", odo.getPosX());
        telemetry.addData("y: ", odo.getPosY());
        telemetry.addData("heading: ", Math.toDegrees(odo.getHeading()));
        telemetry.update();

        Log.i(LOG_TAG, String.format("tx=%f, ty=%f, x=%f, y=%f, heading=%f", targetX, targetY, odo.getPosX(), odo.getPosY(), Math.toDegrees(odo.getHeading())));

        if(isTargetReached()) {
            // Give a 100ms to identify overshoot
            Uninterruptibles.sleepUninterruptibly(100, TimeUnit.MILLISECONDS);

            if(isTargetReached()) {
                finished.set(true);
                return;
            }
        }

        // Battery reading of 13.49 required a Kp of 0.015
        double kp = 0.003;
        double x = kp * (targetX - odo.getPosX());
        double y = - kp * (targetY - odo.getPosY());

        double botHeading = odo.getHeading();

        double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
        double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        double frontLeftPower = (rotX + rotY) / denominator;
        double backLeftPower = (rotX - rotY) / denominator;
        double frontRightPower = (rotX - rotY) / denominator;
        double backRightPower = (rotX + rotY) / denominator;

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
        telemetry.addData("frontLeft power", frontLeftPower);
        telemetry.addData("frontRight power", frontRightPower);
        telemetry.addData("backLeft power", backLeftPower);
        telemetry.addData("backRight power", backRightPower);
        telemetry.update();

        driveTrain.setWheelsPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    @Override
    protected boolean isTargetReached() {
        return (Math.abs(targetX - odo.getPosX()) < distanceTolerance) && (Math.abs(targetY - odo.getPosY())) < distanceTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.stop();
        driveTrain.resetOdo();
    }
}
