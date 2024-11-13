package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;

import java.util.concurrent.TimeUnit;

public class TurnAngleCommand extends SounderBotCommandBase {
    double minError = Math.toRadians(1.5);
    double minPower = 0.2;

    AutoMecanumDriveTrain driveTrain;
    GoBildaPinpointDriver odo;
    Telemetry telemetry;
    double targetAngleInRadians;
    double error = Double.MAX_VALUE;
    double driveMotorsPower;

    public TurnAngleCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double targetAngleInDegrees) {
        this.driveTrain = driveTrain;
        this.odo = driveTrain.getOdo();
        this.telemetry = telemetry;
        this.targetAngleInRadians = Math.toRadians(targetAngleInDegrees);
    }

    @Override
    public void execute() {
        odo.update();
        error = targetAngleInRadians - odo.getHeading();
        if (isTargetReached()) {
            Uninterruptibles.sleepUninterruptibly(100, TimeUnit.MILLISECONDS);
            if (isTargetReached()) {
                finished.set(true);
                return;
            }
        }
        driveMotorsPower = -.4 * error;

        if(Math.abs(driveMotorsPower) < minPower) {
            driveMotorsPower = minPower * Math.signum(driveMotorsPower);
        }

        driveTrain.setWheelsPower(-driveMotorsPower, driveMotorsPower, -driveMotorsPower, driveMotorsPower);

    }

    @Override
    protected boolean isTargetReached() {
        return Math.abs(error) < minError;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.stop();
    }
}
