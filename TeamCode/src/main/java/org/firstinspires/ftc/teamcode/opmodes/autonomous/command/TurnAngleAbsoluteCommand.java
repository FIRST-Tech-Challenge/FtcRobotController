package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.SonicPIDFController;

import java.util.concurrent.TimeUnit;

public class TurnAngleAbsoluteCommand extends SounderBotCommandBase {
    private static final String LOG_TAG = TurnAngleAbsoluteCommand.class.getSimpleName();
    double minError = Math.toRadians(2);
    double minPower = 0.15;

    AutoMecanumDriveTrain driveTrain;
    GoBildaPinpointDriver odo;
    Telemetry telemetry;
    double targetTurnInRadians;
    double error = Double.MAX_VALUE;

    SonicPIDFController pidController = new SonicPIDFController(0.5, 0, 0.02);

    public TurnAngleAbsoluteCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double turnInDegrees) {
        this.driveTrain = driveTrain;
        this.telemetry = telemetry;

        odo = driveTrain.getOdo();
        odo.update();
        this.targetTurnInRadians = Math.toRadians(turnInDegrees * -1);
    }

    @Override
    public void execute() {
        odo.update();
        error = targetTurnInRadians - odo.getHeading();

        if (isTargetReached()) {
            driveTrain.stop();
            Uninterruptibles.sleepUninterruptibly(100, TimeUnit.MILLISECONDS);

            odo.update();
            error = targetTurnInRadians - odo.getHeading();
            if (isTargetReached()) {
                finished.set(true);
                return;
            }
        }

        double power = pidController.calculatePIDAlgorithm(error);

        if(Math.abs(power) < minPower) {
            power = minPower * Math.signum(power);
        }

        driveTrain.driveRobotCentric(0, power, 0);
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
