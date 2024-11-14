package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.AngleTracker;

import java.util.concurrent.TimeUnit;

public class TurnAngleRelativeCommand extends SounderBotCommandBase {
    private static final String LOG_TAG = TurnAngleRelativeCommand.class.getSimpleName();
    double min = 0.2;

    AutoMecanumDriveTrain driveTrain;
    GoBildaPinpointDriver odo;
    Telemetry telemetry;

    double turnInRadians;
    double error = Double.MAX_VALUE;

    AngleTracker angleTracker = new AngleTracker();
    double angleTurnedInRadians = 0;


    public TurnAngleRelativeCommand(AutoMecanumDriveTrain driveTrain, Telemetry telemetry, double turnInDegrees) {
        this.driveTrain = driveTrain;
        odo = driveTrain.getOdo();
        this.telemetry = telemetry;
        this.turnInRadians = Math.toRadians(turnInDegrees);
    }

    @Override
    public void initialize() {
        driveTrain.resetOdo();
        odo.update();

        // record start point
        angleTracker.update(odo.getHeading());
    }

    @Override
    protected boolean isTargetReached() {
        return Math.abs(error) < Math.toRadians(1);
    }

    @Override
    public void execute() {
        odo.update();
        double heading = odo.getHeading();
        angleTurnedInRadians = angleTracker.update(heading);
        error = turnInRadians - angleTurnedInRadians;
        if (isTargetReached()) {
            finished.set(true);
            return;
        }

        double power = error * -0.02;
        if(Math.abs(power) < min) {
            power = min * Math.signum(power);
        }

        Log.i(LOG_TAG, String.format("heading=%f, error=%f, power=%f", heading, error, power));

        telemetry.addData("heading", heading);
        telemetry.addData("error", error);
        telemetry.addData("power", power);
        telemetry.update();

        driveTrain.driveRobotCentric(0, power, 0);

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.stop();
        driveTrain.resetOdo();
    }
}
