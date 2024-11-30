package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import android.util.Log;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;

public class RoboticCentricDriveCommand extends SounderBotCommandBase {
    private static final String LOG_TAG = RoboticCentricDriveCommand.class.getSimpleName();
    AutoMecanumDriveTrain driveTrain;

    double strafeSpeed, forwardSpeed, rotSpeed;

    long timeToDriveMs;

    long startTime = Long.MIN_VALUE;

    public RoboticCentricDriveCommand(AutoMecanumDriveTrain driveTrain, double strafeSpeed, double forwardSpeed, double rotSpeed, long timeToDriveMs) {
        this.driveTrain = driveTrain;
        this.strafeSpeed = strafeSpeed;
        this.forwardSpeed = forwardSpeed;
        this.rotSpeed = rotSpeed;
        this.timeToDriveMs = timeToDriveMs;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.resetOdo();
    }

    @Override
    public void doExecute() {
        if (startTime < 0) {
            startTime = System.currentTimeMillis();
        }
        driveTrain.driveRobotCentric(strafeSpeed, forwardSpeed, rotSpeed);
        Log.i(LOG_TAG, "strafeSpeed: " + strafeSpeed + " forwardSpeed: " + forwardSpeed + " rotSpeed:" + rotSpeed);
        Log.i(LOG_TAG, "time passed: " + (System.currentTimeMillis() - startTime));
        if (isTargetReached()) {
            Log.i(LOG_TAG, "Time out!!!");
            driveTrain.stop();
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
        finished = true;
    }

    @Override
    protected boolean isTargetReached() {
        return System.currentTimeMillis() - startTime >= timeToDriveMs;
    }
}
