package org.firstinspires.ftc.teamcode.opmodes.autonomous.movement;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.driver.AutoRobotDriver;

import java.util.function.Supplier;

public abstract class BaseMovement {

    static final String LOG_TAG = BaseMovement.class.getSimpleName();
    private static final double NORMAL_SPEED = 0.5d;
    private static final double SLOW_SPEED = 0.2d;

    private static final double FORWARD_FACTOR = 1d;
    private static final double BACKWARD_FACTOR = -1d;
    private static final double ERROR_TOLERANCE = 0.01d;
    private static final double ERROR_CORRECTION_CONVERSION = 5; //this converts distance to motor speed
    double currentSpeed;
    double distance;

    Pose2d startingPose;
    Pose2d previousPose;

    AutoRobotDriver driver;

    PIDController errorCorrectionController = new PIDController(0.5,0,0);


    Supplier<Pose2d> poseSupplier;

    double directionFactor;

    double accumulatedChanges;

    boolean finished;

    double error;

    double strafeSpeed, forwardSpeed, rotationSpeed;
    public BaseMovement(double distance) {
        Log.i(LOG_TAG, String.format("distance = %f", distance));
        this.distance = distance;
        finished = false;
        directionFactor = distance > 0 ? FORWARD_FACTOR : BACKWARD_FACTOR;
    }

    public void start(double startPosition, AutoRobotDriver driver, Supplier<Pose2d> poseSupplier) {
        Log.i(LOG_TAG, String.format("startPosition = %f", startPosition));
        startingPose = poseSupplier.get();
        previousPose = startingPose;
        this.poseSupplier = poseSupplier;
        accumulatedChanges = 0;
        this.driver = driver;
        errorCorrectionController.setSetPoint(0);
        errorCorrectionController.setTolerance(ERROR_TOLERANCE);
        error = 0;
        currentSpeed = directionFactor * NORMAL_SPEED;
        updateDrivingParameters();
        doDrive();
    }

    /**
     * Update the drive speed or stop if necessary
     */
    public void onEachCycle() {
        if (finished) {
            return;
        }

        Pose2d newPose = poseSupplier.get();

        double newReading = getCurrentDrivingReading(newPose);
        double prevReading = getCurrentDrivingReading(previousPose);

        if (isSuddenMove(prevReading, newReading)) {

            // rotation could have sudden change from 3.135 to - 3.132
            // we still count the difference between 3.135 and 3.132
            if (!isSuddenMove(prevReading, -newReading)) {
                accumulatedChanges += Math.abs(-newReading - prevReading);
            }
            previousPose = newPose;
            return;
        }

        Log.i(LOG_TAG, String.format("newReading = %f", newReading));

        accumulatedChanges += Math.abs(newReading - prevReading);
        previousPose = newPose;
        Log.i(LOG_TAG, String.format("%s accumulated/target = %f/%f, progress = %f", getClass().getSimpleName(), accumulatedChanges, Math.abs(distance), getProgress()));
        if (getProgress() >= 1) {
            driver.stop();
            finished = true;
            return;
        }

        if (isCloseToEnd()) {
            currentSpeed = SLOW_SPEED * directionFactor;
        }

        updateDrivingParameters();
        doDrive();
    }

    private boolean isCloseToEnd() {
        return  getProgress() >= 0.8;
    }

    private double getProgress() {
        return  Math.abs(accumulatedChanges / distance);
    }

    public boolean isFinished() {
        return finished;
    }

    protected void doDrive() {
        driver.drive(strafeSpeed, forwardSpeed, rotationSpeed);
    }

    protected boolean isSuddenMove(double prePos, double currPos) {
        return Math.abs((currPos - prePos) / distance) > .5;
    }

    protected double getErrorCorrectionSpeed(double error) {
        Log.i(LOG_TAG, String.format("error = %f", error));
        double correctionDistance = errorCorrectionController.calculate(error);
        if (error*correctionDistance<0) {
            correctionDistance = -correctionDistance;
        }
        return correctionDistance*ERROR_CORRECTION_CONVERSION;
    }

    protected abstract void updateDrivingParameters();
    protected abstract double getCurrentDrivingReading(Pose2d pose);

}
