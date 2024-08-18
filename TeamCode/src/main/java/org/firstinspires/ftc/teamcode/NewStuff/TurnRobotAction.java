package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TurnRobotAction extends Action {

    DriveTrain driveTrain;
    IMU imu;
    double targetDegrees;

    public TurnRobotAction(Action dependentAction, double targetDegrees, DriveTrain driveTrain, IMU imu) {
        this.dependentAction = dependentAction;
        this.targetDegrees = targetDegrees;
        this.driveTrain = driveTrain;
        this.imu = imu;
    }

    public TurnRobotAction(double targetDegrees, DriveTrain driveTrain, IMU imu) {
        this.dependentAction = new DoneStateAction();
        this.targetDegrees = targetDegrees;
        this.driveTrain = driveTrain;
        this.imu = imu;
    }

    public double getCurrentHeading() {
        double currentYaw;
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // Loop here if IMU is bad (usually because battery power is low)
        while ((robotOrientation.getAcquisitionTime() == 0) && opMode.opModeIsActive()) {
            Log.d("imu crash", "getCurrentHeading: acquisition time is 0");
            driveTrain.
            robotOrientation = imu.getRobotYawPitchRollAngles();
        }

        currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        return currentYaw;
    }

    @Override
    boolean checkDoneCondition() {
        return false;
    }

    @Override
    void update() {
        YawPitchRollAngles robotOrientation;
        double KP = 0.06;
        double KD = 2_500_000;
        double ERROR_TOLERANCE = 0.5; //degrees
        double currentHeading = getCurrentHeading();
        double error = angleWrap(targetAbsDegrees - currentHeading);
        double errorDer;
        double power;
        double currentTime;
        double minPower = 0.15;

        //while start
        while (Math.abs(error) > ERROR_TOLERANCE && opMode.opModeIsActive()) {
            currentHeading = getCurrentHeading();

            currentTime = SystemClock.elapsedRealtimeNanos();
            error = targetAbsDegrees - currentHeading; //error is degrees to goal
            errorDer = (error - prevError) / (currentTime - prevTime);
            power = (KP * error) + (KD * errorDer);

                /*
                Log.d("pid", "setHeading: current heading is " + currentHeading);
                Log.d("pid", "setHeading: Target heading is " + targetAbsDegrees);
                Log.d("pid", "setHeading: time is " + currentTime);
                Log.d("pid", "setHeading: heading error is " + error);
                Log.d("pid", "setHeading: errorDer is " + errorDer);
                Log.d("pid", "setHeading: calculated power is " + power);
                */

            if (power > 0 && power < minPower) {
                power = minPower;
                // Log.d("pid", "setHeading: adjusted power is " + power);
            } else if (power < 0 && power > -1 * minPower) {
                power = minPower * -1;
                // Log.d("pid", "setHeading: adjusted power is " + power);
            }

            //cap power
            power = Range.clip(power, -1 * maxPower, maxPower);
            // Log.d("pid", "straightBlockingFixHeading: power after clipping is " + power);

            drivetrain.setMotorPower(-1 * power, power, -1 * power, power);
            prevError = error;
            prevTime = currentTime;
        }
        drivetrain.setMotorPower(0, 0, 0, 0);
        opMode.sleep(100);
        currentHeading = getCurrentHeading();
        botHeading = targetAbsDegrees;
        Log.d("pid", "setHeading: final heading is " + currentHeading);
    }
}
