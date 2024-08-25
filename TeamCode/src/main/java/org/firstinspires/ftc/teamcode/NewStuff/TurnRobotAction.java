package org.firstinspires.ftc.teamcode.NewStuff;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TurnRobotAction extends Action {

    DriveTrain driveTrain;
    IMU imu;
    double targetDegrees;
    double prevError;
    double prevTime;
    double currentHeading;
    double KP = 0.06;
    double KD = 2_500_000;
    double ERROR_TOLERANCE = 0.5; //degrees
    double error;

    public TurnRobotAction(Action dependentAction, double targetDegrees, DriveTrain driveTrain, IMUModule imu) {
        this.dependentAction = dependentAction;
        this.targetDegrees = targetDegrees;
        this.driveTrain = driveTrain;
        this.imu = imu.getIMU();
    }

    public TurnRobotAction(double targetDegrees, DriveTrain driveTrain, IMUModule imu) {
        this.dependentAction = new DoneStateAction();
        this.targetDegrees = targetDegrees;
        this.driveTrain = driveTrain;
        this.imu = imu.getIMU();
    }

    public double getCurrentHeading() {
        double currentYaw;
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // Loop here if IMU is bad (usually because battery power is low)
        while ((robotOrientation.getAcquisitionTime() == 0) && driveTrain.getOpModeUtilities().getOpMode().opModeIsActive()) {
            Log.d("imu crash", "getCurrentHeading: acquisition time is 0");
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
            robotOrientation = imu.getRobotYawPitchRollAngles();
        }

        currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        return currentYaw;
    }

    private void refreshError() {
        error = angleWrap(targetDegrees - currentHeading);
    }

    @Override
    boolean checkDoneCondition() {
        refreshError();
        if (Math.abs(error) <= ERROR_TOLERANCE) {
            driveTrain.setPower(0,0,0,0);
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
            currentHeading = getCurrentHeading();
            targetDegrees = 0;
            Log.d("pid", "setHeading: final heading is " + currentHeading);
            return true;
        } else {
            return false;
        }
    }

    @Override
    void update() {
        double errorDer;
        double power;
        double currentTime;
        double minPower = 0.15;

        currentHeading = getCurrentHeading();

        currentTime = SystemClock.elapsedRealtimeNanos();
        refreshError(); //error is degrees to goal
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
        power = Range.clip(power, -0.7, 0.7);
        // Log.d("pid", "straightBlockingFixHeading: power after clipping is " + power);

        driveTrain.setPower(-1 * power, power, -1 * power, power);
        prevError = error;
        prevTime = currentTime;

    }

    public static double angleWrap (double angle) {
        double moddedAngle = angle % 360;
        if (moddedAngle > 180) {
            return moddedAngle - 360;
        } else if (moddedAngle <= -180) {
            return moddedAngle + 360;
        }
        return moddedAngle;
    }
}
