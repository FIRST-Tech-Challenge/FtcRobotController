package com.kalipsorobotics.actions;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.MathFunctions;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;

public class TurnRobotAction extends Action {

    DriveTrain driveTrain;
    PIDController controller;
    Odometry odometry;
    double targetDegrees;
    double currentHeading;
    double ERROR_TOLERANCE = 0.5; // degrees
    double error;

    public TurnRobotAction(double targetDegrees, DriveTrain driveTrain, Odometry odometry) {
        this.dependentAction = new DoneStateAction();
        this.targetDegrees = targetDegrees;
        this.driveTrain = driveTrain;
        this.odometry = odometry;
        this.controller = new PIDController(0.2, 0.01, 0.01, "turn");  // placeholder values
    }

    public double getCurrentHeading() {
        return Math.toDegrees(odometry.countTheta());
    }

    private void refreshError() {
        error = MathFunctions.angleWrapDeg(targetDegrees - currentHeading);
    }

    @Override
    public boolean checkDoneCondition() {
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
    public void update() {
        double power;
        double minPower = 0.15;

        currentHeading = getCurrentHeading();
        refreshError();
        power = Range.clip(controller.calculate(currentHeading, targetDegrees), -0.7, 0.7);

        if (power > 0 && power < minPower) {
            power = minPower;
        } else if (power < 0 && power > -1 * minPower) {
            power = minPower * -1;
        }

        driveTrain.setPower(-power, power, -power, power);
    }
}
