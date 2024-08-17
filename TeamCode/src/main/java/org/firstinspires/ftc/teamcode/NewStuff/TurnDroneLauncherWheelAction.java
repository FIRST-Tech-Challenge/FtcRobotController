package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TurnDroneLauncherWheelAction extends Action {

    DcMotor wheel;
    final double ERROR_TOLERANCE = 20;
    final double P_CONSTANT = 0.004;
    double targetTicks;
    double currentTicks;
    double error;


    public TurnDroneLauncherWheelAction(Action dependentAction, double targetTicks, DroneLauncher droneLauncher) {
        wheel = droneLauncher.wheel;
        this.dependentAction = dependentAction;
        this.targetTicks = targetTicks;
    }

    public TurnDroneLauncherWheelAction(double targetTicks, DroneLauncher droneLauncher) {
        wheel = droneLauncher.wheel;
        this.dependentAction = doneStateAction;
        this.targetTicks = targetTicks;
    }

    //TODO make private
    public double calculatePower() {
        refreshError();
        return error * P_CONSTANT;
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
    }

    @Override
    boolean checkDoneCondition() {
        refreshError();
        if (error <= ERROR_TOLERANCE) {
            return true;
        }
        return false;
    }

    @Override
    void update() {
        this.currentTicks = wheel.getCurrentPosition();
        wheel.setPower(calculatePower());
    }

}
