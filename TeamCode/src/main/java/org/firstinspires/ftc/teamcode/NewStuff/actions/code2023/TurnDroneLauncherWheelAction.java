package org.firstinspires.ftc.teamcode.NewStuff.actions.code2023;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NewStuff.actions.Action;
import org.firstinspires.ftc.teamcode.NewStuff.actions.DoneStateAction;

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
        this.dependentAction = new DoneStateAction();
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
    public boolean checkDoneCondition() {
        refreshError();
        if (Math.abs(error) <= ERROR_TOLERANCE) {
            return true;
        }
        return false;
    }

    @Override
    public void update() {
        this.currentTicks = wheel.getCurrentPosition();

        if(!hasStarted) {
            this.targetTicks += currentTicks;
            hasStarted = true;
        }

        wheel.setPower(calculatePower());
    }

}
