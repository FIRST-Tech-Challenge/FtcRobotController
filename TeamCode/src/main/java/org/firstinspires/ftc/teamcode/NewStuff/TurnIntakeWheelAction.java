package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TurnIntakeWheelAction extends Action {

    DcMotor intakeWheel;
    final double ERROR_TOLERANCE = 20;
    final double P_CONSTANT = 0.004;
    double targetTicks;
    double currentTicks;
    double error;


    public TurnIntakeWheelAction(Action dependentAction, double targetTicks, Intake intake) {
        intakeWheel = intake.wheelMotor;
        this.dependentAction = dependentAction;
        this.targetTicks = targetTicks;
    }

    public TurnIntakeWheelAction(double targetTicks, Intake intake) {
        intakeWheel = intake.wheelMotor;
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
    boolean checkDoneCondition() {
        refreshError();
        if (error <= ERROR_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    void update() {
        this.currentTicks = intakeWheel.getCurrentPosition();
        intakeWheel.setPower(calculatePower());
    }
}
