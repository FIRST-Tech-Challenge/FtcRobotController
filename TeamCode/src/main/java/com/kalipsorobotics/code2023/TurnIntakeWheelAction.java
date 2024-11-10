package com.kalipsorobotics.code2023;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;

class TurnIntakeWheelAction extends Action {

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
    public boolean checkDoneCondition() {
        refreshError();
        if (Math.abs(error) <= ERROR_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        if(!hasStarted) {
            this.targetTicks += currentTicks;
            hasStarted = true;
        }

        this.currentTicks = intakeWheel.getCurrentPosition();
        intakeWheel.setPower(calculatePower());
    }
}
