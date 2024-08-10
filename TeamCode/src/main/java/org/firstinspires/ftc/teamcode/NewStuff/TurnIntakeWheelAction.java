package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TurnIntakeWheelAction extends Action {

    DcMotor intakeWheel;
    final double ERROR_TOLERANCE = 20;
    final double P_CONSTANT = 0.004;
    Action dependentAction;
    DoneStateAction doneStateAction = new DoneStateAction();
    double targetTicks;
    double currentTicks;
    double error;
    boolean isDone = false;


    public TurnIntakeWheelAction(Action dependentAction, double targetTicks, Intake intake) {
        intakeWheel = intake.wheelMotor;
        this.dependentAction = dependentAction;
        this.targetTicks = targetTicks;
    }

    public TurnIntakeWheelAction(double targetTicks, Intake intake) {
        intakeWheel = intake.wheelMotor;
        this.dependentAction = doneStateAction;
        this.targetTicks = targetTicks;
    }

    //TODO make private
    public double calculatePower() {
        refreshError();
        return error * P_CONSTANT;
    }

    @Override
    public void update() {
        Log.d("parallelaction", "entered turn wheel");
        if (isDone) { return; } //if i'm done never update
        if (!dependentAction.getIsDone()) { return; } //if dependent action is not done never update
        Log.d("parallelaction", "should turn wheel");
        this.currentTicks = intakeWheel.getCurrentPosition();
        intakeWheel.setPower(calculatePower());

        updateIsDone();
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
    }

    @Override
    boolean updateIsDone() {
        refreshError();
        if (error <= ERROR_TOLERANCE) {
            isDone = true;
        }
        return isDone;
    }

    @Override
    boolean getIsDone() {
        return isDone;
    }

    @Override
    void setDependentAction(Action newAction) {
        this.dependentAction = newAction;
    }

    @Override
    Action getDependentAction() {
        return this.dependentAction;
    }
}
