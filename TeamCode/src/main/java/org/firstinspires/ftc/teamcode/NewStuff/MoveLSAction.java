package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MoveLSAction extends Action {

    DcMotor lsFront;
    DcMotor lsBack;
    final double ERROR_TOLERANCE = 50;
    final double P_CONSTANT = 0.003;
    Action dependentAction;
    DoneStateAction doneStateAction = new DoneStateAction();;
    double targetTicks;
    double currentTicks;
    double error;
    boolean isDone = false;


    public MoveLSAction(Action dependentAction, double targetTicks, Outtake outtake) {
        lsFront = outtake.lsFront;
        lsBack = outtake.lsBack;
        this.dependentAction = dependentAction;
        this.targetTicks = targetTicks;
    }

    public MoveLSAction(double targetTicks, Outtake outtake) {
        lsFront = outtake.lsFront;
        lsBack = outtake.lsBack;
        this.dependentAction = doneStateAction;
        this.targetTicks = targetTicks;
    }

    private double calculatePower() {
        refreshError();
        return error * P_CONSTANT;
    }

    @Override
    public void update() {
        Log.d("parallelaction", "entered move ls");
        if (isDone) { return; } //if i'm done never update
        if (!dependentAction.getIsDone()) { return; } //if dependent action is not done never update

        Log.d("parallelaction", "should move ls");
        this.currentTicks = lsFront.getCurrentPosition();
        lsFront.setPower(calculatePower());
        lsBack.setPower(calculatePower());

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
        } else {
            isDone = false;
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
