package org.firstinspires.ftc.teamcode.NewStuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MoveTrayClampAction extends Action {

    Servo trayClamp;
    Action dependentAction;
    DoneStateAction doneStateAction = new DoneStateAction();
    double targetPos;
    boolean isDone = false;

    public MoveTrayClampAction(Action dependentAction, double targetPos, Outtake outtake) {
        this.trayClamp = outtake.clamp;
        this.dependentAction = dependentAction;
        this.targetPos = targetPos;
    }

    public MoveTrayClampAction(double targetPos, Outtake outtake) {
        this.trayClamp = outtake.clamp;
        this.dependentAction = doneStateAction;
        this.targetPos = targetPos;
    }

    @Override
    boolean updateIsDone() {
        if(trayClamp.getPosition() == targetPos) {
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

    @Override
    void update() {
        if (isDone) { return; } //if i'm done never update
        if (!dependentAction.getIsDone()) { return; } //if dependent action is not done never update

        trayClamp.setPosition(targetPos);

        updateIsDone();
    }
}
