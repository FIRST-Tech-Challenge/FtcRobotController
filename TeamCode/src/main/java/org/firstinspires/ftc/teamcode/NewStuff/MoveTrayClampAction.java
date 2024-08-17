package org.firstinspires.ftc.teamcode.NewStuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MoveTrayClampAction extends Action {

    Servo trayClamp;
    double targetPos;

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
    boolean checkDoneCondition() {
        if(trayClamp.getPosition() == targetPos) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    void update() {
        trayClamp.setPosition(targetPos);
    }
}
