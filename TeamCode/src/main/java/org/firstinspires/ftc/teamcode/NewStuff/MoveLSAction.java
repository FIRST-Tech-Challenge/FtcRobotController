package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MoveLSAction extends Action {

    DcMotor lsFront;
    DcMotor lsBack;
    final double ERROR_TOLERANCE = 50;
    final double P_CONSTANT = 0.003;
    double targetTicks;
    double currentTicks;
    double error;


    public MoveLSAction(Action dependentAction, double targetTicks, Outtake outtake) {
        lsFront = outtake.lsFront;
        lsBack = outtake.lsBack;
        this.dependentAction = dependentAction;
        this.targetTicks = targetTicks;
    }

    public MoveLSAction(double targetTicks, Outtake outtake) {
        lsFront = outtake.lsFront;
        lsBack = outtake.lsBack;
        this.dependentAction = new DoneStateAction();
        this.targetTicks = targetTicks;
    }

    private double calculatePower() {
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
        this.currentTicks = lsFront.getCurrentPosition();

        if(!hasStarted) {
            this.targetTicks += currentTicks;
            hasStarted = true;
        }

        lsFront.setPower(calculatePower());
        lsBack.setPower(calculatePower());
    }
}
