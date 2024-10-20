package org.firstinspires.ftc.teamcode.NewStuff.actions;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.NewStuff.modules.Outtake;

public class MoveLSAction extends Action {

    DcMotor lsFront;
    DcMotor lsBack;
    Outtake outtake;
    final double ERROR_TOLERANCE = 50;
    final double P_CONSTANT = 0.0035;
    double targetTicks;
    double currentTicks;
    double error;


    public MoveLSAction(Action dependentAction, double targetTicks, Outtake outtake) {
        this.outtake = outtake;
        lsFront = outtake.lsFront;
        lsBack = outtake.lsBack;
        this.dependentAction = dependentAction;
        this.targetTicks = targetTicks;
    }

    public MoveLSAction(double targetTicks, Outtake outtake) {
        this.outtake = outtake;
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
        Log.d("movels", "error is " + error);
        if (Math.abs(error) <= ERROR_TOLERANCE) {
            lsFront.setPower(0);
            lsBack.setPower(0);
            //outtake.getOpModeUtilities().getOpMode().sleep(100);
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
