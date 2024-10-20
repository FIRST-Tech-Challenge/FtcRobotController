package org.firstinspires.ftc.teamcode.NewStuff.actions;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewStuff.modules.Outtake;

public class MoveTrayClampAction extends Action {

    Servo trayClamp;
    Outtake outtake;
    double targetPos;

    public static double CLAMP_OPEN_POS = 0.471;
    public static double CLAMP_CLOSE_POS = 0.57;

    public MoveTrayClampAction(Action dependentAction, double targetPos, Outtake outtake) {
        this.outtake = outtake;
        this.trayClamp = outtake.clamp;
        this.dependentAction = dependentAction;
        this.targetPos = targetPos;
    }

    public MoveTrayClampAction(double targetPos, Outtake outtake) {
        this.outtake = outtake;
        this.trayClamp = outtake.clamp;
        this.dependentAction = new DoneStateAction();
        this.targetPos = targetPos;
    }

    @Override
    boolean checkDoneCondition() {
        if(trayClamp.getPosition() == targetPos) {
            outtake.getOpModeUtilities().getOpMode().sleep(100);
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
