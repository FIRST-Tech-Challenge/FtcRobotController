package org.firstinspires.ftc.teamcode.NewStuff.actions.code2023;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewStuff.actions.Action;
import org.firstinspires.ftc.teamcode.NewStuff.actions.DoneStateAction;
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
    public boolean checkDoneCondition() {
        if(trayClamp.getPosition() == targetPos) {
            outtake.getOpModeUtilities().getOpMode().sleep(100);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        trayClamp.setPosition(targetPos);
    }
}
