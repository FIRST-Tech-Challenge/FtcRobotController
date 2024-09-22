package org.firstinspires.ftc.teamcode.NewStuff;

import com.qualcomm.robotcore.hardware.Servo;

public class PivotTrayAction extends Action{

    Servo trayPivot;
    Outtake outtake;

    double targetPos;

    public static double TRAY_INTAKE_POS = 0.3;
    public static double TRAY_OUTTAKE_POS = 0;

    public PivotTrayAction(Action dependentAction, double targetPos, Outtake outtake) {
        this.dependentAction = dependentAction;
        this.targetPos = targetPos;
        this.outtake = outtake;
        this.trayPivot = outtake.tray;
    }

    public PivotTrayAction(double targetPos, Outtake outtake) {
        this.dependentAction = new DoneStateAction();
        this.targetPos = targetPos;
        this.outtake = outtake;
        this.trayPivot = outtake.tray;
    }

    @Override
    boolean checkDoneCondition() {
        if(trayPivot.getPosition() == targetPos) {
            outtake.getOpModeUtilities().getOpMode().sleep(100);
            return true;
        } else {
            return false;
        }
    }

    @Override
    void update() {
        trayPivot.setPosition(targetPos);
    }
}
