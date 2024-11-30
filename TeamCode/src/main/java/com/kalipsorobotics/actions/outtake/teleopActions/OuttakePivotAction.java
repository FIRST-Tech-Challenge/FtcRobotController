package com.kalipsorobotics.actions.outtake.teleopActions;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.Servo;

//0.925 in
//0.0 out
public class OuttakePivotAction {

    final private Outtake outtake;
    private final KServo outtakePivotServo;
    public static double OUTTAKE_PIVOT_IN_POS = 1.0;
    public static double OUTTAKE_PIVOT_OUT_POS = 0.05;
    public static double OUTTAKE_PIVOT_HALF_POS = 0.5;



    private boolean isIn = true;

    public OuttakePivotAction(Outtake outtake) {
        this.outtake = outtake;
        outtakePivotServo = outtake.getOuttakePivotServo();
    }

    public void setPosition(double position) {
        outtakePivotServo.setPosition(position);
    }

    public void moveIn() {
        setPosition(OUTTAKE_PIVOT_HALF_POS);
        setPosition(OUTTAKE_PIVOT_IN_POS);
        isIn = true;
    }

    public void moveOut() {
        setPosition(OUTTAKE_PIVOT_OUT_POS);
        isIn = false;
    }

    public void togglePosition() {
        if (!isIn) {
            moveIn();
        } else {
            moveOut();
        }
    }

    public Outtake getOuttake() {
        return outtake;
    }

}
