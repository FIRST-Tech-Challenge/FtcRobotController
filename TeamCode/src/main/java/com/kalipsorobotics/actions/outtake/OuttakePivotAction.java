package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.Servo;

//0.3 in
//0.65 out
public class OuttakePivotAction {

    final private Outtake outtake;
    private final Servo outtakePivotServo;

    private boolean isIn = true;

    public OuttakePivotAction(Outtake outtake) {
        this.outtake = outtake;
        outtakePivotServo = outtake.getOuttakePivotServo();
    }

    public void setPosition(double position) {
        outtakePivotServo.setPosition(position);
    }

    public void moveIn() {
        setPosition(0.3);
        isIn = true;
    }

    public void moveOut() {
        setPosition(0.7);
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
