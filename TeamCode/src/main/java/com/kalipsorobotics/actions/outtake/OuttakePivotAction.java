package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.Servo;

//0.925 in
//0.0 out
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
        setPosition(0.5);
        setPosition(0.925);
        isIn = true;
    }
    public double getPosition() {
        return outtakePivotServo.getPosition();
    }

    public void moveOut() {
        setPosition(0.0);
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
