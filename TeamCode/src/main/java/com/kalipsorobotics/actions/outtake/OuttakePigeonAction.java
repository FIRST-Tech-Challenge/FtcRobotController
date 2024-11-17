package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakePigeonAction {

    private final Outtake outtake;
    private final Servo outtakePigeonServo;

    private boolean isIn;
    public OuttakePigeonAction(Outtake outtake) {
        this.outtake = outtake;
        this.outtakePigeonServo = outtake.getOuttakePigeonServo();
    }

    public void setPosition(double position) {
        outtakePigeonServo.setPosition(position);
    }

    public void moveIn() {
        setPosition(0.524);
        isIn = true;
    }

    public void moveOut() {
        //Place holder
        setPosition(0.524);
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
