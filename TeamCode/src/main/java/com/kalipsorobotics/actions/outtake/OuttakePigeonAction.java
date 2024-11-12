package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakePigeonAction {

    private final Outtake outtake;
    private final Servo outtakePigeonServo;

    private boolean isAlignedWithRobot;
    public OuttakePigeonAction(Outtake outtake) {
        this.outtake = outtake;
        this.outtakePigeonServo = outtake.getOuttakePigeonServo();
    }

    public void setPosition(double position) {
        outtakePigeonServo.setPosition(position);
    }

    public void moveIn() {
        setPosition(0.3);
        isAlignedWithRobot = true;
    }

    public void moveOut() {
        setPosition(0.65);
        isAlignedWithRobot = false;
    }

    public void togglePosition() {
        if (!isAlignedWithRobot) {
            moveIn();
        } else {
            moveOut();
        }
    }

    public Outtake getOuttake() {
        return outtake;
    }

}
