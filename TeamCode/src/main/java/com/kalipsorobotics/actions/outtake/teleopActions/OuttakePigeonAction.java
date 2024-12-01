package com.kalipsorobotics.actions.outtake.teleopActions;

import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakePigeonAction {

    private final Outtake outtake;
    private final KServo outtakePigeonServo;

    final public static double OUTTAKE_PIGEON_IN_POS = 0.524;

    private boolean isIn;
    public OuttakePigeonAction(Outtake outtake) {
        this.outtake = outtake;
        this.outtakePigeonServo = outtake.getOuttakePigeonServo();
    }

    public void setPosition(double position) {
        outtakePigeonServo.setPosition(position);
    }

    public void moveIn() {
        setPosition(OUTTAKE_PIGEON_IN_POS);
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
