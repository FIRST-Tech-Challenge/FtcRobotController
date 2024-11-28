package com.kalipsorobotics.actions.outtake.teleopActions;

import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.Servo;
//open 0.85
//closed 0.95
public class OuttakeClawAction {

    final private Outtake outtake;
    final private Servo outtakeClawServo;

    private boolean isClosed = true;
    public OuttakeClawAction(Outtake outtake) {
        this.outtake = outtake;
        this.outtakeClawServo = outtake.getOuttakeClawServo();
    }

    public void setPosition(double position) {
        outtakeClawServo.setPosition(position);
    }

    public void close() {
        setPosition(0.45);
        isClosed = true;
    }

    public void open() {
        setPosition(0.65);
        isClosed = false;
    }

    public void togglePosition() {
        if (!isClosed) {
            close();
        } else {
            open();
        }
    }



}
