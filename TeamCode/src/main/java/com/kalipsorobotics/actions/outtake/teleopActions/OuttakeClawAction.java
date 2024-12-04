package com.kalipsorobotics.actions.outtake.teleopActions;

import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.Servo;
//open 0.85
//closed 0.95
public class OuttakeClawAction {

    final private Outtake outtake;
    final private KServo outtakeClawServo;
    final static public double OUTTAKE_CLAW_OPEN_POS = 0.65;
    final static public double OUTTAKE_CLAW_CLOSE_POS = 0.45;

    private boolean isClosed = true;
    public OuttakeClawAction(Outtake outtake) {
        this.outtake = outtake;
        this.outtakeClawServo = outtake.getOuttakeClawServo();
    }

    public void setPosition(double position) {
        outtakeClawServo.setPosition(position);
    }

    public void close() {
        setPosition(OUTTAKE_CLAW_CLOSE_POS);
        isClosed = true;
    }

    public void open() {
        setPosition(OUTTAKE_CLAW_OPEN_POS);
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
