package com.kalipsorobotics.localization;

import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class PigeonHead {
    OpModeUtilities opModeUtilities;
    private final Servo servo;

    public PigeonHead(Servo servo) {
        this.servo = servo;
    }

    public double positionUpdate(double headingChange) {
        return(-headingChange);
    }

    public double getPigeonHeadPos(Servo servo) {
        return(servo.getPosition());
    }

    public void setPigeonHeadPos(Servo servo, double position) {
        servo.setPosition(position);
    }
    public void resetPigeonHeadPos(Servo servo) {
        servo.setPosition(0);
    }
}
