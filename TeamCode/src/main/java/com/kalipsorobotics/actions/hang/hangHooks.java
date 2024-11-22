package com.kalipsorobotics.actions.hang;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class hangHooks {
    OpModeUtilities opModeUtilities;
    private final Servo hangLeft;
    private final Servo hangRight;

    public hangHooks(Servo servo) {
        this.hangLeft = opModeUtilities.getHardwareMap().get(Servo.class, "hang1");
        this.hangRight = opModeUtilities.getHardwareMap().get(Servo.class, "hang2");
    }

    private void setHookPosition(Double position) {
        hangLeft.setPosition(position);
    }

    public void pivot() {
        double middle = 45; //middle degree from folded in to fully extended
        if (hangLeft.getPosition() > middle) { setHookPosition(60.0); } //Position of hook servo when hooks are folded in
        else { setHookPosition(30.0); } //Position of hook servo when hook servo when hooks are extended out
    }
}
