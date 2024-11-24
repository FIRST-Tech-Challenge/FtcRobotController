package com.kalipsorobotics.actions.hang;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class HangHooks {
    OpModeUtilities opModeUtilities;
    private Servo hangLeft;
   //private final Servo hangRight;

    public HangHooks(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUp();
    }
    public void setUp() {
        hangLeft = opModeUtilities.getHardwareMap().servo.get("hang1");
        //this.hangRight = opModeUtilities.getHardwareMap().get(Servo.class, "hang2");
    }

    private void setHookPosition(Double position) {
        hangLeft.setPosition(position);
        //hangRight.setPosition(position);
    }

    public void pivot() {
        double middle = 45; //middle degree from folded in to fully extended
        if (hangLeft.getPosition() > middle) { setHookPosition(60.0); } //Position of hook servo when hooks are folded in
        else { setHookPosition(30.0); } //Position of hook servo when hook servo when hooks are extended out
    }
    public double getHookPosition(String RL) {
        if (RL.equalsIgnoreCase("l")) return(hangLeft.getPosition());
       // else if (RL.equalsIgnoreCase("r")) return(hangRight.getPosition());
        else return(0.0);
    }
}


