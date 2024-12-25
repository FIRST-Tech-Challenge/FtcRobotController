package com.kalipsorobotics.utilities;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;

public class KGamePad {

    private Gamepad gamepad;
    private boolean previousDpadLeft = false;

    public KGamePad(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public Gamepad getGamepad (){
        return gamepad;
    }

    public boolean isToggleDpadLeft (){
        boolean current = gamepad.dpad_left;
        boolean toggle = false;
        Log.d("KGamePad", "This is previous: "+previousDpadLeft + " This is current: " + current);
        if(this.previousDpadLeft == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This Dpad left toggle");
        }

        this.previousDpadLeft = current;
        return toggle;
    }
}
