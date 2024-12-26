package com.kalipsorobotics.utilities;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;

public class KGamePad {

    private Gamepad gamepad;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousButtonB = false;
    private boolean previousButtonX = false;
    private boolean previousButtonA = false;
    private boolean previousButtonY = false;
    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;

    public KGamePad(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public Gamepad getGamepad (){
        return gamepad;
    }

    public boolean isToggleDpadLeft (){
        boolean current = gamepad.dpad_left;
        boolean toggle = false;
        if (this.previousDpadLeft == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This Dpad left toggle");
        }

        this.previousDpadLeft = current;
        return toggle;
    }

    public boolean isToggleDpadRight(){
        boolean current = gamepad.dpad_right;
        boolean toggle = false;
        if (this.previousDpadRight == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This Dpad right toggle");
        }
        this.previousDpadRight = current;
        return toggle;
    }

    public boolean isToggleDpadUp(){
        boolean current = gamepad.dpad_up;
        boolean toggle = false;
        if (this.previousDpadUp == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This Dpad up toggle");
        }
        this.previousDpadUp = current;
        return toggle;
    }

    public boolean isToggleDpadDown(){
        boolean current = gamepad.dpad_down;
        boolean toggle = false;
        if (this.previousDpadDown == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This Dpad down toggle");
        }
        this.previousDpadDown = current;
        return toggle;
    }

    public boolean isToggleButtonB(){
        boolean current = gamepad.b;
        boolean toggle = false;
        if(this.previousButtonB == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This button B toggle");
        }
        this.previousButtonB = current;
        return toggle;
    }

    public boolean isToggleButtonX(){
        boolean current = gamepad.x;
        boolean toggle = false;
        if(this.previousButtonX == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This button X toggle");
        }
        this.previousButtonX = current;
        return toggle;
    }

    public boolean isToggleButtonA(){
        boolean current = gamepad.a;
        boolean toggle = false;
        if(this.previousButtonA == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This button A toggle");
        }
        this.previousButtonA = current;
        return toggle;
    }

    public boolean isToggleButtonY(){
        boolean current = gamepad.y;
        boolean toggle = false;
        if(this.previousButtonY == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This button Y toggle");
        }
        this.previousButtonY = current;
        return toggle;
    }

    public boolean isToggleRightBumper(){
        boolean current = gamepad.right_bumper;
        boolean toggle = false;
        if(this.previousRightBumper == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This right bumper toggle");
        }
        this.previousRightBumper = current;
        return toggle;
    }

    public boolean isToggleLeftBumper(){
        boolean current = gamepad.left_bumper;
        boolean toggle = false;
        if(this.previousLeftBumper == false && current == true){
            toggle = true;
            Log.d("KGamePad", "This left bumper toggle");
        }
        this.previousLeftBumper = current;
        return toggle;
    }

    public boolean isRightTriggerPressed(){

        if (gamepad.right_trigger > 0.9){
           return true;
        }
        return false;
    }

    public boolean isLeftTriggerPressed(){
        if (gamepad.left_trigger > 0.9){
            return true;
        }
        return false;
    }

}
