package org.firstinspires.ftc.teamcode.teamUtil;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEX {

    private final Gamepad gamepad;

    public GamepadEX(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    /*
    public boolean isPressed(){
        switch (this){
            case a:
                return gamepad.a;
            case b:
                return gamepad.b;
            case x:
                return gamepad.x;
            case y:
                return gamepad.y;
            case dpad_up:
                return gamepad.dpad_up;
            case dpad_down:
                return gamepad.dpad_down;
            case dpad_left:
                return gamepad.dpad_left;
            case dpad_right:
                return gamepad.dpad_right;
            case right_bumper:
                return gamepad.right_bumper;
            case left_bumper:
                return gamepad.left_bumper;
        }
        return false;
    }
     */

    public enum buttons{
        a,
        b,
        x,
        y,
        dpad_up,
        dpad_down,
        dpad_left,
        dpad_right,
        right_bumper,
        left_bumper
    }

    enum triggers{
        leftX,
        leftY,
        rightX,
        rightY,
        left_trigger,
        right_trigger
    }
}
