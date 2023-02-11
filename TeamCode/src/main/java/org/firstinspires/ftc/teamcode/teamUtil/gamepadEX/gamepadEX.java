package org.firstinspires.ftc.teamcode.teamUtil.gamepadEX;

import com.qualcomm.robotcore.hardware.Gamepad;

public class gamepadEX {

    public button a;
    public button b;
    public button x;
    public button y;

    public button dpad_up;
    public button dpad_down;
    public button dpad_left;
    public button dpad_right;

    public button left_bumper;
    public button right_bumper;

    public continuousInput leftX;
    public continuousInput leftY;
    public continuousInput rightX;
    public continuousInput rightY;

    public continuousInput left_trigger;
    public continuousInput right_trigger;

    Gamepad gamepad;

    public gamepadEX(Gamepad gamepad){
        this.gamepad = gamepad;

        a = new button();
        b = new button();
        x = new button();
        y = new button();

        dpad_up = new button();
        dpad_down = new button();
        dpad_left = new button();
        dpad_right = new button();

        left_bumper = new button();
        right_bumper = new button();

        leftX = new continuousInput(continuousInputs.leftX, gamepad);
        leftY = new continuousInput(continuousInputs.leftY, gamepad);
        rightX = new continuousInput(continuousInputs.rightX, gamepad);
        rightY = new continuousInput(continuousInputs.rightY, gamepad);

        left_trigger = new continuousInput(continuousInputs.left_trigger, gamepad);
        right_trigger = new continuousInput(continuousInputs.right_trigger, gamepad);
    }

    public enum continuousInputs{
        leftX,
        leftY,
        rightX,
        rightY,
        left_trigger,
        right_trigger
    }

    public void startLoopUpdate(){
        a.startLoopUpdate(gamepad.a);
        b.startLoopUpdate(gamepad.b);
        x.startLoopUpdate(gamepad.x);
        y.startLoopUpdate(gamepad.y);

        dpad_up.startLoopUpdate(gamepad.dpad_up);
        dpad_down.startLoopUpdate(gamepad.dpad_down);
        dpad_left.startLoopUpdate(gamepad.dpad_left);
        dpad_right.startLoopUpdate(gamepad.dpad_right);

        left_bumper.startLoopUpdate(gamepad.left_bumper);
        right_bumper.startLoopUpdate(gamepad.right_bumper);
    }

    public void endLoopUpdate(){
        a.endLoopUpdate();
        b.endLoopUpdate();
        x.endLoopUpdate();
        y.endLoopUpdate();
        dpad_up.endLoopUpdate();
        dpad_down.endLoopUpdate();
        dpad_left.endLoopUpdate();
        dpad_right.endLoopUpdate();
        left_bumper.endLoopUpdate();
        right_bumper.endLoopUpdate();
    }
}

