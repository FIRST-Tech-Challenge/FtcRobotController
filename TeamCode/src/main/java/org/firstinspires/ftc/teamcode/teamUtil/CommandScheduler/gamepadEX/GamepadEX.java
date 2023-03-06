package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEX {

    public ButtonEX a;
    public ButtonEX b;
    public ButtonEX x;
    public ButtonEX y;

    public ButtonEX dpad_up;
    public ButtonEX dpad_down;
    public ButtonEX dpad_left;
    public ButtonEX dpad_right;

    public ButtonEX left_bumper;
    public ButtonEX right_bumper;

    public ContinuousInput leftX;
    public ContinuousInput leftY;
    public ContinuousInput rightX;
    public ContinuousInput rightY;

    public ContinuousInput left_trigger;
    public ContinuousInput right_trigger;

    Gamepad gamepad;

    public GamepadEX(Gamepad gamepad){
        this.gamepad = gamepad;

        a = new ButtonEX();
        b = new ButtonEX();
        x = new ButtonEX();
        y = new ButtonEX();

        dpad_up = new ButtonEX();
        dpad_down = new ButtonEX();
        dpad_left = new ButtonEX();
        dpad_right = new ButtonEX();

        left_bumper = new ButtonEX();
        right_bumper = new ButtonEX();

        leftX = new ContinuousInput(continuousInputs.leftX, gamepad);
        leftY = new ContinuousInput(continuousInputs.leftY, gamepad);
        rightX = new ContinuousInput(continuousInputs.rightX, gamepad);
        rightY = new ContinuousInput(continuousInputs.rightY, gamepad);

        left_trigger = new ContinuousInput(continuousInputs.left_trigger, gamepad);
        right_trigger = new ContinuousInput(continuousInputs.right_trigger, gamepad);
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

