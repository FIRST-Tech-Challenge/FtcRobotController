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



    public gamepadEX(Gamepad gamepad){
        a = new button(buttons.a, gamepad);
        b = new button(buttons.b, gamepad);
        x = new button(buttons.x, gamepad);
        y = new button(buttons.y, gamepad);

        dpad_up = new button(buttons.dpad_up, gamepad);
        dpad_down = new button(buttons.dpad_down, gamepad);
        dpad_left = new button(buttons.dpad_left, gamepad);
        dpad_right = new button(buttons.dpad_right, gamepad);

        left_bumper = new button(buttons.left_bumper, gamepad);
        right_bumper = new button(buttons.right_bumper, gamepad);

        leftX = new continuousInput(continuousInputs.leftX, gamepad);
        leftY = new continuousInput(continuousInputs.leftY, gamepad);
        rightX = new continuousInput(continuousInputs.rightX, gamepad);
        rightY = new continuousInput(continuousInputs.rightY, gamepad);

        left_trigger = new continuousInput(continuousInputs.left_trigger, gamepad);
        right_trigger = new continuousInput(continuousInputs.right_trigger, gamepad);
    }

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

    public enum continuousInputs{
        leftX,
        leftY,
        rightX,
        rightY,
        left_trigger,
        right_trigger
    }

    public void action(button.buttonCondition buttonCondition, button.buttonAction buttonAction){
        if(buttonCondition.buttonCondition()){
            buttonAction.buttonAction();
        }
    }

    public void update(){
        a.update();
        b.update();
        x.update();
        y.update();
        dpad_up.update();
        dpad_down.update();
        dpad_left.update();
        dpad_right.update();
        left_bumper.update();
        right_bumper.update();
    }
}

