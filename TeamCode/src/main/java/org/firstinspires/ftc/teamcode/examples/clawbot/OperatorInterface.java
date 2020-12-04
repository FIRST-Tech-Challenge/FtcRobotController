package org.firstinspires.ftc.teamcode.examples.clawbot;

import com.technototes.library.control.gamepad.CommandGamepad;

public class OperatorInterface {
    //the gamepad to control the robot with
    public CommandGamepad gamepad;
    public Robot robot;
    //create the operator interface
    public OperatorInterface(CommandGamepad g, Robot r){
        gamepad = g;
        robot = r;

    }

}
