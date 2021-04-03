package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;

public class WobbleCommands extends Command {

    public WobbleSubsystem wobble;
    public UpliftTele opMode;
    UpliftRobot robot;
    boolean wobbleButtonPressed = false;
    int clickCount = 0;
    int oldCount = 0;
    boolean isPressed = false;

    public WobbleCommands(UpliftTele opMode, UpliftRobot robot, WobbleSubsystem wobbleSubsystem) {
        super(opMode, wobbleSubsystem);
        this.wobble = wobbleSubsystem;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void init() {
        wobble.setWobblePosition(0.5);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
//        Log.i("wobbleValue",clickCount + "");
        oldCount = clickCount;
        if(opMode.gamepad2.dpad_right) {
            if(!wobbleButtonPressed) {
                clickCount = clickCount + 1;
                if (clickCount > 3) {
                    clickCount = 1;
                }
                wobbleButtonPressed = true;
            }
        } else {
            wobbleButtonPressed = false;
        }

        if (oldCount != clickCount) {
            if (clickCount == 1) {
                wobble.dropOff();
            } else if (clickCount == 2) {
                wobble.closeWobble();
                wobble.endgameWobble();
            } else if (clickCount == 3) {
                wobble.endgameWobble();
                wobble.openWobble();
            }
        }
    }


    @Override
    public void stop(){

    }

}
