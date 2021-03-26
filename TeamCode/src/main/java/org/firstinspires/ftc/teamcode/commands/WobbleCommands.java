package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;

public class WobbleCommands extends Command {

    public WobbleSubsystem wobble;
    public UpliftTele opMode;
    UpliftRobot robot;
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
        if (opMode.gamepad2.dpad_up) {
            wobble.pickUpTeleop();
        }

        if (opMode.gamepad2.dpad_down) {
            wobble.dropOff();
        }

        if(opMode.gamepad2.dpad_right){
            wobble.endgameWobble();
            wobble.openWobble();
        }

    }


    @Override
    public void stop(){

    }

}
