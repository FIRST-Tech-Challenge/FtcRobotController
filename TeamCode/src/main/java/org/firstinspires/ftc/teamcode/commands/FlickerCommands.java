package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

public class FlickerCommands extends Command {

    public FlickerSubsystem flicker;
    public UpliftTele opMode;
    UpliftRobot robot;

    public FlickerCommands(UpliftTele opMode, UpliftRobot robot, FlickerSubsystem flickerSubsystem) {
        super(opMode, flickerSubsystem);
        this.flicker = flickerSubsystem;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void init() {
        // set flicker to start position (back)
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(opMode.gamepad1.right_bumper) {
            flicker.flickRing();
        }

        // Method for shooting (only flicks rings)
        if(opMode.gamepad1.y) {
            flicker.setFlickerPos(0.15);
            for(int i = 0; i < 2; i++) {
                flicker.flickRing();
                robot.safeSleep(100);
            }
            flicker.flickRing();
            robot.setShootingState(UpliftRobot.ShootingState.DONE_SHOOTING);
        }

    }

    @Override
    public void stop() {

    }
}
