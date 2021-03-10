package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SticksSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;

public class SticksCommands extends Command {
    public SticksSubsystem sticks;
    public UpliftTele opMode;
    UpliftRobot robot;

    public SticksCommands(UpliftTele opMode, UpliftRobot robot, SticksSubsystem sticksSubsystem) {
        super(opMode, sticksSubsystem);
        this.sticks = sticksSubsystem;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void init() {
        sticks.dropSticks();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }
}
