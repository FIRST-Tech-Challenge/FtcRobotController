package org.firstinspires.ftc.teamcode.Commands.DefaultCommands;


import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class DriveDefault implements Command {

    private Drive drive;
    private String name = "Drive";

    public DriveDefault(Drive drive){
        this.drive = drive;
    }

    @Override
    public void start() {

    }

    @Override
    public void execute() {
        drive.stop();
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}