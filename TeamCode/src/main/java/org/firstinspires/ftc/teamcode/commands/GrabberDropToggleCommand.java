package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Grabber;

public class GrabberDropToggleCommand extends CommandBase {

    private Grabber grabber;

    public GrabberDropToggleCommand(Grabber grabber) {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        grabber.drop();
    }

    @Override
    public void end(boolean interrupted) {
        grabber.stop();
    }
}
