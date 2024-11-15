package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import com.arcrobotics.ftclib.command.CommandBase;

public class GrabberDropCommand extends CommandBase {

    private Grabber grabber;

    public GrabberDropCommand(Grabber grabber) {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void execute() {
        grabber.drop();
    }

    @Override
    public void end(boolean interrupted) {
        grabber.stop();
    }

}
