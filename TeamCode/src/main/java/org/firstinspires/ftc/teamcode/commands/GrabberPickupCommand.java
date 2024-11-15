package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import com.arcrobotics.ftclib.command.CommandBase;

public class GrabberPickupCommand extends CommandBase {

    private Grabber grabber;

    public GrabberPickupCommand(Grabber grabber) {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void execute() {
        grabber.pickup();
    }

    @Override
    public void end(boolean interrupted) {
        grabber.stop();
    }

}
