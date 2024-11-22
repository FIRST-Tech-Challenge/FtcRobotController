package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Grabber;

public class GrabberPickupToggleCommand extends CommandBase {

    private Grabber grabber;

    public GrabberPickupToggleCommand(Grabber grabber) {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        grabber.pickup();
    }

    @Override
    public void end(boolean interrupted) {
        grabber.stop();
    }
}
