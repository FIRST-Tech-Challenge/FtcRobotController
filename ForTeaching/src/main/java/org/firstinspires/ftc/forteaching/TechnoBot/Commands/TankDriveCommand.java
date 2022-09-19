package org.firstinspires.ftc.forteaching.TechnoBot.Commands;

import com.technototes.library.command.Command;
import com.technototes.library.control.CommandAxis;
import com.technototes.library.control.CommandButton;

import org.firstinspires.ftc.forteaching.TechnoBot.Subsystems.TankDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TankDriveCommand implements Command {
    TankDriveSubsystem subsys;
    DoubleSupplier left, right;
    BooleanSupplier shouldStraighten;

    public TankDriveCommand(TankDriveSubsystem dbSubsystem, CommandAxis l, CommandAxis r, CommandButton straighten) {
        subsys = dbSubsystem;
        addRequirements(subsys);
        left = l;
        right = r;
        shouldStraighten = straighten;
    }

    public TankDriveCommand(TankDriveSubsystem dbSubsystem, CommandAxis l, CommandAxis r) {
        subsys = dbSubsystem;
        addRequirements(subsys);
        left = l;
        right = r;
        shouldStraighten = null;
    }

    @Override
    public void execute() {
        double lp = left.getAsDouble();
        double rp = right.getAsDouble();
        // TODO: Make shouldStraighten work by adjusting the power accordingly (need the IMU too)
        subsys.drive(lp, rp);
    }

    // This command is never finished, cuz it's the manual drive mode
    @Override
    public boolean isFinished() {
        return false;
    }
}
