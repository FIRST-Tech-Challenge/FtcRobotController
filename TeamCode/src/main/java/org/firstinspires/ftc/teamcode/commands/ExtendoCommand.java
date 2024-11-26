package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendoSystem;
import static org.firstinspires.ftc.teamcode.subsystems.ExtendoSystem.Direction;

public class ExtendoCommand extends CommandBase {

    ExtendoSystem subsystem;
    Direction direction;

    public ExtendoCommand(ExtendoSystem subsystem, Direction direction) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        subsystem.setDirection(direction);
    }
}
