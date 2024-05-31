package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Climber;

import java.util.function.DoubleSupplier;

public class ClimbDefault extends Command {

    private final Climber climber;
    private final DoubleSupplier speed;

    public ClimbDefault(Climber climber, DoubleSupplier speed) {
        this.climber = climber;
        this.speed = speed;
    }

    @Override
    public void execute() {
        climber.winch(speed.getAsDouble());
    }
}
