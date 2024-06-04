package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.rustlib.commandsystem.Command;

public class SlideToPosition extends Command {
    private final Slide subsystem;
    private final int position;

    public SlideToPosition(Slide slide, int position) {
        subsystem = slide;
        this.position = position;
    }

    @Override
    public void initialize() {
        subsystem.setTargetPosition(position);
    }

    @Override
    public void execute() {

    }
}
