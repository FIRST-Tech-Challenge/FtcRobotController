package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.rustlib.commandsystem.Command;

import java.util.function.DoubleSupplier;

public class SlideDefault extends Command {
    Slide slide;
    DoubleSupplier speedSupplier;

    public SlideDefault(Slide slide, DoubleSupplier speedSupplier) {
        this.slide = slide;
        this.speedSupplier = speedSupplier;
        addRequirements(this.slide);
    }

    @Override
    public void execute() {
        slide.mizoom(speedSupplier.getAsDouble());
    }
}
