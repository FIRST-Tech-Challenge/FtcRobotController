package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSub;

import java.util.function.DoubleSupplier;

public class MoveLinearSlide extends CommandBase {
    private final LinearSlideSub linearSlideSub;
    private final Telemetry telemetry;
    private final DoubleSupplier leftY;

    public MoveLinearSlide(LinearSlideSub linearSlideSub, DoubleSupplier leftY, Telemetry telemetry) {
        this.linearSlideSub = linearSlideSub;
        this.telemetry = telemetry;
        this.leftY = leftY;
        addRequirements(this.linearSlideSub);

    }

    @Override
    public void execute() {
        linearSlideSub.move(this.leftY.getAsDouble());
    }
}
