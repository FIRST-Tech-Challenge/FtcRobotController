package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSub;

public class MoveLinearSlide extends CommandBase {
    private final LinearSlideSub linearSlideSub;
    private final Telemetry telemetry;
    private final double speed;

    public MoveLinearSlide(LinearSlideSub linearSlideSub, Telemetry telemetry, double speed) {
        this.linearSlideSub = linearSlideSub;
        this.telemetry = telemetry;
        this.speed = speed;
    }

    @Override
    public void execute() {
        linearSlideSub.move(speed);
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
