package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSub;

public class ResetLinearSlide extends CommandBase {
    private final LinearSlideSub linearSlideSub;

    public ResetLinearSlide(LinearSlideSub linearSlideSub) {
        this.linearSlideSub = linearSlideSub;
        addRequirements(this.linearSlideSub);

    }

    @Override
    public void execute() {
        linearSlideSub.resetEncoder();
    }
}
