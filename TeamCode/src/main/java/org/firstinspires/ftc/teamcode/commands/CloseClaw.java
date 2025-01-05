package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class CloseClaw extends CommandBase {
    public Claw claw;

    public CloseClaw(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }
    public void execute(){
        claw.closeClaw();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
