package org.firstinspires.ftc.teamcode.commands.index;

import org.firstinspires.ftc.teamcode.subsystems.IndexSubsystem;

public class SendAllRingsToShooterCommand extends SendRingToShooterCommand {
    public SendAllRingsToShooterCommand(IndexSubsystem subsystem) {
        super(subsystem);
    }
    @Override
    public boolean isFinished() {
        return super.isFinished()&&indexSubsystem.getNumRings()==0;
    }
}
