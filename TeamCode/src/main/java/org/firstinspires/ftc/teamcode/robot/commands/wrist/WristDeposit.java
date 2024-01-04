package org.firstinspires.ftc.teamcode.robot.commands.wrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.WristSubsystem;

public class WristDeposit extends CommandBase {
    private final WristSubsystem wristSubsystem;
    public WristDeposit(WristSubsystem subsystem)
    {
        wristSubsystem = subsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        wristSubsystem.setWristDeposit();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
