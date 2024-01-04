package org.firstinspires.ftc.teamcode.robot.commands.wrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.WristSubsystem;

public class WristIntake extends CommandBase {
    private final WristSubsystem wristSubsystem;
    public WristIntake(WristSubsystem subsystem)
    {
        wristSubsystem = subsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        wristSubsystem.setWristIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
