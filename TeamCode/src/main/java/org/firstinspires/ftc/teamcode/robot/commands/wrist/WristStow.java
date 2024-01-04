package org.firstinspires.ftc.teamcode.robot.commands.wrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.WristSubsystem;

public class WristStow extends CommandBase {
    private final WristSubsystem wristSubsystem;
    public WristStow(WristSubsystem subsystem)
    {
        wristSubsystem = subsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        wristSubsystem.setWristStow();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
