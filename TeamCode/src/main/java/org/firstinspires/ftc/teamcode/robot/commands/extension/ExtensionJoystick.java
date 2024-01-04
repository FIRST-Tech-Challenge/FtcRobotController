package org.firstinspires.ftc.teamcode.robot.commands.extension;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.ExtensionSubsystem;

import java.util.function.DoubleSupplier;

/**
 * open claw
 */
public class ExtensionJoystick extends CommandBase
{
    private final ExtensionSubsystem extensionSubsystem;
    private final DoubleSupplier power;
    public ExtensionJoystick(ExtensionSubsystem subsystem, DoubleSupplier power)
    {
        extensionSubsystem = subsystem;
        this.power = power;
        addRequirements(extensionSubsystem);
    }

    @Override
    public void execute()
    {
        extensionSubsystem.manualControl(power.getAsDouble());
    }
}
