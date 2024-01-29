package org.firstinspires.ftc.teamcode.robot.commands.extension;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.ExtensionSubsystem;

/**
 * open claw
 */
public class ExtensionGoToPosition extends CommandBase
{
    public static final int FULL_EXTENSION = 2300;
    public static final int ONE_STAGE_EXTENSION = 1700;
    public static final int INTAKE_LOW_POSITION = 0;
    public static final int INTAKE_FAR_POSITION = 0;
    public static final int STOW_POSITION = ExtensionSubsystem.UNEXTENDED_POSITION;

    private final ExtensionSubsystem extensionSubsystem;
    private final int targetPosition;
    public ExtensionGoToPosition(ExtensionSubsystem subsystem, int targetPosition)
    {
        extensionSubsystem = subsystem;
        this.targetPosition = targetPosition;
        addRequirements(extensionSubsystem);
    }

    @Override
    public void execute()
    {
        extensionSubsystem.goToPosition(targetPosition);
    }

    // the command has to be interrupted
}
