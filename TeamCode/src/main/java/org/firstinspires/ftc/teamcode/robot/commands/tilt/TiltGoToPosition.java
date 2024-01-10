package org.firstinspires.ftc.teamcode.robot.commands.tilt;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.TiltSubsystem;

/**
 * open claw
 */
public class TiltGoToPosition extends CommandBase
{
    public static final int TELEOP_INTAKE = -46; // floor intake
    public static final int TELEOP_DEPOSIT =- 770; // 30 degree deposit
    public static final int AUTO_DEPOSIT = -300; // kooky-deposit angle
    public static final int AUTO_STACK_INTAKE1 = -600; // top two pixels off stacks
    public static final int AUTO_STACK_INTAKE2 = -600; // 3rd and 4th pixels off stacks

    private final TiltSubsystem tiltSubsystem;
    private final int targetPosition;

    public TiltGoToPosition(TiltSubsystem subsystem, int targetPosition)
    {
        tiltSubsystem = subsystem;
        this.targetPosition = targetPosition;
        addRequirements(tiltSubsystem);
    }

    @Override
    public void initialize()
    {
        tiltSubsystem.setTargetPosition(targetPosition);
    }

    @Override
    public boolean isFinished()
    {
        return tiltSubsystem.atTargetPosition();
    }
}
