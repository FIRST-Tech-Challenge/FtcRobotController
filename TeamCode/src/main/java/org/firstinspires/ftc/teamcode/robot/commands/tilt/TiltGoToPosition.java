package org.firstinspires.ftc.teamcode.robot.commands.tilt;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.TiltSubsystem;

/**
 * open claw
 */
public class TiltGoToPosition extends CommandBase
{
    public static int TELEOP_INTAKE = 0; // floor intake
    public static int TELEOP_DEPOSIT = 90+30; // 30 degree deposit
    public static int AUTO_DEPOSIT = 0; // kooky-deposit angle
    public static  int AUTO_STACK_INTAKE1 = 0; // top two pixels off stacks
    public static int AUTO_STACK_INTAKE2 = 0; // 3rd and 4th pixels off stacks

    private final TiltSubsystem tiltSubsystem;
    private final double targetAngle;

    public TiltGoToPosition(TiltSubsystem subsystem, double targetAngle)
    {
        tiltSubsystem = subsystem;
        this.targetAngle = targetAngle;
        addRequirements(tiltSubsystem);
    }

    @Override
    public void initialize()
    {
        tiltSubsystem.setTargetAngle(targetAngle);
    }

    @Override
    public boolean isFinished()
    {
        return tiltSubsystem.atTargetPosition();
    }
}
