package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ArmDriveForward extends CommandBase {
    private Arm m_arm;

    public ArmDriveForward(Arm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() { m_arm.drive(1.0); }

    @Override
    public boolean isFinished() { return false; }
}
