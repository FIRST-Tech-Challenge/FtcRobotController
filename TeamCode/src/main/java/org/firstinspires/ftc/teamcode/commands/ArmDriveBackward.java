package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ArmDriveBackward extends CommandBase {
    private Arm m_arm;
    private Telemetry m_telemetry;

    public ArmDriveBackward(Arm arm, Telemetry telemetry) {
        m_arm = arm;
        m_telemetry = telemetry;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_arm.drive(-0.5);
    }

    @Override
    public boolean isFinished() { return false; }
}
