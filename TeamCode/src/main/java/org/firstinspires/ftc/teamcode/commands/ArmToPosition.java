package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMecanum;

public class ArmToPosition extends CommandBase {
    private Arm m_arm;
    private Integer m_setpoint;
    private Telemetry m_telemetry;


    public ArmToPosition(Arm arm, Integer setpoint, Telemetry telemetry) {
        m_arm = arm;
        m_setpoint = setpoint;
        m_telemetry = telemetry;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_arm.driveToSetPoint(m_setpoint);

    }

    @Override
    public boolean isFinished() { return true; }
}
