package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestMotorSubsystem;

public class TestMotorCommand extends CommandBase {

    private final TestMotorSubsystem mTestMotorSubsystem;
    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;
    private  int mCompteur;

    public TestMotorCommand(Telemetry telemetry, TestMotorSubsystem testMotorSubsystem, HardwareMap hardwareMap){
        mTelemetry = telemetry;
        mTestMotorSubsystem = testMotorSubsystem;
        mHardwareMap = hardwareMap;

        addRequirements(testMotorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mCompteur = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mCompteur++;
        mTestMotorSubsystem.test();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        mTestMotorSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mCompteur >= 100;
    }
}
