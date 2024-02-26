package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class ZeroWristPositionCommand extends CommandBase {

    WristSubsystem subsystem;

    public ZeroWristPositionCommand(WristSubsystem subsystem) {
        super();
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    boolean executed = false;

    @Override
    public void execute() {
        subsystem.getWrist().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        subsystem.getWrist().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        executed = true;
    }

    @Override
    public boolean isFinished() {
        return executed;
    }
}
