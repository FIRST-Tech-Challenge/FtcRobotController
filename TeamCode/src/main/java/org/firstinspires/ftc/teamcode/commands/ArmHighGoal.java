package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;

public class ArmHighGoal extends CommandBase {
    private final ArmSub armSub;
    private Telemetry telemetry;

    public ArmHighGoal(ArmSub armSub, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.armSub = armSub;
    }

    public ArmHighGoal(ArmSub armSub, DrivetrainSub drivetrain) {
        this.armSub = armSub;
        addRequirements(armSub, drivetrain);
    }

    @Override
    public void execute() {
        armSub.armHighGoal();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
