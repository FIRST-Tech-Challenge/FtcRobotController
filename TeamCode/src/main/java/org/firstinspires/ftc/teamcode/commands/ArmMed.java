package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;

public class ArmMed extends CommandBase {
    private final ArmSub armSub;

    public ArmMed(ArmSub armSub, Telemetry telemetry) {
        this.armSub = armSub;
        addRequirements(armSub);
    }

    public ArmMed(ArmSub armSub, DrivetrainSub drivetrain){
        this.armSub = armSub;
        addRequirements(armSub, drivetrain);
    }

    @Override
    public void execute() {
        armSub.armMed();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
