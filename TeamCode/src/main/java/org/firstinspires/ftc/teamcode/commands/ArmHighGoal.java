package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;

public class ArmHighGoal extends CommandBase {
    private final ArmSub armSub;
    private final GamepadEx gamepad;
    private Telemetry telemetry;

    public ArmHighGoal(ArmSub armSub, GamepadEx gamepad, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.armSub = armSub;
        this.gamepad = gamepad;
    }

    public ArmHighGoal(ArmSub armSub, DrivetrainSub drivetrain) {
        this.armSub = armSub;
        //this is to fix a complaint of the compiler bc i'm trying to just update code on another robot. Don't trust it.
        this.gamepad = null;
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
