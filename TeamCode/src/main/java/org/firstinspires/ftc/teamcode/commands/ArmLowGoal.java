package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;

public class ArmLowGoal extends CommandBase {
    private final ArmSub armSub;
    private final GamepadEx gamepad;
    private Telemetry telemetry;

    public ArmLowGoal(ArmSub armSub, GamepadEx gamepad, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.armSub = armSub;
        this.gamepad = gamepad;
    }

    @Override
    public void execute() {
        armSub.armLowGoal();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
