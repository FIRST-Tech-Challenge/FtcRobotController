package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;

import java.util.function.DoubleSupplier;

public class MoveArm extends CommandBase {
    private final ArmSub armSub;
    private final DoubleSupplier armY;
    private final Telemetry telemetry;

    public MoveArm(ArmSub armSub, DoubleSupplier armY, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.armSub = armSub;
        this.armY = armY;
        addRequirements(this.armSub);

    }

    @Override
    public void execute() {
        if (this.armY.getAsDouble() != 0) {
            armSub.moveArm(this.armY.getAsDouble());
        }
    }
}
