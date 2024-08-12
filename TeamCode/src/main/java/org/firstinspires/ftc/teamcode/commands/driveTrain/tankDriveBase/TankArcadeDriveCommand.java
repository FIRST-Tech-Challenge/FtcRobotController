package org.firstinspires.ftc.teamcode.commands.driveTrain.tankDriveBase;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.TankDriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleCommandBase;

public class TankArcadeDriveCommand extends SympleCommandBase<TankDriveBaseSubsystem> {
    private final GamepadEx controller;

    public TankArcadeDriveCommand(TankDriveBaseSubsystem driveBase, GamepadEx controller) {
        super(driveBase);
        this.controller = controller;
    }

    @Override
    public void execute() {
        double speedModifier = 1 - (this.controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5f);

        double rotationSpeed = controller.getRightX() * 0.5f * (this.subsystem.isInverted() ? -1 : 1) * speedModifier;
        double linerSpeed = controller.getLeftY() * 0.8f * (this.subsystem.isInverted() ? -1 : 1) * speedModifier;

        double rawLeftSpeed = (linerSpeed + rotationSpeed);
        double rawRightSpeed = (linerSpeed - rotationSpeed);

        double normalizedLeftSpeed = rawLeftSpeed;
        double normalizedRightSpeed = rawRightSpeed;

        double absRightSpeed = Math.abs(rawRightSpeed);
        double absLeftSpeed = Math.abs(rawLeftSpeed);
        double maxValue = Math.max(absLeftSpeed, absRightSpeed);

        if (maxValue >= 1) {
            normalizedLeftSpeed /= maxValue;
            normalizedRightSpeed /= maxValue;
        }

        this.subsystem.moveMotors(normalizedLeftSpeed, normalizedRightSpeed);
    }
}