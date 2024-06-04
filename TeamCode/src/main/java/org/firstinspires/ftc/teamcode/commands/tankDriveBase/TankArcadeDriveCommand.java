package org.firstinspires.ftc.teamcode.commands.tankDriveBase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.TankDriveBaseSubSystem;

public class TankArcadeDriveCommand extends CommandBase {
    private final TankDriveBaseSubSystem driveBase;
    private final GamepadEx controller;

    public TankArcadeDriveCommand(TankDriveBaseSubSystem driveBase, GamepadEx controller) {
        addRequirements(driveBase);
        this.driveBase = driveBase;
        this.controller = controller;
    }

    @Override
    public void execute() {
        double speedModifier = 1 - (this.controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5f);

        double rotationSpeed = controller.getRightX() * 0.5f * (this.driveBase.isInverted() ? -1 : 1) * speedModifier;
        double linerSpeed = controller.getLeftY() * 0.8f * (this.driveBase.isInverted() ? -1 : 1) * speedModifier;

        double rawLeftSpeed = (linerSpeed + rotationSpeed);
        double rawRightSpeed = (linerSpeed - rotationSpeed);

        double normalizedLeftSpeed = rawLeftSpeed;
        double normalizedRightSpeed = rawRightSpeed;


        double absRightSpeed = Math.abs(rawRightSpeed);
        double absLeftSpeed = Math.abs(rawLeftSpeed);
        double maxValue = Math.max(absLeftSpeed, absRightSpeed);

        if (maxValue >= 1) {
            normalizedLeftSpeed = rawLeftSpeed / maxValue;
            normalizedRightSpeed = rawRightSpeed / maxValue;
        }

        this.driveBase.moveMotors(normalizedLeftSpeed, normalizedRightSpeed);
    }
}