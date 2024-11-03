package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.tankDriveBase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.TankDriveSubsystem;

public class TankArcadeDriveCommand extends CommandBase {
    private final GamepadEx controller;

    private final TankDriveSubsystem subsystem;

    public TankArcadeDriveCommand(TankDriveSubsystem subsystem, GamepadEx controller) {
        this.subsystem = subsystem;
        this.controller = controller;

        addRequirements(subsystem);
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