package org.firstinspires.ftc.teamcode.commands.mechanumDrive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.MechanumDriveSubsystem;

public class MechanumArcadeDriveCommand extends CommandBase {
    private final MechanumDriveSubsystem mechanumDriveSubsystem;
    private final GamepadEx gamepad;

    public MechanumArcadeDriveCommand(MechanumDriveSubsystem mechanumDriveSubsystem, GamepadEx gamepad) {
        this.mechanumDriveSubsystem = mechanumDriveSubsystem;
        this.gamepad = gamepad;

        addRequirements(this.mechanumDriveSubsystem);
    }

    @Override
    public void execute() {
        double vSpeed = this.gamepad.getLeftY();
        double hSpeed = this.gamepad.getLeftX();
        double rotationSpeed = this.gamepad.getRightX();

        double rawFrontRightSpeed = vSpeed + rotationSpeed + hSpeed;
        double rawBackRightSpeed = vSpeed + rotationSpeed - hSpeed;
        double rawFrontLeftSpeed = vSpeed - rotationSpeed - hSpeed;
        double rawBackLeftSpeed = vSpeed - rotationSpeed + hSpeed;

        double normalizedFrontRightSpeed = rawFrontRightSpeed;
        double normalizedBackRightSpeed = rawBackRightSpeed;
        double normalizedFrontLeftSpeed = rawFrontLeftSpeed;
        double normalizedBackLeftSpeed = rawBackLeftSpeed;

        double absFrontRightSpeed = Math.abs(rawFrontRightSpeed);
        double absBackRightSpeed = Math.abs(rawBackRightSpeed);
        double absFrontLeftSpeed = Math.abs(rawFrontLeftSpeed);
        double absBackLeftSpeed = Math.abs(rawBackLeftSpeed);

        double maxSpeed = Math.max(absFrontRightSpeed, Math.max(absBackRightSpeed, Math.max(absFrontLeftSpeed, absBackLeftSpeed)));

        if(maxSpeed > 1) {
            normalizedFrontRightSpeed /= maxSpeed;
            normalizedBackRightSpeed /= maxSpeed;
            normalizedFrontLeftSpeed /= maxSpeed;
            normalizedBackLeftSpeed /= maxSpeed;
        }

        this.mechanumDriveSubsystem.moveMotor(MechanumDriveSubsystem.MotorNames.FRONT_RIGHT, normalizedFrontRightSpeed);
        this.mechanumDriveSubsystem.moveMotor(MechanumDriveSubsystem.MotorNames.BACK_RIGHT, normalizedBackRightSpeed);
        this.mechanumDriveSubsystem.moveMotor(MechanumDriveSubsystem.MotorNames.FRONT_LEFT, normalizedFrontLeftSpeed);
        this.mechanumDriveSubsystem.moveMotor(MechanumDriveSubsystem.MotorNames.BACK_LEFT, normalizedBackLeftSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        this.mechanumDriveSubsystem.moveMotor(MechanumDriveSubsystem.MotorNames.FRONT_RIGHT, 0);
        this.mechanumDriveSubsystem.moveMotor(MechanumDriveSubsystem.MotorNames.BACK_RIGHT, 0);
        this.mechanumDriveSubsystem.moveMotor(MechanumDriveSubsystem.MotorNames.FRONT_LEFT, 0);
        this.mechanumDriveSubsystem.moveMotor(MechanumDriveSubsystem.MotorNames.BACK_LEFT, 0);
    }
}
