package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;

public class StrafeInAngleMecanumCommand extends CommandBase {
    private final double angle;
    private final double meters;

    private double STARTING_FORWARD_DIST = 0;
    private double STARTING_SIDE_DIST = 0;
    private PController pController;

    private final MecanumDriveSubsystem subsystem;

    public StrafeInAngleMecanumCommand(MecanumDriveSubsystem subsystem, double angle, double meters) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        this.angle = angle + 90;
        this.meters = meters;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.STARTING_FORWARD_DIST = this.subsystem.getForwardDistanceDriven();
        this.STARTING_SIDE_DIST = this.subsystem.getSideDistanceDriven();

        this.pController = new PController(0.8);
        this.pController.setTolerance(0.02);
        this.pController.setSetPoint(this.meters);
    }

    @Override
    public void execute() {
        double hSpeed = Math.cos(Math.toRadians(angle)) * meters;
        double vSpeed = Math.sin(Math.toRadians(angle)) * meters;


        double forwardDistanceMoved = this.subsystem.getForwardDistanceDriven() - this.STARTING_FORWARD_DIST;
        double sideDistanceMoved = this.subsystem.getSideDistanceDriven() - this.STARTING_SIDE_DIST;

        double currentDist = Math.hypot(forwardDistanceMoved, sideDistanceMoved); // => √x*x + y*y
        double powerMultiplier = this.pController.calculate(currentDist);

        double rawFrontRightSpeed = (vSpeed + hSpeed) * powerMultiplier;
        double rawBackRightSpeed = (vSpeed - hSpeed) * powerMultiplier;
        double rawFrontLeftSpeed = (vSpeed - hSpeed) * powerMultiplier;
        double rawBackLeftSpeed = (vSpeed + hSpeed) * powerMultiplier;

        double normalizedFrontRightSpeed = rawFrontRightSpeed;
        double normalizedBackRightSpeed = rawBackRightSpeed;
        double normalizedFrontLeftSpeed = rawFrontLeftSpeed;
        double normalizedBackLeftSpeed = rawBackLeftSpeed;

        double absFrontRightSpeed = Math.abs(rawFrontRightSpeed);
        double absBackRightSpeed = Math.abs(rawBackRightSpeed);
        double absFrontLeftSpeed = Math.abs(rawFrontLeftSpeed);
        double absBackLeftSpeed = Math.abs(rawBackLeftSpeed);

        double maxSpeed = Math.max(absFrontRightSpeed, Math.max(absBackRightSpeed, Math.max(absFrontLeftSpeed, absBackLeftSpeed)));

        if (maxSpeed > 1) {
            normalizedFrontRightSpeed /= maxSpeed;
            normalizedBackRightSpeed /= maxSpeed;
            normalizedFrontLeftSpeed /= maxSpeed;
            normalizedBackLeftSpeed /= maxSpeed;
        }

        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_RIGHT, normalizedFrontRightSpeed);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_RIGHT, normalizedBackRightSpeed);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_LEFT, normalizedFrontLeftSpeed);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_LEFT, normalizedBackLeftSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_RIGHT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_RIGHT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.FRONT_LEFT, 0);
        this.subsystem.moveMotor(MecanumDriveSubsystem.MotorNames.BACK_LEFT, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.pController.atSetPoint();
    }
}
