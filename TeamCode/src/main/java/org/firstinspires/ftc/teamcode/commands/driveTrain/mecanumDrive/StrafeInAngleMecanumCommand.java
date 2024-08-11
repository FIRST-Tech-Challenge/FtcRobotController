package org.firstinspires.ftc.teamcode.commands.driveTrain.mecanumDrive;

import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleCommandBase;

public class StrafeInAngleMecanumCommand extends SympleCommandBase<MecanumDriveSubsystem> {
    private final double angle;
    private final double meters;

    private double STARTING_FORWARD_DIST = 0;
    private double STARTING_SIDE_DIST = 0;
    private PController pController;

    public StrafeInAngleMecanumCommand(MecanumDriveSubsystem subsystem, double angle, double meters) {
        super(subsystem);
        this.angle = angle;
        this.meters = meters;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.STARTING_FORWARD_DIST = this.subsystem.getForwardDistanceDriven();
        this.STARTING_SIDE_DIST = this.subsystem.getSideDistanceDriven();

        this.pController = new PController(0.5);
        this.pController.setTolerance(0.02);
        this.pController.setSetPoint(this.meters);
    }

    @Override
    public void execute() {
        double hSpeed = Math.cos(angle) * meters;
        double vSpeed = Math.sin(angle) * meters;


        double forwardDistanceMoved = this.subsystem.getForwardDistanceDriven() - this.STARTING_FORWARD_DIST;
        double sideDistanceMoved = this.subsystem.getSideDistanceDriven() - STARTING_SIDE_DIST;

        double currentDist = Math.sqrt(Math.pow(forwardDistanceMoved, 2) + Math.pow(sideDistanceMoved, 2));
        double allPower = this.pController.calculate(currentDist);

        double rawFrontRightSpeed = (vSpeed + hSpeed) * allPower;
        double rawBackRightSpeed = (vSpeed - hSpeed) * allPower;
        double rawFrontLeftSpeed = (vSpeed - hSpeed) * allPower;
        double rawBackLeftSpeed = (vSpeed + hSpeed) * allPower;

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
