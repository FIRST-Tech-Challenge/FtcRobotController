package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands.mecanumDrive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.drivetrain.MecanumChassisUtils;

@Config
public class StrafeInAngleMecanumCommand extends CommandBase {
    public static double Kp = 1;
    public static double Kd = 0.3;
    public static double Kf = 0.65;
    public static double Ki = 0;

    public static double rotationKp = 0.02;

    private double angle;
    private final double meters;
    private double invert;

    private double STARTING_FORWARD_DIST = 0;
    private double STARTING_SIDE_DIST = 0;
    private PIDFController pidfController;

    private PIDController rotationController;

    private final MecanumDriveSubsystem subsystem;

    private static double angleFix = 0;

    public static void updateAngle(double angle) {
        angleFix = angle;
    }

    public StrafeInAngleMecanumCommand(MecanumDriveSubsystem subsystem, double angle, double meters) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        this.angle = angle;
        this.meters = meters;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.invert = Math.signum(meters);
        this.angle -= 90 - angleFix;

        this.STARTING_FORWARD_DIST = this.subsystem.getForwardDistanceDriven();
        this.STARTING_SIDE_DIST = this.subsystem.getSideDistanceDriven();

        this.pidfController = new PIDFController(Kp, Ki, Kd, Kf);
        this.pidfController.setTolerance(0.0185);
        this.pidfController.setSetPoint(this.meters);

        this.rotationController = new PIDController(rotationKp, 0, 0);
        this.rotationController.setSetPoint(this.subsystem.getHeading());
        this.rotationController.setTolerance(0);
    }

    @Override
    public void execute() {
        double hSpeed = Math.cos(Math.toRadians(angle));
        double vSpeed = Math.sin(Math.toRadians(angle));
        Vector2d vector2d = new Vector2d(hSpeed, vSpeed);

        double rotationSpeed = this.rotationController.calculate(this.subsystem.getHeading());

        double forwardDistanceMoved = this.subsystem.getForwardDistanceDriven() - this.STARTING_FORWARD_DIST;
        double sideDistanceMoved = this.subsystem.getSideDistanceDriven() - this.STARTING_SIDE_DIST;

        double currentDist = invert * Math.hypot(forwardDistanceMoved, sideDistanceMoved); // => √x*x + y*y
        double powerMultiplier = this.pidfController.calculate(currentDist);

        MecanumChassisUtils.MecanumWheelSpeeds mecanumWheelSpeeds = MecanumChassisUtils.chassisSpeedToWheelSpeeds(vector2d, rotationSpeed)
                .mul(powerMultiplier);

        this.subsystem.moveMotors(mecanumWheelSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.setAllChassisPower(0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.pidfController.atSetPoint();
    }
}
