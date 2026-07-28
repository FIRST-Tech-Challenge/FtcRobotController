package org.firstinspires.ftc.teamcode.robots.teamA;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.hardware.DriveHardware;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.core.robot.Robot;

/**
 * Team A's robot composition and public drivetrain API.
 *
 * <p>OpModes call this robot's public methods instead of manipulating mechanisms directly. This
 * keeps FTC entry points focused on mapping controls, while the robot and subsystem layers retain
 * responsibility for behavior and hardware access.</p>
 */
public class TeamARobot extends Robot {
    private final RobotHardware robotHardware;
    private final DriveSubsystem driveSubsystem;
    private boolean hardwareInitialized;

    /** Creates Team A's robot with the shared baseline hardware composition. */
    public TeamARobot() {
        this(new RobotHardware());
    }

    /**
     * Creates Team A's robot with the supplied hardware composition.
     *
     * @param robotHardware the hardware wrappers owned by this robot
     */
    public TeamARobot(RobotHardware robotHardware) {
        if (robotHardware == null) {
            throw new IllegalArgumentException("Team A robot needs robot hardware.");
        }

        this.robotHardware = robotHardware;
        driveSubsystem = new DriveSubsystem(robotHardware.getDriveHardware());
        registerSubsystem(driveSubsystem);
    }

    /**
     * Initializes hardware first, then initializes every registered subsystem once.
     *
     * @param hardwareMap the FTC hardware map supplied by the OpMode
     */
    public void initialize(HardwareMap hardwareMap) {
        if (hardwareInitialized) {
            return;
        }

        robotHardware.initialize(hardwareMap);
        hardwareInitialized = true;
        super.initialize();
    }

    /** Requests mecanum drive values for the next robot update. */
    public void drive(double forward, double strafe, double rotate) {
        driveSubsystem.drive(forward, strafe, rotate);
    }

    /** Requests normal manual drivetrain behavior. */
    public void enableManualDrive() {
        driveSubsystem.enableManualDrive();
    }

    /** Requests the safe disabled drivetrain behavior. */
    public void disableDrive() {
        driveSubsystem.disableDrive();
    }

    /** Requests the current heading-hold placeholder behavior. */
    public void enableHeadingHold() {
        driveSubsystem.enableHeadingHold();
    }

    /** Returns the active drivetrain state name for telemetry. */
    public String getDriveStateName() {
        return driveSubsystem.getCurrentStateName();
    }

    public double getRequestedForward() {
        return driveSubsystem.getRequestedForward();
    }

    public double getRequestedStrafe() {
        return driveSubsystem.getRequestedStrafe();
    }

    public double getRequestedRotate() {
        return driveSubsystem.getRequestedRotate();
    }

    public double getFrontLeftMotorPower() {
        return getDriveHardware().getFrontLeftPower();
    }

    public double getFrontRightMotorPower() {
        return getDriveHardware().getFrontRightPower();
    }

    public double getRearLeftMotorPower() {
        return getDriveHardware().getRearLeftPower();
    }

    public double getRearRightMotorPower() {
        return getDriveHardware().getRearRightPower();
    }

    @Override
    protected void onStop() {
        robotHardware.stopAll();
    }

    private DriveHardware getDriveHardware() {
        return robotHardware.getDriveHardware();
    }
}
