package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.XDrive;

/**
 * The X-drive drivetrain, wrapped as a command-based subsystem.
 *
 * All of the real work - the X-drive wheel mixing, power normalization and IMU handling - lives in
 * {@link XDrive}. This class only exists to let the scheduler own the drivetrain so it can run a
 * default command and resolve conflicts between commands that want to drive.
 */
public class DrivebaseSubsystem extends SubsystemBase {

    private final XDrive drive;

    public DrivebaseSubsystem(HardwareMap hardwareMap) {
        drive = new XDrive(hardwareMap, RobotConstants.LOGO_DIRECTION, RobotConstants.USB_DIRECTION);
    }

    /** Drives from the driver's point of view, using the heading captured at start. */
    public void driveFieldCentric(double forward, double right, double rotate) {
        drive.driveFieldCentric(forward, right, rotate);
    }

    /** Drives from the robot's point of view. Not currently bound to anything. */
    public void driveRobotCentric(double forward, double right, double rotate) {
        drive.driveRobotCentric(forward, right, rotate);
    }

    public void stop() {
        drive.stop();
    }

    /** Sets the current heading as "away from the driver" for field-centric drive. */
    public void resetYaw() {
        drive.resetYaw();
    }

    public double getHeading(AngleUnit unit) {
        return drive.getHeading(unit);
    }

    /** @param scale 0.0 to 1.0 - multiplies every wheel power. */
    public void setSpeedScale(double scale) {
        drive.setSpeedScale(scale);
    }
}
