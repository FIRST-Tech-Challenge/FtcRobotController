package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

/**
 * The drivebase's default command: reads the sticks every loop and drives field-centric.
 *
 * This never finishes, so it runs the whole match unless something else claims the drivebase.
 */
public class DriveCommand extends CommandBase {

    private final DrivebaseSubsystem drivebase;
    private final GamepadEx driver;

    public DriveCommand(DrivebaseSubsystem drivebase, GamepadEx driver) {
        this.drivebase = drivebase;
        this.driver = driver;
        addRequirements(drivebase);
    }

    @Override
    public void execute() {
        // GamepadEx.getLeftY() already flips the stick, so pushing forward gives a positive number.
        double forward = shape(driver.getLeftY());
        double right   = shape(driver.getLeftX());
        double rotate  = shape(driver.getRightX()) * RobotConstants.TURN_SCALE;

        drivebase.driveFieldCentric(forward, right, rotate);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Removes the deadband and squares the stick input. Squaring keeps full power available at the
     * end of the stick's travel while making small movements much gentler.
     */
    private double shape(double input) {
        double magnitude = Math.abs(input);
        if (magnitude < RobotConstants.DEADBAND) {
            return 0.0;
        }
        // Rescale so the robot starts moving right at the edge of the deadband rather than jumping.
        double scaled = (magnitude - RobotConstants.DEADBAND) / (1.0 - RobotConstants.DEADBAND);
        return Math.signum(input) * scaled * scaled;
    }
}
