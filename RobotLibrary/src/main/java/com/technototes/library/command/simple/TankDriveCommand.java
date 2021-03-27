package com.technototes.library.command.simple;

import com.technototes.control.gamepad.Stick;
import com.technototes.library.command.Command;
import com.technototes.library.subsystem.drivebase.TankDrivebaseSubsystem;

import java.util.function.DoubleSupplier;

/** Simple command for driving tank drivebase
 * @author Alex Stedman
 */
public class TankDriveCommand extends Command {
    private TankDrivebaseSubsystem subsystem;
    private DoubleSupplier xv, yv;

    /** Make tank drive command
     *
     * @param subsystem The {@link TankDrivebaseSubsystem} for this command
     * @param xSupplier The supplier for driving the tank side to side
     * @param ySupplier The supplier for driving the tank forward and back
     */
    public TankDriveCommand(TankDrivebaseSubsystem subsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier){
        addRequirements(subsystem);
        this.subsystem = subsystem;
        xv = xSupplier;
        yv = ySupplier;
    }

    /** Make tank drive command
     *
     * @param subsystem The {@link TankDrivebaseSubsystem} for this command
     * @param stick The {@link Stick} for this command
     */
    public TankDriveCommand(TankDrivebaseSubsystem subsystem, Stick stick){
        new TankDriveCommand(subsystem, stick.getXSupplier(), stick.getYSupplier());
    }

    /** Drive robot with {@link TankDrivebaseSubsystem}'s arcade drive
     *
     */
    @Override
    public void execute() {
        subsystem.arcadeDrive(yv.getAsDouble(), yv.getAsDouble());
    }
}
