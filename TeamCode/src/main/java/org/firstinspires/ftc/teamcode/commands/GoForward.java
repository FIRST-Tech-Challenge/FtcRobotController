package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * A complex auto command that drives forward, releases a stone, and then drives backward.
 */
public class GoForward extends SequentialCommandGroup {

    private static final double INCHES = 3.0;
    private static final double SPEED = 0.5;

    /**
     * Creates a new ReleaseAndBack command group.
     *
     * @param drive The drive subsystem this command will run on
     */
    public GoForward(DriveSubsystem drive) {
        addCommands(
                new DriveDistance(INCHES, SPEED, drive)
        );
    }

}
