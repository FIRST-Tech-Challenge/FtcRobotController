package org.firstinspires.ftc.teamcode.main.utils.interactions.groups;

import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;

/**
 * A StandardDrivetrain represents the motors which control movement of the robot.
 */
public abstract class StandardDrivetrain extends InteractionSurface {
    public abstract void stop();
    public abstract void brake();
    public abstract void reset();
}
