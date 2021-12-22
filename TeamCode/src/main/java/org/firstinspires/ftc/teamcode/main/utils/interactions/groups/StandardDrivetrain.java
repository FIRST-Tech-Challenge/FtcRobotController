package org.firstinspires.ftc.teamcode.main.utils.interactions.groups;

/**
 * A StandardDrivetrain represents the motors which control movement of the robot.
 */
public abstract class StandardDrivetrain extends InteractionGroup {

    public abstract void stop();

    public abstract void brake();

    public abstract void reset();

}
