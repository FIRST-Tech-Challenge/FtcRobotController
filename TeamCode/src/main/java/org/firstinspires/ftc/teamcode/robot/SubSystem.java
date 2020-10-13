package org.firstinspires.ftc.teamcode.robot;

/**
 * A subsystem of the robot. You should extend this class with another in the season folder.
 * @Author Jaxon Brown
 */
public abstract class SubSystem {
    /**
     * The robot this subsystem is a part of.
     */
    protected Robot robot;

    /**
     * The initialization routine of this subsystem. Cache stuff from the hardware map during this routine.
     */
    public abstract void init();

    /**
     * Handle driver controlled updates. You can fetch the gamepads with {@Code robot.gamepadX}.
     */
    public abstract void handle();

    /**
     * Stop everything on the robot.
     */
    public abstract void stop();

    /**
     * Construct a subsystem with the robot it applies to.
     * @param robot
     */
    public SubSystem(Robot robot) {
        this.robot = robot;
    }
}