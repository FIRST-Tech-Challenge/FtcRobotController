package com.SCHSRobotics.HAL9001.system.robot;

import org.jetbrains.annotations.NotNull;

/**
 * An interface for writing vision-based subsystems.
 * <p>
 * Creation Date: 11/6/19
 *
 * @author Cole Savage, Level Up
 * @version 2.0.0
 * @see SubSystem
 * @see HALPipeline
 * @since 1.0.5
 */
public abstract class VisionSubSystem extends SubSystem {
    /**
     * The constructor for VisionSubSystem. Mirrors the constructor for SubSystem.
     *
     * @param robot The robot the subsystem is contained within.
     */
    public VisionSubSystem(@NotNull Robot robot) {
        super(robot);
    }

    /**
     * Gets all the HAL pipelines contained within this subsystem.
     *
     * @return All the HAL pipelines contained within this subsystem.
     */
    protected abstract HALPipeline[] getPipelines();
}