package org.firstinspires.ftc.Team19567.util;

/**
 * Enum describing all of the possible states the robot can be in during an autonomous program. <br>
 * Actually useful thing, more on enums <a href="https://docs.oracle.com/javase/tutorial/java/javaOO/enum.html">here</a>. <br>
 * The enums are somewhat listed in order of their execution in an auto.
 */
public enum AUTO_STATE {
    /**
     * Detecting the TSE's level
     */
    DETECTING_OPENCV,
    /**
     * Moving to the alliance hub to deliver the preload
     */
    MOVING_TO_HUB,
    /**
     * Delivering freight to the alliance hub after reaching it
     */
    DELIVERING_FREIGHT,
    /**
     * Moving to the warehouse to intake freight
     */
    MOVING_TO_WAREHOUSE,
    /**
     * Moving to the carousel to spin it
     */
    MOVING_TO_CAROUSEL,
    /**
     * Intaking freight in the warehouse
     */
    INTAKING_FREIGHT,
    /**
     * Returning to the alliance hub to deliver freight after intaking
     */
    RETURNING_TO_HUB,
    /**
     * Rotating the carousel
     */
    ROTATING_CAROUSEL,
    /**
     * Moving to the storage unit to park in it
     */
    PARKING_UNIT,
    /**
     * Moving to the warehouse to park in it
     */
    PARING_WAREHOUSE,
    /**
     * State representing the end of an auto
     */
    PATH_FINISHED
}