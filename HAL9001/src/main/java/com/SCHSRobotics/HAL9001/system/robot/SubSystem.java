package com.SCHSRobotics.HAL9001.system.robot;

import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.SCHSRobotics.HAL9001.util.misc.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.jetbrains.annotations.NotNull;

/**
 * An abstract class representing a subsystem on the robot.
 * <p>
 * Creation Date: 2017
 *
 * @author Andrew Liang, Level Up
 * @version 1.0.0
 * @see Robot
 * @see HALProgram
 * @see com.SCHSRobotics.HAL9001.system.config.AutonomousConfig
 * @see com.SCHSRobotics.HAL9001.system.config.TeleopConfig
 * @since 0.0.0
 */
public abstract class SubSystem {
    //The robot the subsystem belongs to.
    protected Robot robot;
    //A boolean specifying whether or not the subsystem should use the configuration menu.
    protected boolean usesConfig;

    /**
     * Constructor for subsystem.
     *
     * @param robot The robot the subsystem is contained within.
     */
    public SubSystem(@NotNull Robot robot) {
        this.robot = robot;
        usesConfig = false;
    }

    /**
     * A method containing the code that the subsystem runs when being initialized.
     */
    public abstract void init();

    /**
     * A method that contains code that runs in a loop on init.
     */
    public abstract void init_loop();

    /**
     * A method containing the code that the subsystem runs when being start.
     */
    public abstract void start();

    /**
     * A method containing the code that the subsystem runs every loop in a teleop program.
     */
    public abstract void handle();

    /**
     * A method containing the code that the subsystem runs when the program is stopped.
     */
    public abstract void stop();

    /**
     * Waits for a specified number of milliseconds.
     *
     * @param millis - The number of milliseconds to wait.
     *
     * @see Timer
     */
    protected final void waitTime(long millis) {
        Timer timer = new Timer();
        timer.start(millis, HALTimeUnit.MILLISECONDS);
        while (robot.opModeIsActive() && !timer.requiredTimeElapsed())
            ((LinearOpMode) robot.getOpMode()).sleep(1);
    }

    /**
     * Waits for a specified number of milliseconds, running a function in a loop while its waiting.
     *
     * @param millis The number of milliseconds to wait.
     * @param runner The code to run each loop while waiting.
     *
     * @see Timer
     * @see Runnable
     */
    protected final void waitTime(long millis, @NotNull Runnable runner) {
        Timer timer = new Timer();
        timer.start(millis, HALTimeUnit.MILLISECONDS);
        while (robot.opModeIsActive() && !timer.requiredTimeElapsed()) {
            runner.run();
            ((LinearOpMode) robot.getOpMode()).sleep(1);
        }
    }

    /**
     * Waits until a condition returns true.
     *
     * @param condition The boolean condition that must be true in order for the program to stop waiting.
     *
     * @see Supplier
     */
    protected final void waitUntil(@NotNull Supplier<Boolean> condition) {
        while (robot.opModeIsActive() && !condition.get())
            ((LinearOpMode) robot.getOpMode()).sleep(1);
    }

    /**
     * Waits until a condition returns true, running a function in a loop while its waiting.
     *
     * @param condition The boolean condition that must be true in order for the program to stop waiting.
     * @param runner The code to run each loop while waiting.
     *
     * @see Supplier
     * @see Runnable
     */
    protected final void waitUntil(@NotNull Supplier<Boolean> condition, @NotNull Runnable runner) {
        while (robot.opModeIsActive() && !condition.get()) {
            runner.run();
            ((LinearOpMode) robot.getOpMode()).sleep(1);
        }
    }

    /**
     * Waits while a condition is true.
     *
     * @param condition The boolean condition that must become false for the program to stop waiting.
     *
     * @see Supplier
     */
    protected final void waitWhile(@NotNull Supplier<Boolean> condition) {
        while (robot.opModeIsActive() && condition.get())
            ((LinearOpMode) robot.getOpMode()).sleep(1);
    }

    /**
     * Waits while a condition is true, running a function in a loop while its waiting.
     *
     * @param condition The boolean condition that must become false for the program to stop waiting.
     * @param runner The code to run each loop while waiting.
     *
     * @see Supplier
     * @see Runnable
     */
    protected final void waitWhile(@NotNull Supplier<Boolean> condition, @NotNull Runnable runner) {
        while (robot.opModeIsActive() && condition.get()) {
            runner.run();
            ((LinearOpMode) robot.getOpMode()).sleep(1);
        }
    }
}