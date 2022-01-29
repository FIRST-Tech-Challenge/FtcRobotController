package com.SCHSRobotics.HAL9001.system.robot;

import android.util.Log;

import com.SCHSRobotics.HAL9001.util.control.AutoTransitioner;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.SCHSRobotics.HAL9001.util.misc.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Objects;

/**
 * The base class for all HAL programs.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see BaseAutonomous
 * @see BaseTeleop
 * @see Robot
 * @see LinearOpMode
 * @since 1.1.0
 */
public abstract class HALProgram extends LinearOpMode {
    //The robot running the opmode.
    private Robot robot;

    /**
     * A method that is used to instantiate the robot.
     *
     * @return The robot being used in the opmode.
     *
     * @see Robot
     */
    protected Robot buildRobot() {
        return null;
    }

    /**
     * Method that runs when the robot is initialized. It is not an abstract method so that it does not have to be implemented if it
     * is unneeded.
     */
    protected void onInit() {}

    /**
     * Method that runs in a loop after the robot is initialized. It is not an abstract method so that it does not have to be implemented if it
     * is unneeded.
     */
    protected void onInitLoop() {}

    /**
     * Method that runs when the robot is stopped. It is not an abstract method so that it does not have to be implemented if it
     * is unneeded.
     */
    protected void onStop() {}

    @Override
    public void runOpMode() {
        //Finds if the buildRobot function is present.
        boolean buildRobotPresent;
        try {
            Method m = this.getClass().getDeclaredMethod("buildRobot");
            buildRobotPresent = true;
        } catch (NoSuchMethodException e) {
            buildRobotPresent = false;
        }

        //If buildRobot is present, build the robot, otherwise, build the robot from main robot.
        try {
            if (buildRobotPresent) robot = buildRobot();
            else {
                Field[] fields = this.getClass().getDeclaredFields();
                for (Field field : fields) {
                    if (field.isAnnotationPresent(MainRobot.class) && Robot.class.isAssignableFrom(field.getType())) {
                        try {
                            robot = (Robot) field.getType().getConstructor(OpMode.class).newInstance(this);
                        } catch (NoSuchMethodException ex) {
                            throw new NoSuchMethodException("Your robot does not have a constructor only taking an opmode as input, use buildRobot instead.");
                        } catch (IllegalAccessException ex) {
                            throw new IllegalAccessException("Your robot's constructor is not public, :(");
                        }
                        try {
                            //TODO field.setAccessible(true);
                            field.set(this, robot);
                        } catch (IllegalAccessException e) {
                            throw new IllegalAccessException("Your robot isn't accessible, and so @MainRobot won't work. The program can't access it. SHARE!!!!");
                        }
                    }
                }
            }

            //Set up link to next program
            if (this.getClass().isAnnotationPresent(LinkTo.class)) {
                LinkTo link = Objects.requireNonNull(this.getClass().getAnnotation(LinkTo.class));
                if (link.auto_transition()) {
                    AutoTransitioner.transitionOnStop(this, link.destination());
                }
            }
        } catch(Throwable ex){
            errorLoop(ex);
        }
    }

    /**
     * Loops the program so that it reports the error.
     *
     * @param ex The error to report.
     */
    protected final void errorLoop(Throwable ex) {
        Log.e(this.getClass().getSimpleName(), ex.getMessage(), ex);
        while(!isStopRequested()) {
            robot.telemetry.clearAll();
            robot.telemetry.addData("Error", ex.getMessage());
            robot.telemetry.update();
        }
    }

    /**
     * Gets the robot running the program.
     *
     * @return The robot running this program.
     *
     * @see Robot
     */
    @Contract(pure = true)
    protected final Robot getRobot() {
        return robot;
    }

    /**
     * Waits for a specified number of milliseconds.
     *
     * @param millis The number of milliseconds to wait.
     *
     * @see Timer
     */
    protected final void waitTime(long millis) {
        Timer timer = new Timer();
        timer.start(millis, HALTimeUnit.MILLISECONDS);
        while (robot.opModeIsActive() && !timer.requiredTimeElapsed()) sleep(1);
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
            sleep(1);
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
        while (robot.opModeIsActive() && !condition.get()) sleep(1);
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
            sleep(1);
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
        while (robot.opModeIsActive() && condition.get()) sleep(1);
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
            sleep(1);
        }
    }

    /**
     * Waits a certain amount of time.
     *
     * @param millis The amount of time in milliseconds to wait.
     * @deprecated Renamed to waitTime
     */
    @Deprecated
    protected final void waitFor(long millis) {
        waitTime(millis);
    }
}