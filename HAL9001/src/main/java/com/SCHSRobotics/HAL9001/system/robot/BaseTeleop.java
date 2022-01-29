package com.SCHSRobotics.HAL9001.system.robot;

/**
 * An abstract class used to more easily create teleop programs
 * <p>
 * Creation Date: 2017
 *
 * @author Andrew Liang, Level Up
 * @version 1.0.0
 * @see HALProgram
 * @see BaseAutonomous
 * @see Robot
 * @since 0.0.0
 */
public abstract class BaseTeleop extends HALProgram {

    /**
     * A method that is used to instantiate the robot.
     *
     * @return The robot being used in the opmode.
     */
    protected Robot buildRobot() {
        return null;
    }

    /**
     * A method that runs when the robot is started. It is not an abstract method so that it does not have to be implemented if it
     * is unneeded.
     */
    protected void onStart() {}

    /**
     * A method that runs every loop cycle. It is not an abstract method so that it does not have to be implemented if it
     * is unneeded.
     */
    protected void onUpdate() {}

    @Override
    public final void runOpMode() {
        super.runOpMode();
        Robot robot = getRobot();

        try {
            robot.init();
            onInit();

            while (!isStarted() && !isStopRequested()) {
                robot.init_loop();
                onInitLoop();
            }

            robot.onStart();
            onStart();

            while(!isStopRequested()) {
                robot.driverControlledUpdate();
                onUpdate();
            }

            onStop();
            robot.stopAllComponents();
        }
        catch (Throwable ex) {
            errorLoop(ex);
        }
    }
}