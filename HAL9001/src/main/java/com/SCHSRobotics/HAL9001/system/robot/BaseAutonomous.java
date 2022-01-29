package com.SCHSRobotics.HAL9001.system.robot;

/**
 * An abstract class used to more easily create opmodes.
 * <p>
 * Creation Date: 2017
 *
 * @author Andrew Liang, Level Up
 * @version 1.0.0
 * @see HALProgram
 * @see BaseTeleop
 * @see Robot
 * @since 0.0.0
 */
public abstract class BaseAutonomous extends HALProgram {

    /**
     * A method that contains the code for the robot to run.
     */
    public abstract void main();

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
            main();

            onStop();
            robot.stopAllComponents();

        }
        catch (Throwable ex) {
            errorLoop(ex);
        }
    }
}