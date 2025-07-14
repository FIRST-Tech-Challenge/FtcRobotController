package ftc_library.autonomous;

import ftc_library.hardware.RobotHardware;

/**
 * AutoFarRoutine is a sample autonomous routine that moves the robot sideways for 7 seconds.
 * Extend or modify this class for your own autonomous strategies.
 */
public class AutoFarRoutine extends BaseAutonomous {

    public AutoFarRoutine(RobotHardware hardware) {
        super(hardware);
    }

    /**
     * Runs the AutoFar routine.
     * Moves the robot to the side for 7 seconds at 50% power.
     */
    public void run() throws InterruptedException {
        stopMoving(); // Make sure robot is stopped before starting
        moveXForTime(0.5, 7000); // Move sideways at 50% power for 7 seconds
        // Add more actions here for your full autonomous!
    }
}
