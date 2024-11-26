package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Manages graceful shutdown of robot systems.
 * Monitors OpMode status and ensures proper cleanup of resources.
 */
public class ShutdownManager {
    private final LinearOpMode linearOpMode;
    private final BaseRobot baseRobot;
    private final Timer timer;

    /**
     * Creates a new shutdown manager
     * 
     * @param linearOpMode Current OpMode instance
     * @param baseRobot    Reference to main robot instance
     */
    public ShutdownManager(LinearOpMode linearOpMode, BaseRobot baseRobot) {
        this.linearOpMode = linearOpMode;
        this.baseRobot = baseRobot;
        this.timer = new Timer();
    }

    /**
     * Schedules periodic checks for OpMode status
     * Initiates shutdown sequence if OpMode becomes inactive
     */
    public void scheduleShutdownCheck() {
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                if (!linearOpMode.opModeIsActive()) {
                    // OpMode is no longer active, initiate shutdown
                    shutdown();
                }
            }
        }, 0, 500); // Schedule the check every 500ms
    }

    private void shutdown() {
        // Perform other cleanup or shutdown actions
        baseRobot.shutDown();
        // Cancel the timer to stop further periodic checks
        timer.cancel();

        // force the system to shut down
        throw new RuntimeException("Time to shut down!");
    }
}
