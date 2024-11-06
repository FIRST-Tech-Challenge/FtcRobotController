package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.MainAuto;

import java.util.Timer;
import java.util.TimerTask;

/** @noinspection FieldCanBeLocal, unused */
public class ShutdownManager {
    private final LinearOpMode linearOpMode;
    private final BaseRobot baseRobot;
    private final MainAuto mainAuto;

    private final Timer timer;

    public ShutdownManager(LinearOpMode linearOpMode, BaseRobot baseRobot, MainAuto mainAuto) {
        this.linearOpMode = linearOpMode;
        this.baseRobot = baseRobot;
        this.mainAuto = mainAuto;
        this.timer = new Timer();
    }

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

        // force the autonomous to shut down
        throw new RuntimeException("Time to shut down autonomous!");
    }
}
