package org.firstinspires.ftc.teamcode.rework.ModuleTools;

import android.os.SystemClock;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rework.Robot;

/**
 * ModuleExecutor creates a new thread where modules will be executed and data will be retrieved
 * from the hubs.
 */
public class ModuleExecutor extends Thread {

    final boolean SHOW_UPDATE_SPEED = true;

    Robot robot;
    Telemetry telemetry;

    public ModuleExecutor(Robot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        setName("module executor");
    }

    /**
     * Gets all modules from robot, then runs update on them.
     */
    public void run() {
        long lastUpdateTime = SystemClock.elapsedRealtime();
        long currentTime;

        while (robot.isOpModeActive()) {
            robot.update();

            if (SHOW_UPDATE_SPEED) {
                currentTime = SystemClock.elapsedRealtime();
                robot.telemetryDump.addHeader("---ModuleExector Update Speed---");
                robot.telemetryDump.addData("Module Executor thread loop time: ", (currentTime - lastUpdateTime));
                lastUpdateTime = currentTime;
            }

            if(robot.isStopRequested() && robot.WILL_FILE_DUMP){
                robot.fileDump.writeFilesToDevice();
            }

            robot.telemetryDump.update();
        }
        System.out.println("Module executor thread exited due to opMode no longer being active.");
    }
}