package org.firstinspires.ftc.teamcode.rework.ModuleTools;

import android.os.SystemClock;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rework.Robot;

/**
 * ModuleExecutor creates a new thread where modules will be executed and data will be retrieved
 * from the hubs.
 */
public class ModuleExecutor extends Thread {
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/rework/ModuleTools/ModuleExecutor.java

    final boolean SHOW_UPDATE_SPEED = true;
=======
    /**
     * Whether or not to telemetry data on update speed, for debugging.
     */
    final static boolean SHOW_UPDATE_SPEED = true;
>>>>>>> 661b8a8450127843346bf11f914073b604a851b6:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/rework/Robot/Modules/ModuleExecutor.java

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
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/rework/ModuleTools/ModuleExecutor.java
            robot.update();
=======
            robot.getBulkData();

            robot.updateModules();
>>>>>>> 661b8a8450127843346bf11f914073b604a851b6:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/rework/Robot/Modules/ModuleExecutor.java

            if (SHOW_UPDATE_SPEED) {
                currentTime = SystemClock.elapsedRealtime();
                robot.telemetryDump.addHeader("---ModuleExector Update Speed---");
                robot.telemetryDump.addData("Module Executor thread loop time: ", (currentTime - lastUpdateTime));
                lastUpdateTime = currentTime;
            }
            robot.telemetryDump.update();
        }
        System.out.println("Module executor thread exited due to opMode no longer being active.");
    }
}
