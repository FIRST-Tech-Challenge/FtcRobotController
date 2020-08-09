package org.firstinspires.ftc.teamcode.rework.Modules;

import org.firstinspires.ftc.teamcode.rework.ReworkRobot;

/**
 * ModuleExecutor creates a new thread where modules will be executed and data will be retrieved
 * from the hubs.
 */
public class ModuleExecutor extends Thread {
    ReworkRobot robot;

    public ModuleExecutor(ReworkRobot robot) {
        this.robot = robot;
    }

    /**
     * Gets all modules from robot, then runs update on them.
     */
    public void run() {
        while (robot.isOpModeActive()) {
            robot.updateModules();
            robot.getBulkData();

            // TODO: Get data
        }
        System.out.println("Module executor thread exited due to opMode no longer being active.");
    }
}
