package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Utility functions for log files.
 */
public class LoggingUtil {
    private static final long LOG_QUOTA = 25 * 1024 * 1024; // 25MB log quota for now
    public static String ROOT_FOLDER = AppUtil.ROOT_FOLDER + "/RoadRunner/";
    public static final File ROAD_RUNNER_FOLDER =
            new File(ROOT_FOLDER + "/RoadRunner/");

    private static void buildLogList(List<File> logFiles, @NotNull File dir) {
        for (File file : dir.listFiles()) {
            if (file.isDirectory()) {
                buildLogList(logFiles, file);
            } else {
                logFiles.add(file);
            }
        }
    }

    private static void pruneLogsIfNecessary() {
        List<File> logFiles = new ArrayList<>();
        buildLogList(logFiles, ROAD_RUNNER_FOLDER);
        Collections.sort(logFiles, (lhs, rhs) ->
                Long.compare(lhs.lastModified(), rhs.lastModified()));

        long dirSize = 0;
        for (File file : logFiles) {
            dirSize += file.length();
        }

        while (dirSize > LOG_QUOTA) {
            if (logFiles.size() == 0) break;
            File fileToRemove = logFiles.remove(0);
            dirSize -= fileToRemove.length();
            //noinspection ResultOfMethodCallIgnored
            fileToRemove.delete();
        }
    }

    /**
     * Obtain a log file with the provided name
     */
    @Contract("_ -> new")
    public static @NotNull File getLogFile(String name) {
        //noinspection ResultOfMethodCallIgnored
        ROAD_RUNNER_FOLDER.mkdirs();

        pruneLogsIfNecessary();

        return new File(ROAD_RUNNER_FOLDER, name);
    }
}
