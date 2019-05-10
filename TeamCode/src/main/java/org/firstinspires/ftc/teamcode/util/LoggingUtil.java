package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Utility functions for log files.
 */
public class LoggingUtil {
    public static final File ROAD_RUNNER_FOLDER =
            new File(AppUtil.ROOT_FOLDER + "/RoadRunner/");

    private static final long LOG_QUOTA = 25 * 1024 * 1024; // 25MB log quota for now

    private static void buildLogList(List<File> logFiles, File dir) {
        for (File file : dir.listFiles()) {
            if (file.isDirectory()) {
                buildLogList(logFiles, file);
            } else {
                logFiles.add(file);
            }
        }
    }

    private static void pruneLogsIfNecessary() {
        long totalSpace = ROAD_RUNNER_FOLDER.getTotalSpace();
        List<File> logFiles = null;
        while (totalSpace > LOG_QUOTA) {
            if (logFiles == null) {
                logFiles = new ArrayList<>();
                buildLogList(logFiles, ROAD_RUNNER_FOLDER);
                Collections.sort(logFiles, (lhs, rhs) ->
                        Long.compare(lhs.lastModified(), rhs.lastModified()));
            }

            if (logFiles.size() == 0) break;
            File fileToRemove = logFiles.remove(0);
            totalSpace -= fileToRemove.getTotalSpace();
            //noinspection ResultOfMethodCallIgnored
            fileToRemove.delete();
        }
    }

    /**
     * Obtain a log file with the provided name
     */
    public static File getLogFile(String name) {
        //noinspection ResultOfMethodCallIgnored
        ROAD_RUNNER_FOLDER.mkdirs();

        pruneLogsIfNecessary();

        return new File(ROAD_RUNNER_FOLDER, name);
    }
}
