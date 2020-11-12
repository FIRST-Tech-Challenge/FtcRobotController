package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
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
        List<File> logFiles = new ArrayList<>();
        buildLogList(logFiles, ROAD_RUNNER_FOLDER);
        Collections.sort(logFiles, new Comparator<File>() {
            @Override
            public int compare(File lhs, File rhs) {
                return Long.compare(lhs.lastModified(), rhs.lastModified());
            }
        });

        long dirSize = 0;
        for (File file: logFiles) {
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
    public static File getLogFile(String name) {
        //noinspection ResultOfMethodCallIgnored
        ROAD_RUNNER_FOLDER.mkdirs();

        pruneLogsIfNecessary();

        return new File(ROAD_RUNNER_FOLDER, name);
    }
}
