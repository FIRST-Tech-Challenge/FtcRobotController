package com.hfrobots.tnt.corelib.util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Utility functions for log files.
 */
public class Logging {
    public static final File TNT_LOG_FOLDER =
            new File(AppUtil.ROOT_FOLDER + "/TNT/");

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
        long totalSpace = TNT_LOG_FOLDER.getTotalSpace();
        List<File> logFiles = null;
        while (totalSpace > LOG_QUOTA) {
            if (logFiles == null) {
                logFiles = new ArrayList<>();
                buildLogList(logFiles, TNT_LOG_FOLDER);
                Collections.sort(logFiles, new Comparator<File>() {
                            @Override
                            public int compare(File lhs, File rhs) {
                                return Long.compare(lhs.lastModified(), rhs.lastModified());
                            }
                        });
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
        TNT_LOG_FOLDER.mkdirs();

        pruneLogsIfNecessary();

        return new File(TNT_LOG_FOLDER, name);
    }
}
