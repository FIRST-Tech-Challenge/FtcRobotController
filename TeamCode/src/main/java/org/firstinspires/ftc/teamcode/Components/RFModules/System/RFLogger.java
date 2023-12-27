package org.firstinspires.ftc.teamcode.Components.RFModules.System;

import android.annotation.SuppressLint;
import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.FileHandler;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.LogRecord;
import java.util.logging.SimpleFormatter;

// default verbosity = INFO
// maximum verbosity = FINEST
// global verbosity level
// print name of program at top of logfile
// in competition opmodes = only info verbosity

public class
RFLogger {
    public Logger LOGGER;
    ArrayList<FileHandler> handlerList = new ArrayList<>();
    Severity logLevel = Severity.ALL;
    static FileHandler GeneralFH, AutonomousFH, HardwareFH, QueuerFH;
    public static Severity FILTER = Severity.INFO;

    private boolean logLevelSet = false;


    public enum Files {
        @SuppressLint("SdCardPath") GENERAL_LOG("/sdcard/tmp/General.log", 0),
        @SuppressLint("SdCardPath") AUTONOMOUS_LOG("/sdcard/tmp/Autonomous.log", 1),
        @SuppressLint("SdCardPath") HARDWARE_LOG("/sdcard/tmp/Hardware.log", 2),
        @SuppressLint("SdCardPath") QUEUER_LOG("/sdcard/tmp/Queuer.log", 3);

        final String filePath;
        final int index;

        Files(String p_filePath, int p_index) {
            filePath = p_filePath;
            index = p_index;
        }
    }

    public enum Severity {
        ALL(Level.SEVERE),
        SEVERE(Level.SEVERE),
        WARNING(Level.WARNING),
        INFO(Level.INFO),
        CONFIG(Level.CONFIG),
        FINE(Level.FINE),
        FINER(Level.FINER),
        FINEST(Level.FINEST);

        final Level logSeverity;

        Severity(Level p_logSeverity) {
            logSeverity = p_logSeverity;
        }
    }

    public RFLogger(String className) {
        LOGGER = Logger.getLogger(className);
        LOGGER.setLevel(logLevel.logSeverity);

        try {
            GeneralFH = new FileHandler(Files.GENERAL_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            AutonomousFH = new FileHandler(Files.AUTONOMOUS_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            HardwareFH = new FileHandler(Files.HARDWARE_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            QueuerFH = new FileHandler(Files.QUEUER_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        handlerList.add(GeneralFH);
        handlerList.add(AutonomousFH);
        handlerList.add(HardwareFH);
        handlerList.add(QueuerFH);

        SimpleFormatter customSH = new SimpleFormatter() {
            private static final String format = "[%1$tT.%1$tL] [%2$-7s] %3$s %n";

            @SuppressLint("DefaultLocale")
            public synchronized String format(LogRecord lr) {
                return String.format(format,
                        lr.getMillis(),
                        lr.getLevel().getLocalizedName(),
                        lr.getMessage()
                );
            }
        };

        GeneralFH.setFormatter(customSH);
        AutonomousFH.setFormatter(customSH);
        HardwareFH.setFormatter(customSH);
        QueuerFH.setFormatter(customSH);

        LOGGER.addHandler(GeneralFH);
    }

    public void setLogLevel(Severity p_severity) {
        logLevel = p_severity;
        LOGGER.setLevel(logLevel.logSeverity);
        logLevelSet = true;
    }

    /**
     * Logs an inputted string using to the default file(general) at the default verbosity level(info)
     * @param info what string you want to be logged
     */
    public void log(String info) {
        Severity severity = Severity.INFO;
        if (logLevelSet) {
            severity = logLevel;
        }
        log(Files.GENERAL_LOG, severity, info);
    }

    public void log(Severity p_severity, String info) {
        log(Files.GENERAL_LOG, p_severity, info);
    }

    public void log(Files p_file, String info) {
        log(p_file, Severity.INFO, info);
    }

    public void log(Files p_file, Severity p_Severity, String info) {
        setLogLevel(p_Severity);
        if (logLevel.logSeverity.intValue() >= FILTER.logSeverity.intValue()) {
            for (Handler i : LOGGER.getHandlers()) {
                LOGGER.removeHandler(i);
            }
            LOGGER.addHandler(handlerList.get(p_file.index));
            StringBuilder output = new StringBuilder();
            StackTraceElement[] elements = Thread.currentThread().getStackTrace();
            boolean first = false;
            StackTraceElement firstElement = elements[0];
            for (StackTraceElement element : elements) {
                if (element.toString().startsWith("org")) {
                    output.append(".");
                    if (!first && !element.getMethodName().startsWith("log")) {
                        first = true;
                        firstElement = element;
                    }
                }
            }
            String fileNam = firstElement.getFileName();
            LOGGER.log(logLevel.logSeverity, fileNam.substring(0, fileNam.length() - 5) + "." + firstElement.getMethodName()
                    + "(): " + output + info);
        }
        logLevelSet = false;
    }

    public void setFilter(Severity p_severity) {
        FILTER = p_severity;
    }
}
