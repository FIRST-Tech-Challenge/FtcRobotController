package org.firstinspires.ftc.teamcode.main.utils.tests;

import java.util.Date;
import java.util.Locale;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

/**
 * A TestLogger logs information from Test in a proper way. It uses a Logger under the hood. It logs information about a Tester's current status and the status of Tests, like their passes, failures, and logs.
 */
public class TestLogger {

    private Logger LOGGER;

    public TestLogger() {
        Logger mainLogger = Logger.getLogger(TestLogger.class.getName());
        // setup custom logger handler
        mainLogger.setUseParentHandlers(false);
        ConsoleHandler handler = new ConsoleHandler();
        handler.setFormatter(new SimpleFormatter() {

            private static final String format = "%1$s %2$s %3$s %n";

            @Override
            public synchronized String format(LogRecord lr) {
                // determine grade based on log level, and format message accordingly. warnings are currently being used to identify misc. information, but anything besides info and severe levels can be used
                String grade;
                if(lr.getLevel().equals(Level.INFO)) {
                    grade = "\u001b[38;5;10m ✔ PASSING - ";
                }else if(lr.getLevel().equals(Level.SEVERE)) {
                    grade = "\u001b[38;5;9m ✘ FAILING - ";
                }else{
                    grade = "\u001b[38;5;33m \uD83D\uDEC8 INFO - ";
                }
                return String.format(Locale.ROOT, format, grade, lr.getMessage(), "\u001B[0m");
            }

        });
        mainLogger.addHandler(handler);
        LOGGER = mainLogger;
    }

    public enum Grade {
        PASSING,
        FAILING
    }

    public void logTestResult(Grade grade, String name, String logs, long startTime, long endTime) {
        // build result message
        StringBuilder message = new StringBuilder();
        String separator = "-----------------------------------------------";
        message.append(name).append(System.lineSeparator()).append(separator);
        if(logs != null && logs.length() > 0) {
            // shorten lines to be 4 shorter than the separator for style
            StringBuilder finalLogs = new StringBuilder();
            String workingLogs = logs;
            while(workingLogs.length() > 0) {
                int min = Math.min(workingLogs.length(), separator.length() - 5);
                finalLogs.append(workingLogs.substring(0, min)).append(System.lineSeparator());
                workingLogs = workingLogs.substring(0, min);
            }
        }
        message.append(separator).append("Test completed in ").append(endTime - startTime).append("ms");
        // log with level based on grade
        if(grade == Grade.PASSING) {
            LOGGER.log(Level.INFO, message.toString());
        }else{
            LOGGER.log(Level.SEVERE, message.toString());
        }
    }

    public void logSpecificRun(String name) {
        LOGGER.log(Level.WARNING, "Running " + name + "...");
    }

    public void logStart() {
        LOGGER.log(Level.WARNING, "Testing will start momentarily.");
    }

    public void logSearch() {
        LOGGER.log(Level.WARNING, "Searching for Tests...");
    }

    public void logFind() {
        LOGGER.log(Level.WARNING, "Found Tests. Running now...");
    }

    public void logFindErr() {
        LOGGER.log(Level.WARNING, "Test Arraylist format error; Arraylist must only have two elements. This run will be stopped.");
    }

    public void logUnitRun() {
        LOGGER.log(Level.WARNING, "Running Unit Tests...");
    }

    public void logIntegrationRun() {
        LOGGER.log(Level.WARNING, "Running Integration Tests...");
    }

    public void logEnd() {
        LOGGER.log(Level.WARNING, "Testing complete!");
    }

}
