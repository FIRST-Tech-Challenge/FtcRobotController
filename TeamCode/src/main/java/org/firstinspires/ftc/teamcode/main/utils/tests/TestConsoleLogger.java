package org.firstinspires.ftc.teamcode.main.utils.tests;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Locale;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

/**
 * A TestLogger logs information from Test in a proper way. It uses a Logger under the hood. It logs information about a Tester's current status and the status of Tests, like their passes, failures, and logs.
 */
public class TestConsoleLogger {

    private final Logger LOGGER;

    public TestConsoleLogger() {
        Logger mainLogger = Logger.getLogger(TestConsoleLogger.class.getName());
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

    public void logTestResult(Grade grade, String name, String logs, long startTime, long endTime, Tester.FailureReason reason, Exception exception) {
        // build result message
        StringBuilder message = new StringBuilder();
        String separator = "-----------------------------------------------";
        message.append(name).append(System.lineSeparator()).append(separator);
        if(logs != null && logs.length() > 0) {
            // shorten lines to be 4 shorter than the separator for style
            StringBuilder finalLogs = new StringBuilder();
            StringBuilder workingLogs = new StringBuilder().append(logs);
            while(workingLogs.length() > 0) {
                int min = Math.min(workingLogs.length(), separator.length() - 5);
                finalLogs.append(workingLogs.substring(0, min)).append(System.lineSeparator());
                workingLogs.delete(min - 1, workingLogs.length());
            }
            message.append(workingLogs).append(System.lineSeparator());
        }
        message.append(separator).append("Test completed in ").append(endTime - startTime).append("ms");
        // if there was an exception, log it
        if(reason == Tester.FailureReason.EXCEPTION) {
            StringWriter string = new StringWriter();
            PrintWriter printer = new PrintWriter(string);
            exception.printStackTrace(printer);
            String stackTrace = string.toString();
            message.append(System.lineSeparator()).append(separator).append("Exception: ").append(exception.getLocalizedMessage()).append(System.lineSeparator()).append(System.lineSeparator()).append(stackTrace);
        }
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

    public void logResults(ArrayList<HashMap<String, Boolean>> results) {
        int passes = 0;
        int failures = 0;
        for(HashMap<String, Boolean> result : results) {
            HashMap.Entry<String, Boolean> entry = result.entrySet().iterator().next();
            if(entry.getValue().equals(true)) {
                LOGGER.log(Level.WARNING, entry.getKey() + " passed!");
                passes++;
            }else{
                LOGGER.log(Level.WARNING, entry.getKey() + " failed!");
                failures++;
            }
        }
        LOGGER.log(Level.WARNING, passes + " passes and " + failures + " failures.");
    }

    public void logEnd() {
        LOGGER.log(Level.WARNING, "Testing complete!");
    }

}
