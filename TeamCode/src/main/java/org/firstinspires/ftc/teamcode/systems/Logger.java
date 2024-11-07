package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/** @noinspection FieldCanBeLocal, unused */
public class Logger {
    /** Telemetry instance for data output */
    private final Telemetry telemetry;

    /** Reference to main robot instance */
    private final BaseRobot baseRobot;

    /** Queue for persistent messages that should always be displayed */
    private final Queue<String> permanentQueue;

    /** Queue for debug messages (only shown when debugging is enabled) */
    private final Queue<String> debugQueue;

    /** Queue for real-time data that updates frequently */
    private final Queue<String> liveQueue;

    /** Maps for tracking message counts and timing data */
    private final Map<String, Integer> countMap = new HashMap<>();
    private final Map<String, Long> timeMap = new HashMap<>();

    /** Storage for data that needs to be updated regularly */
    private final Map<String, String> updateableData = new HashMap<>();

    private final ScheduledFuture<?> scheduledTask;

    /**
     * Creates a new Logger instance
     * 
     * @param baseRobot Reference to main robot instance
     */
    public Logger(BaseRobot baseRobot) {
        this.telemetry = baseRobot.telemetry;
        this.baseRobot = baseRobot;

        permanentQueue = new LinkedList<>();
        debugQueue = new LinkedList<>();
        liveQueue = new LinkedList<>();

        // Initialize with permanent section
        telemetry.log().add("-- Permanent --");

        ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
        scheduledTask = scheduler.scheduleWithFixedDelay(this::periodic, 0, 800, TimeUnit.MILLISECONDS);
    }

    public void stop() {
        scheduledTask.cancel(true);
    }

    private void periodic() {
        // Add permanent lines
        processQueue(permanentQueue, LogType.PERMANENT);

        // Debug information
        if (Settings.Deploy.DEBUG) {
            telemetry.addLine("-- Debug --");
            if (!Settings.getDisabledFlags().isEmpty()) {
                telemetry.addLine("DISABLED: " + Settings.getDisabledFlags());
            }
            processQueue(debugQueue, LogType.DEBUG);
        }

        // Update mutable data
        for (Map.Entry<String, String> entry : updateableData.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }

        // Add live lines
        processQueue(liveQueue, LogType.LIVE);

        telemetry.update();
    }

    /**
     * Processes and displays queued messages based on their type
     * 
     * @param queue   Message queue to process
     * @param logType Type of messages being processed
     */
    private void processQueue(Queue<String> queue, LogType logType) {
        while (!queue.isEmpty()) {
            switch (logType) {
                case PERMANENT:
                    telemetry.log().add(queue.poll());
                    break;
                case DEBUG:
                case LIVE:
                default:
                    telemetry.addLine(queue.poll());
            }
        }
    }

    /**
     * Adds a message to the specified logging queue
     * 
     * @param line    Message to add
     * @param logType Queue to add the message to (PERMANENT, DEBUG, or LIVE)
     */
    public void add(String line, LogType logType) {
        switch (logType) {
            case PERMANENT:
                permanentQueue.add(line);
                break;
            case DEBUG:
                debugQueue.add(line);
                break;
            case LIVE:
            default:
                liveQueue.add(line);
        }
    }

    /**
     * Increments the count for a given name and updates the telemetry
     * 
     * @param name Name of the data to count
     */
    public void count(String name) {
        countMap.put(name, countMap.getOrDefault(name, 0) + 1);
        update(name + "occurrences: ", countMap.get(name).toString());
    }

    /**
     * Measures the elapsed time since the last measurement and updates the
     * telemetry
     * 
     * @param name Name of the data to measure
     */
    public void time(String name) {
        long currentTime = System.nanoTime();
        // noinspection DataFlowIssue
        long startTime = timeMap.getOrDefault(name, currentTime);

        long elapsedTime = currentTime - startTime;

        update(name + " took", elapsedTime / 1_000_000.0 + " milliseconds");

        // Reset the start time for the next measurement
        timeMap.put(name, currentTime);
    }

    /**
     * Updates the telemetry with a new data entry
     * 
     * @param caption Caption of the data
     * @param data    Data to add
     */
    public void update(String caption, String data) {
        updateableData.put(caption, data);
    }

    public enum LogType {
        PERMANENT,
        DEBUG,
        LIVE,
    }
}
