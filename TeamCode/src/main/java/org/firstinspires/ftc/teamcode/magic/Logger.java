package org.firstinspires.ftc.teamcode.magic;

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

public class Logger {
    private final Telemetry telemetry;
    private final BaseRobot baseRobot;

    private final Queue<String> permanentQueue;
    private final Queue<String> debugQueue;
    private final Queue<String> liveQueue;

    private final Map<String, Integer> countMap = new HashMap<>();
    private final Map<String, Long> timeMap = new HashMap<>();
    private final Map<String, String> updateableData = new HashMap<>();
    private final ScheduledFuture<?> scheduledTask;

    public Logger(BaseRobot baseRobot) {
        this.telemetry = baseRobot.telemetry;
        this.baseRobot = baseRobot;

        permanentQueue = new LinkedList<>();
        debugQueue = new LinkedList<>();
        liveQueue = new LinkedList<>();

        // Initialize with permanent section
        telemetry.log().add("-- Permanent --");

        ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
        scheduledTask = scheduler.scheduleAtFixedRate(this::periodic, 0, 800, TimeUnit.MILLISECONDS);
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

    public void add(String line) {
        add(line, LogType.LIVE);
    }

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
     * @noinspection DataFlowIssue
     */
    public void count(String name) {
        countMap.put(name, countMap.getOrDefault(name, 0) + 1);
        update(name + "occurrences: ", countMap.get(name).toString());
    }

    public void time(String name) {
        long currentTime = System.nanoTime();
        //noinspection DataFlowIssue
        long startTime = timeMap.getOrDefault(name, currentTime);

        long elapsedTime = currentTime - startTime;

        update(name + " took", elapsedTime / 1_000_000.0 + " milliseconds");

        // Reset the start time for the next measurement
        timeMap.put(name, currentTime);
    }

    public void update(String caption, String data) {
        updateableData.put(caption, data);
    }

    public enum LogType {
        PERMANENT,
        DEBUG,
        LIVE,
    }
}
