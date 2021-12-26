package org.firstinspires.ftc.teamcode.main.utils.tests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.utils.locations.HandDistanceSensorLocation;

import java.util.ArrayList;
import java.util.HashMap;

public class TestTelemetryLogger {

    private final Telemetry TELEMETRY;

    public TestTelemetryLogger(Telemetry telemetry) {
        TELEMETRY = telemetry;
        TELEMETRY.setAutoClear(false);
        TELEMETRY.clear();
        TELEMETRY.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        TELEMETRY.log().clear();
    }

    public void logTestResult(Grade grade, String name, long startTime, long endTime, Tester.FailureReason reason, Exception exception) {
        // build simplified result message
        StringBuilder message = new StringBuilder();
        message.append(name).append("<br>").append("          ").append("Time: ").append(endTime - startTime).append("ms");
        if(grade == Grade.FAILING) {
            message.append("<br>").append("          ").append("Reason of Failure: ");
            if(reason == Tester.FailureReason.EXCEPTION) {
                message.append("Exception thrown");
            }else{
                message.append("Returned false");
            }
        }
        if(grade == Grade.PASSING) {
            logPass("PASSING - " + message.toString());
        }else{
            logFail("FAILING - " + message.toString());
        }
    }

    enum TestType {
        UNIT,
        INTEGRATION
    }

    public void logSpecificRun(TestType type, String name) {
        if(type == TestType.UNIT) {
            logUnitRun();
        }else{
            logIntegrationRun();
        }
        logInfo("Running" + name + "...");
    }

    public void logStart() {
        logInfo("Testing will start momentarily.");
    }

    public void logDisclaimer() {
        logInfo("More detailed logs are available via the console. If I'm not virtualized, access the console by keeping the computer plugged in to the robot or via ADB.");
    }

    public void logSearch() {
        logInfo("Searching for Tests...");
    }

    public void logFind() {
        logInfo("Found Tests. Running now...");
    }

    public void logFindErr() {
        logInfo("Test Arraylist format error; Arraylist must only have two elements. This run will be stopped.");
    }

    public void logUnitRun() {
        logInfo("Running Unit Tests...");
    }

    public void logIntegrationRun() {
        logInfo("Running Integration Tests...");
    }

    public void logResults(ArrayList<HashMap<String, Boolean>> results) {
        int passes = 0;
        int failures = 0;
        for(HashMap<String, Boolean> result : results) {
            HashMap.Entry<String, Boolean> entry = result.entrySet().iterator().next();
            if(entry.getValue().equals(true)) {
                passes++;
            }else{
                failures++;
            }
        }
        logInfo(passes + " passes and " + failures + " failures.");
    }

    public void logEnd() {
        logInfo("Testing complete!");
    }

    private void logPass(String message) {
        TELEMETRY.log().add("<font color=\"#00FF00\" face=\"monospace\">" + message + "</font>");
        TELEMETRY.update();
    }

    private void logFail(String message) {
        TELEMETRY.log().add("<font color=\"#FF0000\" face=\"monospace\">" + message + "</font>");
        TELEMETRY.update();
    }

    private void logInfo(String message) {
        TELEMETRY.log().add("<font color=\"#0087ff\" face=\"monospace\">" + message + "</font>");
        TELEMETRY.update();
    }

    public void clear() {
        TELEMETRY.clear();
        TELEMETRY.log().clear();
    }

}
