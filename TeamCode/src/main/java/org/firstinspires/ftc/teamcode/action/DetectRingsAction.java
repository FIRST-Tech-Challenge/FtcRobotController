package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

import java.util.List;

public class DetectRingsAction implements Action {

    public enum DetectRingsResult {
        NONE,
        SINGLE,
        QUAD
    }

    public DetectRingsResult detectRingsResult = DetectRingsResult.NONE;
    private double timeToDetect;
    private double endTime;

    public DetectRingsAction(double timeToDetect) {
        this.timeToDetect = timeToDetect;
    }

    @Override
    public void init(RobotHardware hardware) {
        if (hardware.tfod != null) {
            hardware.tfod.activate();
        }
        endTime = System.currentTimeMillis() + this.timeToDetect;
    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        if (hardware.tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = hardware.tfod.getRecognitions();
            if (updatedRecognitions != null) {
                hardware.telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    hardware.telemetry.addData(String.format("Label (%d)", i), recognition.getLabel());
                    if (recognition.getLabel().equals("Single")) {
                        this.detectRingsResult = DetectRingsResult.SINGLE;
                    } else if (recognition.getLabel().equals("Quad")) {
                        this.detectRingsResult = DetectRingsResult.QUAD;
                    }
                    hardware.telemetry.addData("Confidence", "%.03f", recognition.getConfidence());
                    return true;
                }
            }
        }

        return System.currentTimeMillis() >= endTime;
    }

    @Override
    public DetectRingsResult getActionResult() {
        return this.detectRingsResult;
    }

    @Override
    public Double progress() {
        return null;
    }

    @Override
    public String progressString() {
        return null;
    }
}
