package org.firstinspires.ftc.teamcode.robots.reachRefactor.vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.TelemetryProvider;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public abstract class VisionProvider implements TelemetryProvider {

    private Map<Position, Integer> positionFrequencies;
    private Position mostFrequentPosition;

    public VisionProvider() {
        mostFrequentPosition = Position.HOLD;

        positionFrequencies = new HashMap<>();
        positionFrequencies.put(Position.LEFT, 0);
        positionFrequencies.put(Position.MIDDLE, 0);
        positionFrequencies.put(Position.RIGHT, 0);
    }

    abstract public void initializeVision(HardwareMap hardwareMap);

    abstract public void shutdownVision();

    abstract public Position getPosition();

    abstract public void reset();

    private void updateMostFrequentPosition() {
        int mostFrequentPositionCount = -1;
        for(Map.Entry<Position, Integer> entry: positionFrequencies.entrySet()) {
            Position position = entry.getKey();
            if(position != Position.HOLD && position != Position.NONE_FOUND) {
                int positionFrequency = entry.getValue();
                if (positionFrequency > mostFrequentPositionCount) {
                    mostFrequentPositionCount = positionFrequency;
                    mostFrequentPosition = position;
                }
            }
        }
        if(mostFrequentPositionCount == 0)
            mostFrequentPosition = Position.NONE_FOUND;
    }

    abstract protected void updateVision();

    public void update() {
        updateVision();

        Position position = getPosition();
        if(position != Position.HOLD && position != Position.NONE_FOUND)
            positionFrequencies.put(position, positionFrequencies.get(position) + 1);
        updateMostFrequentPosition();
    }

    public Position getMostFrequentPosition() {
        return mostFrequentPosition;
    }

    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();

        telemetryMap.put("Most Frequent Detected Position", mostFrequentPosition);
        telemetryMap.put("Detected Position", getPosition());

        return telemetryMap;
    }
}
