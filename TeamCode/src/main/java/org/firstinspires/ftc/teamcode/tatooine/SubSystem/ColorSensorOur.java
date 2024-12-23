package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;

public class ColorSensorOur {

    // Constants and configuration
    private static final String SUBSYSTEM_NAME = "ColorSensorOur";
    private static final double DEFAULT_DELTA = 40;
    private static final int DEFAULT_GAIN = 51;

    // Color data
    private final double[] myCol = new double[3];
    private final double delta;

    // Color sensor hardware and state
    private final RevColorSensorV3 colorSensor;
    private final Telemetry telemetry;
    private final boolean debugMode;
    private boolean isRedAlliance;

    // Constructor with optional debug mode
    public ColorSensorOur(OpMode opMode, boolean debugMode) {
        this.delta = DEFAULT_DELTA;
        this.telemetry = opMode.telemetry;
        this.debugMode = debugMode;
        this.colorSensor = opMode.hardwareMap.get(RevColorSensorV3.class, "CS");
        this.colorSensor.setGain(DEFAULT_GAIN);
        initializeAlliance();
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Constructor initialized with debug mode", debugMode);
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Delta", delta);
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Gain", DEFAULT_GAIN);
    }

    // Overloaded constructor without debug mode
    public ColorSensorOur(OpMode opMode) {
        this(opMode, false);
    }

    // Initialize alliance color (red or blue)
    private void initializeAlliance() {
        this.isRedAlliance = CheckAlliance.isRed();
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Alliance Initialized", isRedAlliance ? "Red" : "Blue");
    }

    // Get the distance to the object in centimeters
    public double getDistance() {
        double distance = colorSensor.getDistance(DistanceUnit.CM);
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Distance (cm)", distance);
        return distance;
    }

    // Check if the detected color matches the expected color (specimen or general check)
    public boolean isRightColor(boolean isSpecimen) {
        DominantColor currentColor = detectDominantColor();
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Detected Color", currentColor);
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Alliance", isRedAlliance ? "Red" : "Blue");
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Specimen Check", isSpecimen);

        boolean result = (isRedAlliance && (isSpecimen ? currentColor == DominantColor.RED : currentColor == DominantColor.RED || currentColor == DominantColor.YELLOW)) ||
                (!isRedAlliance && (isSpecimen ? currentColor == DominantColor.BLUE : currentColor == DominantColor.BLUE || currentColor == DominantColor.YELLOW));
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Is Correct Color", result);
        return result;
    }

    // Detect the dominant color from the sensor
    public DominantColor detectDominantColor() {
        // Retrieve normalized color values and scale to 0-255
        myCol[0] = colorSensor.getNormalizedColors().red * 255;   // Red channel
        myCol[1] = colorSensor.getNormalizedColors().green * 255; // Green channel
        myCol[2] = colorSensor.getNormalizedColors().blue * 255;  // Blue channel

        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Red Channel", myCol[0]);
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Green Channel", myCol[1]);
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Blue Channel", myCol[2]);
        DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Delta Threshold", delta);

        // Determine dominant color
        if (myCol[0] > myCol[1] + delta && myCol[0] > myCol[2] + delta) {
            DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Dominant Color", "Red");
            return DominantColor.RED;
        } else if (myCol[2] > myCol[0] + delta && myCol[2] > myCol[1] + delta) {
            DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Dominant Color", "Blue");
            return DominantColor.BLUE;
        } else {
            DebugUtils.logDebug(telemetry, debugMode, SUBSYSTEM_NAME, "Dominant Color", "Yellow (Default)");
            return DominantColor.YELLOW;
        }
    }

    // Enum representing possible dominant colors
    public enum DominantColor {
        RED, YELLOW, BLUE
    }
}
