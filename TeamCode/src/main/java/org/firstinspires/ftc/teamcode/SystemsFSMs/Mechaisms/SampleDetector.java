package org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.GobildaBlindToucherV69;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.ColorUtils;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;

import java.util.Arrays;

public class SampleDetector {

    private Logger logger;

    private double distance;
    private double r, g, b, a;
    private double h, s, v;

    private int bufferCounter = 0;
    private double[] buffer = new double[5];

    private GobildaBlindToucherV69 colorSensor;

    public enum Status {
        sampleDetected,
        noSampleDetected;
    }

    public enum SampleColor {
        yellow,
        blue,
        red,
        unknown;
    }

    private Status status;
    private SampleColor color;

    public SampleDetector (Hardware hardware, Logger logger) {
        colorSensor = hardware.intakeCS;
        this.logger = logger;
    }

    public void update() {
        distance  = colorSensor.getDistance(DistanceUnit.MM);
        buffer[bufferCounter % 5] = distance;
        bufferCounter += 1;

        findStatus();
    }

    public void log() {
        logger.log("<b>" + "Sample Detector" + "</b>", "", Logger.LogLevels.production);

        logger.log("Status", status, Logger.LogLevels.debug);
        logger.log("AO5 Distance", aO5(), Logger.LogLevels.debug);
        logger.log("Color", color, Logger.LogLevels.debug);
        logger.log("Distance", distance, Logger.LogLevels.developer);
        logger.log("Distance Buffer", Arrays.toString(buffer), Logger.LogLevels.developer);

        logger.log("r", r, Logger.LogLevels.developer);
        logger.log("g", g, Logger.LogLevels.developer);
        logger.log("b", b, Logger.LogLevels.developer);
        logger.log("a", a, Logger.LogLevels.developer);

        logger.log("h", h, Logger.LogLevels.developer);
        logger.log("s", s, Logger.LogLevels.developer);
        logger.log("v", v, Logger.LogLevels.developer);

    }

    public Status getStatus(){
        return status;
    }

    public SampleColor getSampleColor(){
        return color;
    }

    private void findStatus() {

        if (sampleDetected()) {
            status = Status.sampleDetected;

            detectColor();

        } else {
            status = Status.noSampleDetected;

            color = SampleColor.unknown;
        }
    }

    private boolean sampleDetected() {
        return (aO5() <= IntakeConstants.detectionDistance);
    }

    private double aO5() {
        double total = 0.00;
        double items = 0;

        for (double distance : buffer) {
            if (distance != 0) {
                total += distance;
                items += 1;
            }
        }

        items = items == 0 ? 1 : items;

        return total / items;
    }

    private void detectColor()  {

        colorSensor.updateColors();

        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();
        a = colorSensor.alpha();

        double r1 = ColorUtils.Clamp(r,0 ,IntakeConstants.maxR);
        double g1 = ColorUtils.Clamp(g,0 ,IntakeConstants.maxG);
        double b1 = ColorUtils.Clamp(b,0 ,IntakeConstants.maxB);
        double a1 = ColorUtils.Clamp(a,0 ,IntakeConstants.maxA);

        r1 = ColorUtils.Normalize(r1, 255, IntakeConstants.maxR);
        g1 = ColorUtils.Normalize(g1, 255, IntakeConstants.maxG);
        b1 = ColorUtils.Normalize(b1, 255, IntakeConstants.maxB);
        a1 = ColorUtils.Normalize(a1, 1, IntakeConstants.maxA);

        double[] hsv = ColorUtils.RGBAtoHSV(r1, g1, b1, a1);

        h = hsv[0];
        s = hsv[1];
        v = hsv[2];

        color = ColorUtils.ClassifyColor(hsv);
    }

}


