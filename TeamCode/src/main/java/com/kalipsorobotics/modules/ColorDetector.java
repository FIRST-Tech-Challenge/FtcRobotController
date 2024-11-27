package com.kalipsorobotics.modules;

import android.util.Log;

import com.kalipsorobotics.utilities.KColor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class ColorDetector {

    OpModeUtilities opModeUtilities;

    ColorSensor colorSensor;
    KColor.Color detectedColor;

    KColor currentColors;
    KColor baseColors;

    public ColorDetector(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.colorSensor = opModeUtilities.getHardwareMap().get(ColorSensor.class, "color sensor");

        opModeUtilities.getOpMode().sleep(100);

        baseColors = new KColor(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        Log.d("color detectors", "base set to " + baseColors.getRed() + " " + baseColors.getGreen() + " " + baseColors.getBlue());
        detectedColor = KColor.Color.NONE;
    }

    public KColor.Color detectColor() {

        if(currentColors == null) {
            currentColors = new KColor();
        }

        currentColors.setRed(colorSensor.red());
        currentColors.setGreen(colorSensor.green());
        currentColors.setBlue(colorSensor.blue());

        Log.d("color detector", "current " + currentColors.getRed() + " " + currentColors.getGreen() + " " + currentColors.getBlue());
        Log.d("color detector", "base " + baseColors.getRed() + " " + baseColors.getGreen() + " " + baseColors.getBlue());


        int redOffset = currentColors.getRed() - baseColors.getRed();
        int greenOffset = currentColors.getGreen() - baseColors.getGreen();
        int blueOffset = currentColors.getBlue() - baseColors.getBlue();

        opModeUtilities.getTelemetry().addData("Deltas", redOffset + " " + greenOffset + " " + blueOffset);

        if(redOffset > greenOffset && redOffset > blueOffset && redOffset > 100) {
            detectedColor = KColor.Color.RED;
        } else if (blueOffset > redOffset && blueOffset > greenOffset && blueOffset > 100) {
            detectedColor = KColor.Color.BLUE;
        } else if (greenOffset > blueOffset && greenOffset > redOffset && greenOffset > 100) {
            detectedColor = KColor.Color.YELLOW;
        }

        return detectedColor;

    }

}
