package com.kalipsorobotics.modules;

import static java.lang.Thread.sleep;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.utilities.KColor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class ColorDetector {

    OpModeUtilities opModeUtilities;

    ColorSensor colorSensor;

    KColor currentColors;
    KColor baseColors;

    public ColorDetector(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.colorSensor = opModeUtilities.getHardwareMap().get(ColorSensor.class, "intakeColorSensor");

        opModeUtilities.getOpMode().sleep(100);

        baseColors = new KColor(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        Log.d("color detectors", "base set to " + baseColors.getRed() + " " + baseColors.getGreen() + " " + baseColors.getBlue());
    }

    public KColor.Color detectColor() {
        KColor.Color detectedColor;

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

 //       opModeUtilities.getTelemetry().addData("Deltas", redOffset + " " + greenOffset + " " + blueOffset);
        Log.d("Deltas", redOffset + " " + greenOffset + " " + blueOffset);
        if (greenOffset > blueOffset && greenOffset > 20) {
            detectedColor = KColor.Color.YELLOW;
        } else if(redOffset > greenOffset && redOffset > blueOffset && redOffset > 50) {
            detectedColor = KColor.Color.RED;
        } else if (blueOffset > redOffset && blueOffset > greenOffset && blueOffset > 50) {
            detectedColor = KColor.Color.BLUE;
        } else {
            detectedColor = KColor.Color.NONE;
        }
        Log.d("detected color" , "" + detectedColor);
        return detectedColor;

    }
    public boolean detectRed() {
        if (detectColor() == KColor.Color.RED) {
            return true;
        } else {
            return false;
        }
    }
    public boolean detectBlue() {
        if (detectColor() == KColor.Color.BLUE) {
            return true;
        } else {
            return false
        }
    }
    public boolean detectYellow() {
        if (detectColor() == KColor.Color.YELLOW) {
            return true;
        } else {
            return false;
        }
    }
    public void cycle(boolean isRed, boolean takeInYellow, IntakeNoodleAction intakeNoodleAction) {
        if (((!isRed) && detectRed()) ||
                (isRed) && detectBlue()) {
            intakeNoodleAction.reverse();
            SystemClock.sleep(1000);
        } else if (takeInYellow && ((!isRed) && detectRed()) || (detectYellow()) ||
                (isRed) && detectBlue()) {
            intakeNoodleAction.reverse();
            SystemClock.sleep(1000);
        } else {
            intakeNoodleAction.run();
        }
    }
}
