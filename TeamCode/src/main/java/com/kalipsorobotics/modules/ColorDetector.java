package com.kalipsorobotics.modules;

import static java.lang.Thread.sleep;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.utilities.KColor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ColorDetector {

    OpModeUtilities opModeUtilities;

    ColorSensor colorSensor;

    KColor currentColors;
    KColor baseColors;
    RevLED revLED;

    public ColorDetector(OpModeUtilities opModeUtilities, HardwareMap hardwareMap) {
        this.opModeUtilities = opModeUtilities;
        this.colorSensor = opModeUtilities.getHardwareMap().get(ColorSensor.class, "intakeColorSensor");

        opModeUtilities.getOpMode().sleep(100);
        revLED = new RevLED(hardwareMap,"red1", "green1", "red2","green2","red3","green3","red4","green4");
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
            return false;
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
        if (takeInYellow && ((!isRed) && detectRed()) ||
                (isRed) && detectBlue()) { //if wants yellow and blue and sees red OR if wants red and sees blue
            intakeNoodleAction.reverse();
            revLED.turnOnRed();
        } else if (!takeInYellow && ((!isRed) && detectRed()) || (detectYellow()) ||
                (isRed) && detectBlue()) { //if does not want yellow and wants blue and sees red OR sees yellow OR wants red and sees blue
            intakeNoodleAction.reverse();
            revLED.turnOnRed();
        } else if (detectColor() == KColor.Color.NONE){ //if no color
            intakeNoodleAction.run();
            revLED.turnoff();
        } else { //
            intakeNoodleAction.run();
            revLED.turnOnGreen();
        }
    }
}
