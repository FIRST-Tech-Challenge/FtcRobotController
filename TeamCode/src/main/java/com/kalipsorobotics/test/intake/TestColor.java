package com.kalipsorobotics.test.intake;

import com.kalipsorobotics.modules.KColor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class TestColor extends LinearOpMode {

    // Define a variable for our color sensor
    ColorSensor colorSensor;
    KColor.Color detectedColor;

    KColor currentColors;
    KColor baseColors;


//    public Color detectColor() {
//        if(colorSensor.red() > 1.1*colorSensor.green() && colorSensor.red() > 2*colorSensor.blue()) {
//            return Color.RED;
//        } else if (colorSensor.green() > 1.25*colorSensor.red() && colorSensor.green() > 2*colorSensor.blue()) {
//            return Color.YELLOW;
//        } else if (colorSensor.blue() > 1.25*colorSensor.green() && colorSensor.blue() > 2*colorSensor.red()) {
//            return Color.BLUE;
//        }
//
//        return Color.NONE;
//    }

    public KColor.Color detectColor() {
        int redOffset = currentColors.getRed() - baseColors.getRed();
        int greenOffset = currentColors.getGreen() - baseColors.getGreen();
        int blueOffset = currentColors.getBlue() - baseColors.getBlue();

        telemetry.addData("Deltas", redOffset + " " + greenOffset + " " + blueOffset);

        if(redOffset > greenOffset && redOffset > blueOffset && redOffset > 100) {
            detectedColor = KColor.Color.RED;
        } else if (blueOffset > redOffset && blueOffset > greenOffset && blueOffset > 100) {
            detectedColor = KColor.Color.BLUE;
        } else if (greenOffset > blueOffset && greenOffset > redOffset && greenOffset > 100) {
            detectedColor = KColor.Color.YELLOW;
        }

        return detectedColor;
    }

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        colorSensor = hardwareMap.get(ColorSensor.class, "color sensor");

        baseColors = new KColor(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        currentColors = new KColor();

        // Wait for the Play button to be pressed
        waitForStart();

        // While the OpMode is running, update the telemetry values.
        while (opModeIsActive()) {

            currentColors.setRed(colorSensor.red());
            currentColors.setGreen(colorSensor.green());
            currentColors.setBlue(colorSensor.blue());

            telemetry.addData("Current colors", colorSensor.red() + " " + colorSensor.green() + " " + colorSensor.blue());

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Color Detected", detectColor());
            telemetry.update();

        }
    }

}
