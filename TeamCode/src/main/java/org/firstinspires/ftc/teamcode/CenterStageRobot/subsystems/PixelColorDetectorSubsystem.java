package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.hardware.ColorSensor;

public class PixelColorDetectorSubsystem extends SubsystemBase {
    private ColorSensor frontSensor; //, backSensor;

    public enum PixelColor {
        NONE, // 1, 5, 14
        WHITE, // 31, 64, 71
        YELLOW, // 15, 25, 8
        PURPLE, // 14, 24, 42
        GREEN // 5, 19, 9
    }

    private PixelColor frontPixelColor = PixelColor.NONE;
    private PixelColor backPixelColor = PixelColor.NONE;

    private int frontPixelExistence = 0, backPixelExistence = 0; // Binary

    private int numOfPixels = 0;

    private Telemetry telemetry;

    public PixelColorDetectorSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.telemetry = telemetry;

        frontSensor = new ColorSensor(hm, "front_color_sensor");
//        backSensor = new ColorSensor(hm, "back_color_sensor");
    }

    public PixelColor predictColor(double redCh, double greenCh, double blueCh) {
        double average = (redCh + greenCh + blueCh) / 3;

        if (average > 45) {
            return PixelColor.WHITE;
        } else if (greenCh > redCh && greenCh > blueCh && redCh > blueCh) {
            return PixelColor.YELLOW;
        } else if (greenCh > redCh && greenCh > blueCh) {
            return PixelColor.GREEN;
        } else if (blueCh > redCh && blueCh > greenCh && greenCh > redCh && average > 15) {
            return PixelColor.PURPLE;
        }

        return PixelColor.NONE;
    }

    @Override
    public void periodic() {
        frontPixelColor = predictColor(frontSensor.getRed(), frontSensor.getGreen(), frontSensor.getBlue());
//        backPixelColor = predictColor(backSensor.getRed(), backSensor.getGreen(), backSensor.getBlue());

        frontPixelExistence = frontPixelColor != PixelColor.NONE ? 1 : 0;
        backPixelExistence = backPixelColor != PixelColor.NONE ? 1 : 0;

        numOfPixels = frontPixelExistence + backPixelExistence;

//        telemetry.addData("Front Red: ", frontSensor.getRed());
//        telemetry.addData("Front Green: ", frontSensor.getGreen());
//        telemetry.addData("Front Blue: ", frontSensor.getBlue());
//        telemetry.addData("Back Red: ", backSensor.getRed());
//        telemetry.addData("Back Green: ", backSensor.getGreen());
//        telemetry.addData("Back Blue: ", backSensor.getBlue());
        telemetry.addData("Front Color Prediction: ", frontPixelColor);
        telemetry.addData("Back Color Prediction: ", backPixelColor);
    }

    public PixelColor getFrontPixelColor() {
        return frontPixelColor;
    }

    public PixelColor getBackPixelColor() {
        return backPixelColor;
    }

    public boolean isFrontPixel() {
        return frontPixelExistence == 1;
    }

    public boolean isBackPixel() {
        return backPixelExistence == 1;
    }

    public int getNumOfPixels() {
        return numOfPixels;
    }
}
