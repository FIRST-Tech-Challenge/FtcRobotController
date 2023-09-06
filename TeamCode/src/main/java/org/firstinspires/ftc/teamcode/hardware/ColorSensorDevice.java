package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorDevice {
    public static final int RED   = 1;
    public static final int GREEN = 2;
    public static final int BLUE  = 3;

    private int last_check=0;

    private final ColorSensor _colorSensor;

    public ColorSensorDevice(ColorSensor colorSensor) {
        _colorSensor = colorSensor;
    }

    public boolean checkOnLine(int color, int threshold) {
        int value=0;
        switch (color) {
            case BLUE:
                value = _colorSensor.blue();
                break;
            case GREEN:
                value = _colorSensor.green();
                break;
            case RED:
                value = _colorSensor.red();
                break;
        }
        if ( value  < last_check - threshold ) {
            last_check = 0;
            return true;
        }
        last_check = value;
        return false;
    }

    public int findMaxColor() {
        int red = _colorSensor.red();
        int blue = _colorSensor.blue();
        int green = _colorSensor.green();
        if ((red > blue) &&  (red > green)) return RED;
        else if (blue > green) return BLUE;
        else return GREEN;
    }

    }
