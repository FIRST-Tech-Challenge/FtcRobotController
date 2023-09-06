package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

public class LineFollowerSensors implements SubSystem {

    NormalizedColorSensor [] line_sensors;

    public LineFollowerSensors(NormalizedColorSensor[] line_sensors) {
        this.line_sensors = line_sensors;
    }

    public final static int LEFT = 1;
    public final static int RIGHT = -1;
    public final static int STEADY = 0;

    public final static int RED=0;
    public final static int GREEN=1;
    public final static int BLUE=2;

    public int check(int color) {
        // so assumption for this is that we are already on the line
        // if on the line, and left sensor is not move left, ditto right, otherwise steady onward.
        // sensors are inside the line
        NormalizedRGBA left = line_sensors[0].getNormalizedColors();
        NormalizedRGBA right = line_sensors[1].getNormalizedColors();

        //check left
        int left_value=0, right_value=0;
        switch (color) {
            case RED:
                left_value = (left.red < .5)?1:0;
                right_value = (right.red< .5)?1:0;
                break;
            case GREEN:
                left_value = (left.green < .5)?1:0;
                right_value = (right.green<.5)?1:0;
                break;
            case BLUE:
                left_value = (left.blue < .5)?1:0;
                right_value = (right.blue<.5)?1:0;
                break;
        }
        return Range.clip(left_value - right_value, -1,1);
    }
}
