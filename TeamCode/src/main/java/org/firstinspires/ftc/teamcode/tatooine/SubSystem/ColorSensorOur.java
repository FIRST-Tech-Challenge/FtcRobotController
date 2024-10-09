package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

public class ColorSensorOur {

    RevColorSensorV3 colorSensor = null;
    private final double dalta = 40;
    private int col;
    private final double[] myCol = new double[3];
    private int gain = 51;
    private boolean isRed = false;


    public ColorSensorOur(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.setGain(gain);
    }

    public void init() {
        isRed = CheckAlliance.isRed();
    }

    public boolean isRightColorForSample() {
        checkColors();
        if (isRed) {
            if (col == 0) {
                return true;
            } else if (col == 1) {
                return true;
            } else {
                return true;
            }
        } else if (!isRed) {
            if (col == 2) {
                return true;
            } else if (col == 1) {
                return true;
            } else {
                return false;
            }
        }
        else {
            return true;
        }

    }
    public boolean isRightColorForSpecimen(){
        checkColors();
        isRed = CheckAlliance.isRed();
        if (isRed) {
            if (col == 0) {
                return true;
            } else {
                return false;
            }
        } else if (!isRed) {
            if (col == 2) {
                return true;
            }
                else {
                return false;
            }
        }
        else {
            return true;
        }
    }

    public void checkColors() {
        myCol[0] = colorSensor.getNormalizedColors().red * 255;
        myCol[1] = colorSensor.getNormalizedColors().green * 255;
        myCol[2] = colorSensor.getNormalizedColors().blue * 255;
        if (myCol[0] > (myCol[1] + dalta) && myCol[0] > (myCol[2] + dalta)) {
            col = 0;
        } else if (myCol[2] > (myCol[0] + dalta) && myCol[2] > (myCol[1] + dalta)) {
            col = 2;
        } else {
            col = 1;
        }
    }
}
