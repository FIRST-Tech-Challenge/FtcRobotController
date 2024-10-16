package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

public class ColorSensorOur {

    //add variables
    private final double dalta = 40;
    private final double[] myCol = new double[3];
    RevColorSensorV3 colorSensor = null;
    private int col;
    private final int gain = 51;
    private boolean isRed = false;
    private final boolean isRightColor = false;

    private final OpMode opMode;

//color sensor constructor
    public ColorSensorOur(OpMode opMode) {
        this.opMode = opMode;
        colorSensor = opMode.hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.setGain(gain);
        init();
    }

    //checks if im red or blue
    public void init() {
        isRed = CheckAlliance.isRed();
    }

    //returns the distance of the object from the sensor in Centimeters
    public double getDistance() {
        return colorSensor.getDistance(DistanceUnit.CM);
    }

    //checks if the color of the object is right by Specimen or not
    public boolean isRightColor(boolean isSpecimen) {
        col = checkColors();
        if (isRed && isSpecimen) {
            return col == 0;
        } else if (isRed) {
            return col == 0 || col == 1;

        } else if (isSpecimen) {
            return col == 2;
        } else {
            return col == 2 || col == 1;
        }
    }

    //returns the color that the sensor gets blue yellow or red
    public int checkColors() {
        myCol[0] = colorSensor.getNormalizedColors().red * 255;
        myCol[1] = colorSensor.getNormalizedColors().green * 255;
        myCol[2] = colorSensor.getNormalizedColors().blue * 255;
        if (myCol[0] > (myCol[1] + dalta) && myCol[0] > (myCol[2] + dalta)) {
            return 0;
        } else if (myCol[2] > (myCol[0] + dalta) && myCol[2] > (myCol[1] + dalta)) {
            return 2;
        } else {
            return 1;
        }
    }
}
