package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

public class ColorSensorOur {

    //add variables
    private final double dalta = 40;
    private final double[] myCol = new double[3];
    private final int gain = 51;
    private final boolean isRightColor = false;
    private int col;
    private boolean isRed = false;
    private OpMode opMode;
    private RevColorSensorV3 colorSensor = null;
    private boolean IS_DEBUG = false;
    private Telemetry telemetry;

    //color sensor constructor
    public ColorSensorOur(OpMode opMode, boolean IS_DEBUG) {
        this.IS_DEBUG = IS_DEBUG;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        colorSensor = opMode.hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.setGain(gain);
        init();
        if (IS_DEBUG) {
            telemetry.addData("(CS)Constructor", true);
        }
    }

    public ColorSensorOur(OpMode opMode) {
        new ColorSensorOur(opMode, false);
    }

    //checks if im red or blue
    public void init() {
        isRed = CheckAlliance.isRed();
        if (IS_DEBUG) {
            telemetry.addData("(CS)Init", true);
        }
    }

    //returns the distance of the object from the sensor in Centimeters
    public double getDistance() {
        if (IS_DEBUG) {
            telemetry.addData("Distance", DistanceUnit.CM);
        }
        return colorSensor.getDistance(DistanceUnit.CM);
    }

    public double getDalta() {
        return dalta;
    }

    public double[] getMyCol() {
        return myCol;
    }

    public int getCol() {
        return col;
    }

    public void setCol(int col) {
        this.col = col;
    }

    public int getGain() {
        return gain;
    }

    public boolean isRed() {
        return isRed;
    }

    public void setRed(boolean red) {
        isRed = red;
    }

    public boolean isRightColor() {
        return isRightColor;
    }

    public OpMode getOpMode() {
        return opMode;
    }

    public void setOpMode(OpMode opMode) {
        this.opMode = opMode;
    }

    public RevColorSensorV3 getColorSensor() {
        return colorSensor;
    }

    public void setColorSensor(RevColorSensorV3 colorSensor) {
        this.colorSensor = colorSensor;
    }

    public boolean isIS_DEBUG() {
        return IS_DEBUG;
    }

    public void setIS_DEBUG(boolean IS_DEBUG) {
        this.IS_DEBUG = IS_DEBUG;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    //checks if the color of the object is right by Specimen or not
    public boolean isRightColor(boolean isSpecimen) {
        col = checkColors();
        if (IS_DEBUG) {
            telemetry.addData("color", col);
        }
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
