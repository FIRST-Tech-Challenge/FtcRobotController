package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

public class ColorSensorOur {

    RevColorSensorV3 colorSensor = null;
    private final double dalta = 40;
    private int col;
    private final double[] myCol = new double[3];
    private int gain = 51;
    private boolean isRed = false;
    private boolean isRightColor = false;

    private OpMode opMode;


    public ColorSensorOur(OpMode opMode) {
        this.opMode = opMode;
        colorSensor = opMode.hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.setGain(gain);
        init();
    }

    public void init() {
        isRed = CheckAlliance.isRed();
    }

    public double getDistance (){
        return colorSensor.getDistance(DistanceUnit.CM);
    }

    public boolean isRightColor(boolean isSpecimen) {
        col = checkColors();
        opMode.telemetry.addData("col", col);
            if (col == 0 && isRed) {
                return true;
            } else if (col == 2 && !isRed) {
                return true;
            } else if (col ==1 && isSpecimen) {
                return true;
            } else {
                return false;
            }
        }


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
