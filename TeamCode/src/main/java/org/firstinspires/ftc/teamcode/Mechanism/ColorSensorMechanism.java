/*
package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanism;

import java.util.ArrayList;


public class ColorSensorMechanism extends Mechanism {

    public ColorSensor colorSensorLeft;
    public ColorSensor colorSensorMiddle;
    public ColorSensor colorSensorRight;
    public double leftBlue;
    public double middleBlue;
    public double rightBlue;
    public double leftRed;
    public double middleRed;
    public double rightRed;

    public double colorValue;

    public ColorSensorMechanism(Telemetry telemetry, HardwareMap hardwareMap) {

        super(telemetry, hardwareMap);

        colorSensorLeft = this.hardwareMap.colorSensor.get("left");
        colorSensorMiddle = this.hardwareMap.colorSensor.get("middle");
        colorSensorRight = this.hardwareMap.colorSensor.get("right");

        colorValue = 3000;
    }

    public boolean isDetectingBlueLine() {

        leftBlue = colorSensorLeft.blue();
        middleBlue = colorSensorMiddle.blue();
        rightBlue = colorSensorRight.blue();

        if (leftBlue < colorValue && middleBlue >= colorValue && rightBlue < colorValue) {

            telemetry.addLine("The robot is currently on the path of the blue tape");

            telemetry.addData("Color Sensor Left Blue ", leftBlue);
            telemetry.addData("Color Sensor Middle Blue Value ", middleBlue);
            telemetry.addData("Color Sensor Right Blue Value ", rightBlue);

            telemetry.update();

            return true;

        }

        telemetry.addLine("The robot is not currently on the path of the blue tape");

        telemetry.addData("Color Sensor Left Blue ", leftBlue);
        telemetry.addData("Color Sensor Middle Blue Value ", middleBlue);
        telemetry.addData("Color Sensor Right Blue Value ", rightBlue);

        telemetry.update();

        return false;

    }

    public boolean isDetectingRedLine() {

        leftRed = colorSensorLeft.red();
        middleRed = colorSensorMiddle.red();
        rightRed = colorSensorRight.red();

        if (leftRed < colorValue && middleRed >= colorValue && rightRed < colorValue) {

            telemetry.addLine("The robot is currently on the path of the red tape");

            telemetry.addData("Color Sensor Left Red ", leftRed);
            telemetry.addData("Color Sensor Middle Red Value ", middleRed);
            telemetry.addData("Color Sensor Right Red Value ", rightRed);

            telemetry.update();

            return true;

        }

        telemetry.addLine("The robot is not currently on the path of the red tape");

        telemetry.addData("Color Sensor Left Red ", leftRed);
        telemetry.addData("Color Sensor Middle Red Value ", middleRed);
        telemetry.addData("Color Sensor Right Red Value ", rightRed);

        telemetry.update();

        return false;

    }

    public void correctMovement() {

    }

    public ArrayList<String>  colorValues(){
        ArrayList<String> colorArray = new ArrayList<>();

        if(leftBlue >= colorValue){
            colorArray.add("left");
        }
        if(rightBlue >= colorValue){
            colorArray.add("right");
        }
        if(middleBlue >= colorValue){
            colorArray.add("middle");
        }

        return colorArray;
    }
}
*/
