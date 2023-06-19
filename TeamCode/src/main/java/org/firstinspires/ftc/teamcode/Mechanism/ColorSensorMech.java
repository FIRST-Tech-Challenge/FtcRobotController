package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HWMap;

import java.util.ArrayList;


public class ColorSensorMech extends HWMap {

    public double leftBlue;
    public double rightBlue;
    public double middleBlue;
    public double rightRed;
    public double leftRed;
    public double middleRed;

    /*
    Color Values:
    V3: 550
    V2: 100
     */
    private double V3colorValue;
    private double V2colorValue;

    public ColorSensorMech(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
        V3colorValue = 550;
        V2colorValue = 150;
    }


    public double isDetectingBlueLine() {

        leftBlue = leftCS.blue();
        middleBlue = middleCS.blue();
        rightBlue = rightCS.blue();

        if(middleBlue > V3colorValue && rightBlue > V3colorValue && leftBlue < V3colorValue){
            return 1;
        } else if(middleBlue > V3colorValue && rightBlue < V2colorValue && leftBlue > V3colorValue) {
            return 0.001; //center because of gripper
        } else if (middleBlue < V3colorValue && rightBlue < V2colorValue && leftBlue > V3colorValue){
            return -1;
        } else if(middleBlue < V3colorValue && rightBlue > V2colorValue && leftBlue < V3colorValue){
            return 1;
        }else if(middleBlue > V3colorValue && rightBlue < V2colorValue && leftBlue < V3colorValue){
            return 1;
        } else{
            return -0.001;
        }

/*        if (leftBlue < colorValue && middleBlue >= colorValue && rightBlue < colorValue) {

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

        return false;*/

    }

/*    public boolean isDetectingRedLine() {

        leftRed = leftCS.red();
        middleRed = middleCS.red();
        rightRed = rightCS.red();

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
    }*/
}
