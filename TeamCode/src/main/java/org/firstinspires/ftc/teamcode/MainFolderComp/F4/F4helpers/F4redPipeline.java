package org.firstinspires.ftc.teamcode.MainFolderComp.F4.F4helpers;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;


public class F4redPipeline extends OpenCvPipeline {

    List<Integer> ELEMENT_COLOR = Arrays.asList(255, 0, 0); //(red, green, blue)


    String color_zone = "nothing";

    int toggleShow = 1;


    Mat zone1;
    Mat zone2;
    Mat zone3; // control

    Scalar avgColor1;
    Scalar avgColor2;
    Scalar avgColor3;

    double distance1;
    double distance2;
    double distance3;

    double max_distance;

    @Override
    public void init(Mat input) {

        zone1 = input.submat(new Rect(0, 170, 110, 140));
        zone2 = input.submat(new Rect(345, 210, 150, 175));
        zone3 = input.submat(new Rect(410, 400, 60, 50));



    }



    @Override
    public Mat processFrame(Mat input) {

        if (toggleShow == -1) {
            return input;
        }

        //Averaging the colors in the zones
        avgColor1 = Core.mean(zone1);
        avgColor2 = Core.mean(zone2);
        avgColor3 = Core.mean(zone3);

        //Putting averaged colors on zones (we can see on camera now)
        zone1.setTo(avgColor1);
        zone2.setTo(avgColor2);
        zone3.setTo(avgColor3);

        distance1 = color_distance(avgColor1, ELEMENT_COLOR);
        distance2 = color_distance(avgColor2, ELEMENT_COLOR);
        distance3 = color_distance(avgColor3, ELEMENT_COLOR);

        max_distance = Math.min(distance3, Math.min(distance1, distance2));

        if  (max_distance == distance1) {
            //telemetry.addData("Element is on the left side", distance3);
            color_zone = "left";
        } else if (max_distance == distance2) {
            //telemetry.addData("Element is on right side", distance1);
            color_zone = "middle";

        } else if (max_distance == distance3) {
            //telemetry.addData("Element is in the middle", distance2);
            color_zone = "right";
        }

        return input;

    }

    public double color_distance(Scalar color1, List color2) {
        double r1 = color1.val[0];
        double g1 = color1.val[1];
        double b1 = color1.val[2];

        int r2 = (int) color2.get(0);
        int g2 = (int) color2.get(1);
        int b2 = (int) color2.get(2);

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }


    public String getLocation() {
        return color_zone;
    }

    public void toggleCVonoff() {
        toggleShow = toggleShow * -1;
    }

}