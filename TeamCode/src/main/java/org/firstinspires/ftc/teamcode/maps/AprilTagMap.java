package org.firstinspires.ftc.teamcode.maps;

import org.checkerframework.checker.units.qual.A;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class AprilTagMap {
    // All units are in mm

    // The field orientation is with the food reservoir at the back
    //int width = 7000; // 0 is left
    //int length = 7000; // 0 is top
    // 0:0 is top left at the blue field edge nexus goal


    AprilTag tag100 = new AprilTag();
    AprilTag tag101 = new AprilTag();
    AprilTag tag102 = new AprilTag();
    AprilTag tag103 = new AprilTag();
    AprilTag tag104 = new AprilTag();
    AprilTag tag105 = new AprilTag();
    AprilTag tag106 = new AprilTag();
    AprilTag tag107 = new AprilTag();

    Map<Integer, AprilTag> aprilTags = new HashMap <Integer, AprilTag>();



    public AprilTagMap() { //all dimensions are from the center of the apriltag
        {
            aprilTags.put(100, tag100);
            aprilTags.put(101, tag101);
            aprilTags.put(102, tag102);
            aprilTags.put(103, tag103);
            aprilTags.put(104, tag104);
            aprilTags.put(105, tag105);
            aprilTags.put(106, tag106);
            aprilTags.put(107, tag107);
        } // creating a map that can be accessed thru their id
        //dimensions are with a .3 mm accuracy according to the step file provided by FIRST
        {
            tag100.setAttributes("Blue Nexus Goal - Field Center - Facing Platform",
                    1025, 4080, 3911,
                    new int[0]
            );

            tag101.setAttributes("Red Nexus Goal - Field Center - Facing Platform",
                    1025, 2920, 3911,
                    new int[0]
            );

            tag102.setAttributes("Red Nexus Goal - Field Center - Facing Food Warehouse",
                    1025, 2920, 3039,
                    new int[0]
            );

            tag103.setAttributes("Blue Nexus Goal - Field Center - Facing Food Warehouse",
                    1025, 4080, 3039,
                    new int[0]
            );

            tag104.setAttributes("Blue Nexus Goal - Field Edge - Alliance Station",
                    1025, 642, 506,
                    new int[0]
            );

            tag105.setAttributes("Blue Nexus Goal - Field Edge - Center Field",
                    785, 1614, 506,
                    new int[0]
            );

            tag106.setAttributes("Red Nexus Goal - Field Edge - Center Field",
                    785, 5386, 506,
                    new int[0]
            );

            tag107.setAttributes("Red Nexus Goal - Field Edge - Alliance Station",
                    1025, 6358, 506,
                    new int[0]
            );
        } // aprilTags

    }
    public int[] getTagLocation(int tagID){

        return new int[] {0,0,0};

    }
}

class AprilTag {
    String name;
    int height;
    int x;
    int y;
    int[] detectionArea;

    public void setAttributes(String namePassed, int heightPassed, int xPassed, int yPassed, int[] detectionAreaPassed) {
        name = namePassed;
        height = heightPassed;
        x = xPassed;
        y = yPassed;
        detectionArea = detectionAreaPassed;
    }

    public String getName() {
        return name;
    }

    public int getHeight() {
        return height;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public boolean getDetectionArea() {
        return true;
    }
}
