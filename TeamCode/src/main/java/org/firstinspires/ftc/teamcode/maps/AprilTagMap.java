package org.firstinspires.ftc.teamcode.maps;

public class AprilTagMap {
    // All units are in mm

    // The field orientation is with the food reservoir at the back
    int width = 7000; // 0 is left
    int length = 7000; // 0 is top
    // 0:0 is top left at the blue field edge nexus goal

    AprilTag tag100;
    AprilTag tag101;
    AprilTag tag102;
    AprilTag tag103;
    AprilTag tag104;
    AprilTag tag105;
    AprilTag tag106;
    AprilTag tag107;

    public AprilTagMap() { //all dimensions are from the center of the apriltag
        //dimensions are with a .3 mm accuracy according to the step file provided by FIRST
        tag100 = new AprilTag();
        tag100.setAttributes("Blue Nexus Goal - Field Center - Facing Platform",
                1025, 4080, 3911,
                new int[0]
        );

        tag101 = new AprilTag();
        tag101.setAttributes("Red Nexus Goal - Field Center - Facing Platform",
                1025, 2920, 3911,
                new int[0]
        );

        tag102 = new AprilTag();
        tag102.setAttributes("Red Nexus Goal - Field Center - Facing Food Warehouse",
                1025, 2920, 3039,
                new int[0]
        );

        tag103 = new AprilTag();
        tag103.setAttributes("Blue Nexus Goal - Field Center - Facing Food Warehouse",
                1025, 4080, 3039,
                new int[0]
        );

        tag104 = new AprilTag();
        tag104.setAttributes("Blue Nexus Goal - Field Edge - Alliance Station",
                1025, 642, 506,
                new int[0]
        );

        tag105 = new AprilTag();
        tag105.setAttributes("Blue Nexus Goal - Field Edge - Center Field",
                785, 1614, 506,
                new int[0]
        );

        tag106 = new AprilTag();
        tag106.setAttributes("Red Nexus Goal - Field Edge - Center Field",
                785, 5386, 506,
                new int[0]
        );

        tag107 = new AprilTag();
        tag107.setAttributes("Red Nexus Goal - Field Edge - Alliance Station",
                1025, 6358, 506,
                new int[0]
        );
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
