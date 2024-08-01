package org.firstinspires.ftc.teamcode;

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

    public AprilTagMap() {
        tag100 = new AprilTag();
        tag100.setAttributes("Blue Nexus Goal - Field Center - Facing Platform",
                160, 40, 20,
                new int[0]
        );

        tag101 = new AprilTag();
        tag101.setAttributes("Red Nexus Goal - Field Center - Facing Platform",
                160, 60, 30,
                new int[0]
        );

        tag102 = new AprilTag();
        tag102.setAttributes("Red Nexus Goal - Field Center - Facing Food Warehouse",
                160, 80, 40,
                new int[0]
        );

        tag103 = new AprilTag();
        tag103.setAttributes("Blue Nexus Goal - Field Center - Facing Food Warehouse",
                160, 100, 50,
                new int[0]
        );

        tag104 = new AprilTag();
        tag104.setAttributes("Blue Nexus Goal - Field Edge - Alliance Station",
                160, 120, 60,
                new int[0]
        );

        tag105 = new AprilTag();
        tag105.setAttributes("Blue Nexus Goal - Field Edge - Center Field",
                160, 140, 70,
                new int[0]
        );

        tag106 = new AprilTag();
        tag106.setAttributes("Red Nexus Goal - Field Edge - Center Field",
                160, 160, 80,
                new int[0]
        );

        tag107 = new AprilTag();
        tag107.setAttributes("Red Nexus Goal - Field Edge - Alliance Station",
                160, 180, 90,
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
