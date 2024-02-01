package org.firstinspires.ftc.teamcode.utility;

public enum AprilTagLocation {
    NO_VALUE(0),
    BLUE_LEFT(1),
    BLUE_CENTRE(2),
    BLUE_RIGHT(3),
    RED_LEFT(4),
    RED_CENTRE(5),
    RED_RIGHT(6);
    int tagVal;
    AprilTagLocation(int i) {
        tagVal = i;
    }
    public int TagNum(){
        return tagVal;
    }
}
