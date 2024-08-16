package org.firstinspires.ftc.teamcode.maps;

import java.util.HashMap;
import java.util.Map;

public class AprilTagMapping {

    private static final Map<Integer, AprilTag> aprilTags = new HashMap<>();

    static {
        aprilTags.put(100, new AprilTag("Blue Nexus Goal - Field Center - Facing Platform", 1025, 4080, 3911, new int[0]));
        aprilTags.put(101, new AprilTag("Red Nexus Goal - Field Center - Facing Platform", 1025, 2920, 3911, new int[0]));
        aprilTags.put(102, new AprilTag("Red Nexus Goal - Field Center - Facing Food Warehouse", 1025, 2920, 3039, new int[0]));
        aprilTags.put(103, new AprilTag("Blue Nexus Goal - Field Center - Facing Food Warehouse", 1025, 4080, 3039, new int[0]));
        aprilTags.put(104, new AprilTag("Blue Nexus Goal - Field Edge - Alliance Station", 1025, 642, 506, new int[0]));
        aprilTags.put(105, new AprilTag("Blue Nexus Goal - Field Edge - Center Field", 785, 1614, 506, new int[0]));
        aprilTags.put(106, new AprilTag("Red Nexus Goal - Field Edge - Center Field", 785, 5386, 506, new int[0]));
        aprilTags.put(107, new AprilTag("Red Nexus Goal - Field Edge - Alliance Station", 1025, 6358, 506, new int[0]));
    }

    public static Map<Integer, AprilTag> getMap() {
        return aprilTags;
    }

    public int[] getTagLocation(int tagID) throws Exception{
        AprilTag tag = aprilTags.get(tagID);
        if (tag != null) {
            return new int[]{tag.getX(), tag.getY(), tag.getZ()};
        } else {
            throw new Exception("tag does not exist");
        }
    }

}
