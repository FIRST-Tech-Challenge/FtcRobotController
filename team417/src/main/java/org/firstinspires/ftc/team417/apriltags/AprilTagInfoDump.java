package org.firstinspires.ftc.team417.apriltags;

/*
    Stores all the info about AprilTags, including location on the field, size, ID, tilt, etc.
    Used in AprilTagDetectionConcept
*/

import org.firstinspires.ftc.team417.utilities.AprilTagInfo;

public class AprilTagInfoDump {

    //Stores the location of the AprilTags in an array
    public static final AprilTagInfo[] AprilTagInfoArray = new AprilTagInfo[] {
            // These measurements come from consulting Game Manual 2, field construction instructions,
            //    and then measuring the locations of the April tags on each sheet of paper.
            // Note that these measurements are accurate up to the limits of my ability
            //    (up to +- 1/16 inches) and can double-checked but probably not improved upon
            new AprilTagInfo(7, 5, -72.125, -41.1875, 6.4375),
            new AprilTagInfo(10, 5, -72.125, 41.1875, 6.4375),
            new AprilTagInfo(8, 2, -72.125, -36, 4),
            new AprilTagInfo(9, 2, -72.125, 36, 4),
    };

    // Finds the april tag with a certain ID
    public static AprilTagInfo findTagWithId(int id) {
        for (AprilTagInfo aprilTagInfo : AprilTagInfoArray) {
            if (aprilTagInfo.id == id) {
                return aprilTagInfo;
            }
        }
        return null;
    }
}