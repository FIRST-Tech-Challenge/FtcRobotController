package org.firstinspires.ftc.masters.apriltesting;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class CustomDatabase {

        public static AprilTagLibrary getCurrentGameTagLibrary()
        {
            return new AprilTagLibrary.Builder()
                    .addTags(DataBaseTest())
                    .build();
        }

        public static AprilTagLibrary DataBaseTest()
        {
            return new AprilTagLibrary.Builder()
                    .addTag(0, "Tag 0, 36h11", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(1, "Tag 1, 36h11", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(2, "Tag 2, 36h11", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .build();
        }
}
