package org.firstinspires.ftc.masters;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class SkystoneDatabase {

        public static AprilTagLibrary getCurrentGameTagLibrary()
        {
            return new AprilTagLibrary.Builder()
                    .addTags(DataBaseTest())
                    .build();
        }

        public static AprilTagLibrary DataBaseTest()
        {
            return new AprilTagLibrary.Builder()
                    .addTag(1, "blue_Left_Board", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(2, "blue_Middle_Board", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(3, "blue_Right_Board", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(4, "red_Left_Board", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(5, "red_Middle_Board", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(6, "red_Right_Board", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(1, "Tag 0, 36h11", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(2, "Tag 1, 36h11", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(3, "Tag 2, 36h11", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(1, "Tag 0, 36h11", 100, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .build();
        }
}
