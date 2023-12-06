package org.firstinspires.ftc.masters.apriltesting;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class SkystoneDatabase {

        public static AprilTagLibrary getCurrentGameTagLibrary()
        {
            return new AprilTagLibrary.Builder()
                    .addTags(SkystoneDatabase())
                    .build();
        }

        public static AprilTagLibrary SkystoneDatabase()
        {
            return new AprilTagLibrary.Builder()
                    .addTag(1, "blue_Left_Board", 50.8, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(2, "blue_Middle_Board", 50.8, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(3, "blue_Right_Board", 50.8, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(4, "red_Left_Board", 50.8, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(5, "red_Middle_Board", 50.8, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(6, "red_Right_Board", 50.8, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(10, "blue_Large_Backplate", 127, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(9, "blue_Small_Backplate", 50.8, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(7, "red_Large_Backplate", 127, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .addTag(8, "red_Small_Backplate", 50.8, new VectorF(0,0,0), DistanceUnit.MM, Quaternion.identityQuaternion())
                    .build();
        }
}
