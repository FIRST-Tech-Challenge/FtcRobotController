package org.firstinspires.ftc.teamcode.BBcode;

import com.acmerobotics.roadrunner.Pose2d;

import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.RedBasketPose;

public class PoseStorage {
        // See this static keyword? That's what lets us share the data between opmodes.
        public final static Pose2d center = new Pose2d(0, 0, 0); //This is the pose to use for a manual drive to and button press style reset
        public static Pose2d currentPose = RedBasketPose.basket_init_old; //This is a default pose to use for manual testing. It must be set by autonomous to use auto functions in teleop
        public static boolean hasFieldCentricDrive = false; //This should only be true if we knew our starting location or have set our location in some other way
        public static boolean arePosesEqual(Pose2d pose1, Pose2d pose2) {
        return pose1.position.x == pose2.position.x &&
                pose1.position.y == pose2.position.y &&
                pose1.heading.toDouble() == pose2.heading.toDouble();
        }
        public static boolean hasRolloverPose() {
                //return !arePosesEqual(currentPose, center) && previousOpMode == OpModeType.AUTONOMOUS;
                return true;
        }

        public static OpModeType previousOpMode = OpModeType.UNKNOWN;

}
