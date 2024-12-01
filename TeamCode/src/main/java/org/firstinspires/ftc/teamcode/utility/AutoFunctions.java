package org.firstinspires.ftc.teamcode.utility;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.RobotContainer;

// These functions are used to mirror
public class AutoFunctions {

    // mirror provided pose2d for red vs blue team
    public static Pose2d redVsBlue(Pose2d pose) {
        if (RobotContainer.isRedAlliance)
            return new Pose2d (-pose.getX(),
                            -pose.getY(),
                            new Rotation2d(pose.getHeading()-Math.PI));
        else
            return pose;
    }

    // mirror provided translation2d for red vs blue team
    public static Translation2d redVsBlue(Translation2d translation) {
        if (RobotContainer.isRedAlliance)
            return new Translation2d (-translation.getX(),
                    -translation.getY());
        else
            return translation;
    }

    // mirror provided rotation2d for red vs blue team
    public static Rotation2d redVsBlue(Rotation2d rotation) {
        if (RobotContainer.isRedAlliance)
            return new Rotation2d (rotation.getRadians() - Math.PI);
        else
            return rotation;
    }

}
