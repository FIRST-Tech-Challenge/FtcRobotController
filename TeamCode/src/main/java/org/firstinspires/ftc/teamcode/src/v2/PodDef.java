package org.firstinspires.ftc.teamcode.src.v2;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;

public class PodDef {

    Translation2d left_pod_location = new Translation2d();
    Translation2d right_pod_location = new Translation2d();

    SwerveDriveKinematics differential_swerve_kinematics = new SwerveDriveKinematics(left_pod_location, right_pod_location);

}
